#!/usr/bin/env python3
"""
cut.py - J6 旋切刀片闭环剪切工具（装刀版）

语义（见 docs/M2_spec_j6_blade_home.md）:
    J6 = 0°    刀片全开（剪切起始）
    J6 = -60°  刀片合拢（剪切末尾）
    剪切流程: 0° -> -60° -> 保压 -> 0°

与旧版的区别（旧版已废止）:
    - 旧版 `>0,0,0,0,0,360,50` + 固定 sleep，装刀后会整车转一圈、且会截断在半路；
    - 新版只动 J6，J1~J5 用实测值锁死，行程硬限制在 [-60, 0]，
      并用 #GETJPOS 闭环确认到位，绝不靠 sleep 猜。

安全前置:
    1. ros2_control / controller_manager 在跑时拒绝执行（驱动层会把 J6 覆盖回 cutter_target_pos_）
    2. 全部目标角先过固件区间夹紧（越界会让固件静默丢弃整条指令）
    3. 静止检测: 目标与实测差 < 0.02° 不下发（规避固件 MoveJ 的 0/0 = NaN）

用法:
    python3 scripts/cut.py              # 完整剪切: 合刀 -> 保压 -> 回位
    python3 scripts/cut.py --close      # 只合刀
    python3 scripts/cut.py --open       # 只回位
    python3 scripts/cut.py --dwell 0.5  # 合刀后保压时长(秒)
    python3 scripts/cut.py --dry-run    # 只打印计划，不动硬件
"""

import os
import glob
import time
import termios
import argparse
import json
import subprocess

# ---- 刀片零点配置（软件偏移；固件 #OFFSET_J 在本机无效）----
CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'config', 'j6_blade.json')


def load_blade_config():
    home, span = 0.0, 60.0
    try:
        with open(CONFIG_PATH, 'r') as f:
            cfg = json.load(f)
        home = float(cfg.get('j6_home_deg', home))
        span = float(cfg.get('cut_span_deg', span))
    except Exception:
        pass
    return home, span


J6_HOME, CUT_SPAN = load_blade_config()
CUT_RANGE = (J6_HOME - CUT_SPAN, J6_HOME)   # 合刀到底 .. 全开

CUT_SPEED = 5.0          # 固件 speed 参数（约 3.6°/s，60° 约 17s）
MOVE_SPEED = 10.0        # 非剪切移动速度
POS_TOL = 0.2            # 到位容差(度)
MOVE_TIMEOUT = 30.0      # 单段超时(秒)

FW_RANGES = [
    (-170.0, 170.0), (-75.0, 90.0), (35.0, 180.0),
    (-180.0, 180.0), (-95.0, 95.0), (-720.0, 720.0),
]


def find_ref_port():
    ports = glob.glob('/dev/serial/by-id/*REF*')
    if ports:
        return ports[0]
    for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'):
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'


def ros_control_running():
    """驱动层在跑时 J6 会被覆盖回 cutter_target_pos_，必须拒绝执行。"""
    try:
        out = subprocess.run(['pgrep', '-af', 'ros2_control_node|controller_manager'],
                             capture_output=True, text=True, timeout=3)
        return out.stdout.strip()
    except Exception:
        return ''


class RefSerial:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            pass
        time.sleep(0.15)
        self.drain()

    def drain(self):
        """清空接收缓冲。**必须有界**：设备异常时可能持续输出，
        无界 while 会把工作线程永久卡死（2026-10-24 真机复现）。"""
        for _ in range(64):
            try:
                if not os.read(self.fd, 4096):
                    break
            except Exception:
                break

    def write(self, line):
        if not line.endswith('\n'):
            line += '\n'
        try:
            os.write(self.fd, line.encode())
        except BlockingIOError:
            self.drain()
            time.sleep(0.02)
            try:
                os.write(self.fd, line.encode())
            except OSError:
                pass

    def read_text(self, timeout=0.25):
        buf = b''
        end = time.time() + timeout
        while time.time() < end:
            try:
                chunk = os.read(self.fd, 4096)
                if chunk:
                    buf += chunk
                    end = time.time() + 0.02
            except Exception:
                time.sleep(0.005)
        return buf.decode('utf-8', errors='ignore')

    def joints(self, retries=4):
        for _ in range(retries):
            self.drain()
            self.write('#GETJPOS')
            for l in self.read_text(0.25).splitlines():
                l = l.strip()
                if l.startswith('ok'):
                    parts = l.split()
                    if len(parts) >= 7:
                        try:
                            return [float(x) for x in parts[1:7]]
                        except ValueError:
                            pass
        return None

    def close(self):
        try:
            termios.tcdrain(self.fd)
        except Exception:
            pass
        try:
            os.close(self.fd)
        except Exception:
            pass


def clamp(v, i):
    lo, hi = FW_RANGES[i]
    return max(lo, min(hi, v))


class Cutter:
    def __init__(self, ser, dry_run=False):
        self.ser = ser
        self.dry_run = dry_run

    def move_j6(self, target, speed, label):
        """把 J6 移到 target（绝对值），J1~J5 用实测值锁死。返回是否到位。"""
        lo, hi = CUT_RANGE
        if not (lo - 1e-6 <= target <= hi + 1e-6):
            print(f'[拒绝] {label}: 目标 {target:+.2f}° 超出剪切行程 [{lo:+.0f}, {hi:+.0f}]°')
            return False

        cur = self.ser.joints()
        if cur is None:
            print(f'[失败] {label}: 读不到关节角')
            return False

        if abs(cur[5] - target) < 0.02:
            print(f'[跳过] {label}: J6 已在 {cur[5]:+.2f}°')
            return True

        cmd = (f'>{clamp(cur[0], 0):.2f},{clamp(cur[1], 1):.2f},{clamp(cur[2], 2):.2f},'
               f'{clamp(cur[3], 3):.2f},{clamp(cur[4], 4):.2f},{target:.2f},{speed:.1f}')
        print(f'[下发] {label}: J6 {cur[5]:+.2f}° -> {target:+.2f}° (speed={speed:.0f})')
        if self.dry_run:
            print(f'        {cmd}')
            return True

        self.ser.write(cmd)
        t0 = time.time()
        cur = self.ser.joints()
        last_pos = cur[5] if cur else None
        last_move = time.time()
        while time.time() - t0 < MOVE_TIMEOUT:
            time.sleep(0.1)
            j = self.ser.joints(retries=1)
            if j and abs(j[5] - target) <= POS_TOL:
                print(f'[到位] {label}: J6 = {j[5]:+.2f}°（{time.time() - t0:.1f}s）')
                return True
            # 堵转/卡滞保护：指令已下发但 1.5s 内几乎不动 -> 立即停止下发并回持当前位置
            if j and last_pos is not None:
                if abs(j[5] - last_pos) > 0.15:
                    last_pos, last_move = j[5], time.time()
                elif time.time() - last_move > 1.5:
                    hold = [clamp(j[i], i) for i in range(6)]
                    self.ser.write(f'>{hold[0]:.2f},{hold[1]:.2f},{hold[2]:.2f},'
                                   f'{hold[3]:.2f},{hold[4]:.2f},{hold[5]:.2f},{MOVE_SPEED:.0f}')
                    print(f'[卡滞] {label}: J6 停在 {j[5]:+.2f}° 已 {time.time() - last_move:.0f}s 无位移，'
                          f'已停止下发（电机可能在堵转）。请断电检查螺丝/机械阻力。')
                    return False
        j = self.ser.joints(retries=2)
        if j:
            print(f'[超时] {label}: {MOVE_TIMEOUT:.0f}s 内未到位，当前 J6 = {j[5]:+.2f}°')
        else:
            print(f'[超时] {label}: {MOVE_TIMEOUT:.0f}s 内未到位，且读不到关节角')
        return False


def main():
    ap = argparse.ArgumentParser(description='J6 刀片闭环剪切工具')
    ap.add_argument('--close', action='store_true', help='只合刀到 -60°')
    ap.add_argument('--open', action='store_true', help='只回位到 0°')
    ap.add_argument('--dwell', type=float, default=0.3, help='合刀后保压时长(秒)，默认 0.3')
    ap.add_argument('--speed', type=float, default=CUT_SPEED, help=f'剪切速度，默认 {CUT_SPEED:.0f}')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划，不动硬件')
    ap.add_argument('--port', type=str, default='', help='串口（默认自动查找 REF）')
    args = ap.parse_args()

    busy = ros_control_running()
    if busy:
        print('[拒绝] 检测到 ros2_control / controller_manager 正在运行：')
        print('       ' + busy.splitlines()[0])
        print('       驱动层会把 J6 覆盖回 cutter_target_pos_，剪切期间必须独占串口。')
        print('       请先停止 ROS 控制栈（或改用 ROS 侧接口，见规格 §10 技术债）。')
        return 2

    port = args.port or find_ref_port()
    print(f'[连接] {port}')
    ser = RefSerial(port)
    cutter = Cutter(ser, dry_run=args.dry_run)

    try:
        ser.write('!START')
        ser.read_text(0.2)
        cur = ser.joints()
        if cur is None:
            print('[错误] 读取关节角失败，中止')
            return 1
        print('[姿态] J1~J6 = ' + ' '.join(f'{v:.2f}' for v in cur))

        if not CUT_RANGE[0] - 1 <= cur[5] <= CUT_RANGE[1] + 1:
            print(f'[注意] J6 当前 {cur[5]:+.2f}° 不在剪切行程 {CUT_RANGE} 内，先回零位')

        if args.close and not args.open:
            steps = [('合刀', CUT_RANGE[0], float(args.speed), args.dwell)]
        elif args.open and not args.close:
            steps = [('回位', J6_HOME, MOVE_SPEED, 0.2)]
        else:
            steps = [('合刀', CUT_RANGE[0], float(args.speed), args.dwell),
                     ('回位', J6_HOME, MOVE_SPEED, 0.2)]

        ok = True
        for label, target, speed, dwell in steps:
            if not cutter.move_j6(target, speed, label):
                ok = False
                break
            if dwell > 0 and not args.dry_run:
                time.sleep(dwell)

        if ok:
            print('>>> 剪切完成：J6 已回到 0°（刀片全开）<<<')
            return 0
        print('>>> 剪切未完成，请检查上面的告警 <<<')
        return 1
    except KeyboardInterrupt:
        print('\n[中断] 停止下发，保持当前位置')
        return 130
    finally:
        ser.close()


if __name__ == '__main__':
    raise SystemExit(main())
