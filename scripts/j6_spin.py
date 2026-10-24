#!/usr/bin/env python3
"""
j6_spin.py - Dummy 机械臂第六关节 (J6) 单方向匀速慢转

默认行为: 以 5 °/s 一直朝同一个方向匀速转动, 不折返, 直到固件行程边界自动停下。

关键实现要点 (均为真机踩坑后验证):
1. 必须持续读取串口回复。固件每条 `>` 指令都会回 "%d" 与 "ok", 若只写不读,
   REF 主控 USB-CDC 发送缓冲会被撑满导致指令线程阻塞, 表现为
   「第一次能跑, Ctrl+C 之后再启动无响应 / 写串口报 EAGAIN」。
2. 退出时不要发 !STOP。!STOP 会把固件 isEnabled 置 false 并清空 FIFO
   (见 dummy_robot.cpp 的 EmergencyStop), 改为「按实测角度就地悬停」,
   状态机保持健康, 可无限次反复启停。
3. J6 固件行程为 ±720° (实测 725°/730° 指令被静默拒绝, 719° 可执行)。
   到边界后固件仍然回 "ok" 但不动作, 因此脚本用「实测角度是否变化」判断到界,
   到界后干净退出, 绝不反向。
4. 目标角每周期都从实测角度重新推导, 不靠内部累加, 避免漂移;
   同时对单周期角度突变做保护, 防止编码器/多圈计数抖动导致突然飞车。
5. 串口清除 HUPCL 并保持 DTR 低电平, 防止关闭串口触发 STM32 冷复位。

用法:
    python3 j6_spin.py                 # 5 °/s, 自动选剩余行程大的方向, 一直转
    python3 j6_spin.py --dir +         # 强制正方向
    python3 j6_spin.py --dps 3         # 3 °/s
    python3 j6_spin.py --disable       # 退出后释放力矩
"""

import os
import glob
import time
import termios
import argparse

FW_LIMIT = 720.0        # J6 固件行程 ±720°
SPEED_TO_DPS = 0.72     # 实测标定: 固件 speed=15 -> 约 10.8 °/s

# 固件对各关节的目标角校验区间 (超出任何一个, 整条 > 指令会被静默丢弃, 机械臂完全不动)
FW_RANGES = [
    (-170.0, 170.0),    # J1
    (-75.0, 90.0),      # J2
    (35.0, 180.0),      # J3
    (-180.0, 180.0),    # J4
    (-95.0, 95.0),      # J5
    (-720.0, 720.0),    # J6
]


def clamp(value, index):
    lo, hi = FW_RANGES[index]
    return max(lo, min(hi, value))


def find_ref_port():
    ports = glob.glob('/dev/serial/by-id/*REF*')
    if ports:
        return ports[0]
    for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'):
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'


class RefSerial:
    """REF 主控极简非阻塞串口 (仅标准库)"""

    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[2] &= ~termios.HUPCL      # 关闭时不拉 DTR, 防 STM32 复位
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            pass
        time.sleep(0.1)
        self.drain()

    def drain(self):
        """清空接收缓冲, 防止固件 USB 发送阻塞 (核心)"""
        try:
            while os.read(self.fd, 4096):
                pass
        except Exception:
            pass

    def write(self, line):
        if not line.endswith('\n'):
            line += '\n'
        data = line.encode('utf-8')
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            self.drain()
            time.sleep(0.02)
            try:
                os.write(self.fd, data)
            except OSError:
                pass
        except OSError:
            pass

    def read_text(self, timeout=0.12):
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

    def joints(self, retries=8):
        for _ in range(retries):
            self.drain()
            self.write('#GETJPOS')
            for line in self.read_text(0.2).splitlines():
                line = line.strip()
                if line.startswith('ok'):
                    parts = line.split()
                    if len(parts) >= 7:
                        try:
                            return [float(x) for x in parts[1:7]]
                        except ValueError:
                            pass
            time.sleep(0.04)
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


def main():
    ap = argparse.ArgumentParser(description='Dummy 机械臂 J6 单方向匀速慢转')
    ap.add_argument('--dps', type=float, default=5.0, help='转速 (度/秒, 默认 5)')
    ap.add_argument('--dir', choices=['auto', '+', '-'], default='auto',
                    help='转动方向, auto = 选剩余行程更大的一侧 (默认)')
    ap.add_argument('--deg', type=float, default=None,
                    help='只做一次相对转动并退出, 例如 --deg -2 表示反向转 2°')
    ap.add_argument('--port', type=str, default='', help='串口路径 (默认自动查找 REF)')
    ap.add_argument('--disable', action='store_true', help='退出后释放力矩 (默认保持使能悬停)')
    args = ap.parse_args()

    dps = max(0.5, min(60.0, abs(args.dps)))
    speed = max(1.0, min(100.0, dps / SPEED_TO_DPS))

    port = args.port or find_ref_port()
    print(f'[连接] {port}')
    ser = RefSerial(port)

    try:
        ser.write('!START')
        ser.read_text(0.12)

        j = ser.joints()
        if j is None:
            print('[错误] 读取关节角失败: 检查 USB 连接 / 供电 / 是否被其他程序占用')
            return
        print(f'[姿态] J1={j[0]:.2f} J2={j[1]:.2f} J3={j[2]:.2f} '
              f'J4={j[3]:.2f} J5={j[4]:.2f} J6={j[5]:.2f}   (J1~J5 全程锁定)')

        # 方向: 单方向, 永不折返; auto 只决定一开始往哪边
        if args.dir == '+':
            direction = 1.0
        elif args.dir == '-':
            direction = -1.0
        else:
            up_room = FW_LIMIT - j[5]
            down_room = j[5] + FW_LIMIT
            direction = 1.0 if up_room >= down_room else -1.0

        remain = (FW_LIMIT - j[5]) if direction > 0 else (j[5] + FW_LIMIT)
        print(f'[运行] {dps:.1f} °/s, 方向 {"正向" if direction > 0 else "负向"} (不折返), '
              f'剩余行程约 {remain:.0f}° ≈ {remain / dps / 60:.1f} 分钟')

        # 若 J1~J5 有超出行程的关节, 固件会丢弃整条指令 -> 机械臂完全不动,
        # 因此下发时统一夹到合法区间, 并明确告知(该关节会先被收回合法区间)
        out_of_range = [(i, j[i]) for i in range(5)
                        if j[i] < FW_RANGES[i][0] - 0.1 or j[i] > FW_RANGES[i][1] + 0.1]
        for i, val in out_of_range:
            lo, hi = FW_RANGES[i]
            print(f'[注意] J{i + 1} 当前 {val:.2f}° 超出固件区间 [{lo:.0f}, {hi:.0f}]'
                  f' (固件会因此丢弃全部运动指令), 已按 {clamp(val, i):.2f}° 下发,'
                  f' 该关节会先被慢慢收回区间内')
        if out_of_range:
            print('       回收期间 J6 转速会被拉慢, 属正常现象')

        # ---- 单步相对转动模式: --deg -2 就是反向转 2° 后退出 ----
        if args.deg is not None:
            start = j[5]
            target = clamp(start + args.deg, 5)
            lock = [clamp(j[i], i) for i in range(5)]
            print(f'[单步] J6 {start:.2f}° -> {target:.2f}° (相对 {args.deg:+.2f}°)')
            ser.write(f'>{lock[0]:.2f},{lock[1]:.2f},{lock[2]:.2f},'
                      f'{lock[3]:.2f},{lock[4]:.2f},{target:.2f},{speed:.1f}')
            t0 = time.time()
            last = start
            cur = j
            while time.time() - t0 < 40:
                time.sleep(0.1)
                cur = ser.joints(retries=1) or cur
                if abs(cur[5] - target) <= 0.05:
                    break
                if abs(cur[5] - last) > 0.01:
                    last = cur[5]
                    t0 = time.time()
            cur = ser.joints(retries=2) or cur
            print(f'[完成] J6 = {cur[5]:.2f}°, 实际变化 {cur[5] - start:+.2f}°')
            return

        print('      按 Ctrl+C 平滑停止\n')

        interval = 0.2          # 指令下发周期
        lookahead = 0.6         # 前瞻时间, 保证固件一直有运动目标 => 匀速
        step = dps * lookahead

        last_meas = j[5]        # 上一次可信角度
        last_move_t = time.time()
        last_print = time.time()
        last_print_pos = j[5]
        jump_guard = max(3.0 * step, 15.0)   # 单周期最大可信变化量

        while True:
            cur = ser.joints(retries=1)
            now = time.time()
            if cur is None:
                time.sleep(interval)
                continue

            j6 = cur[5]

            # 角度突变保护: 复核一次, 确认是真实跳变才接受
            if abs(j6 - last_meas) > jump_guard:
                time.sleep(0.05)
                again = ser.joints(retries=2)
                if again is not None:
                    if abs(again[5] - j6) <= 1.0:
                        print(f'[警告] J6 读数跳变 {last_meas:.1f}° -> {j6:.1f}°, '
                              f'按新读数继续 (可能是多圈计数复位)')
                    else:
                        time.sleep(interval)
                        continue

            # 到界判断: 已无剩余行程 / 或角度长时间不变化
            at_limit = (direction > 0 and j6 >= FW_LIMIT - 0.5) or \
                       (direction < 0 and j6 <= -FW_LIMIT + 0.5)
            stalled = (abs(j6 - last_meas) < 0.02) and (now - last_move_t > 2.0)

            if at_limit:
                print(f'\n[结束] J6 已到行程边界 (当前 {j6:.1f}°, 固件限位 ±{FW_LIMIT:.0f}°), '
                      f'单方向转动停止。')
                print('       如需继续同方向转动, 可先用 --dir 反向跑回另一端。')
                break

            if stalled:
                # 不是边界却完全不动: 说明指令被固件丢弃, 打印诊断信息
                bad = [f'J{i + 1}={cur[i]:.2f}° (允许 {FW_RANGES[i][0]:.0f}~{FW_RANGES[i][1]:.0f})'
                       for i in range(6)
                       if cur[i] < FW_RANGES[i][0] - 0.1 or cur[i] > FW_RANGES[i][1] + 0.1]
                print('\n[结束] 固件没有执行运动指令, J6 停在 '
                      f'{j6:.1f}° (未到 ±{FW_LIMIT:.0f}° 边界)。')
                if bad:
                    print('       超出行程的关节: ' + ', '.join(bad))
                print('       请检查: 1) 机械臂是否处于 !START 使能状态; '
                      '2) 是否有其他程序(ros2_control/moveit)占用了串口。')
                break

            # 目标角始终由实测角推导: 不累积漂移, 也不会因读数扰动而飞车
            target = j6 + direction * step
            target = max(-FW_LIMIT, min(FW_LIMIT, target))

            # J1~J5 必须落在固件合法区间内, 否则整条指令会被丢弃
            lock = [clamp(cur[i], i) for i in range(5)]
            ser.write(f'>{lock[0]:.2f},{lock[1]:.2f},{lock[2]:.2f},'
                      f'{lock[3]:.2f},{lock[4]:.2f},{target:.2f},{speed:.1f}')

            if abs(j6 - last_meas) >= 0.02:
                last_move_t = now

            if now - last_print >= 2.0:
                dt = max(now - last_print, 1e-6)
                print(f'    J6 = {j6:8.2f}°   实测 {((j6 - last_print_pos) / dt):+6.2f} °/s')
                last_print = now
                last_print_pos = j6

            last_meas = j6
            time.sleep(interval)

    except KeyboardInterrupt:
        print('\n[停止] 收到 Ctrl+C, 就地平滑悬停...')
        cur = ser.joints(retries=3)
        if cur:
            hold = [clamp(cur[i], i) for i in range(6)]
            ser.write(f'>{hold[0]:.2f},{hold[1]:.2f},{hold[2]:.2f},'
                      f'{hold[3]:.2f},{hold[4]:.2f},{hold[5]:.2f},20')
            ser.read_text(0.1)
            print(f'[悬停] J6={cur[5]:.2f}°, 机械臂保持使能, 可随时重新运行本脚本')
        if args.disable:
            ser.write('!DISABLE')
            ser.read_text(0.1)
            print('[失能] 已释放力矩')
    except Exception as exc:
        print(f'\n[异常] {exc}')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
