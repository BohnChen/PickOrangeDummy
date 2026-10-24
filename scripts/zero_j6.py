#!/usr/bin/env python3
"""
zero_j6.py - 把 J6 当前物理位置固化为永久零点（写电机 EEPROM）

背景（详见 docs/M2_spec_j6_blade_home.md）:
  刀片已装到 J6。约定 0° = 刀片全开（剪切起始），-60° = 合刀到底。
  固件 #OFFSET_J 6 会触发电机 ApplyPosAsHomeOffset() 并落盘 EEPROM，
  与固件自带的 #CALIBRATION 是同一机制，只是只作用于 6 号电机。

安全设计（三条，缺一不可）:
  1) 探测: #OFFSET_J 99（节点越界）。命令存在时只回执、不下发 CAN、不写 EEPROM。
  2) 零位与目标下发必须放在**同一次 write** 内:
        #OFFSET_J 6
        >j1,j2,j3,j4,j5,<eps>,1
     否则控制环(200Hz)可能插入一条「陈旧 targetJoints」，电机会按绝对目标补转 |旧角度|。
  3) 目标带 eps(默认 -0.2°) 偏移:
       - 规避固件 MoveJ 在零位移时 0/0 = NaN 的速度限幅；
       - eps 取合刀侧，避免顶在「全开」方向的限位上。
  4) 若零位后 J6 出现 >5° 的意外位移，立即 !STOP。

用法:
     python3 scripts/zero_j6.py            # 干跑: 只做探测与打印计划，不动硬件
     python3 scripts/zero_j6.py --apply    # 真正执行零位
     python3 scripts/zero_j6.py --apply --eps -0.2 --speed 1
"""

import os
import glob
import time
import termios
import argparse

# 固件对各关节目标角的校验区间（J6 保持 ±720，多圈能力不能动）
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


class RefSerial:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[2] &= ~termios.HUPCL      # 防关闭串口触发 STM32 复位
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

    def write_raw(self, data: bytes):
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            self.drain()
            time.sleep(0.02)
            try:
                os.write(self.fd, data)
            except OSError:
                pass

    def read_text(self, timeout=0.4):
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

    def ask(self, line, timeout=0.6):
        self.drain()
        self.write_raw((line + '\n').encode())
        return self.read_text(timeout).strip()

    def joints(self, retries=6):
        for _ in range(retries):
            resp = self.ask('#GETJPOS', 0.3)
            for l in resp.splitlines():
                l = l.strip()
                if l.startswith('ok'):
                    parts = l.split()
                    if len(parts) >= 7:
                        try:
                            return [float(x) for x in parts[1:7]]
                        except ValueError:
                            pass
            time.sleep(0.05)
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


def main():
    ap = argparse.ArgumentParser(description='J6 永久零位工具')
    ap.add_argument('--apply', action='store_true', help='真正执行（默认只干跑）')
    ap.add_argument('--eps', type=float, default=-0.2, help='零位后 J6 目标偏移（度，默认 -0.2）')
    ap.add_argument('--speed', type=float, default=1.0, help='零位后下发速度（默认 1，最小）')
    ap.add_argument('--port', type=str, default='', help='串口（默认自动查找 REF）')
    args = ap.parse_args()

    port = args.port or find_ref_port()
    print(f'[连接] {port}')
    ser = RefSerial(port)

    try:
        # ---------- 1) 使能（'#' 命令仅在使能后才会进入命令分支）----------
        print('[使能] !START ->', ser.ask('!START', 1.0) or '(无回执)')

        before = ser.joints()
        if before is None:
            print('[错误] 读取关节角失败，中止（未做任何修改）')
            return
        print('[姿态] J1~J6 = ' + ' '.join(f'{v:.2f}' for v in before))

        # ---------- 2) 只读探测 ----------
        probe = ser.ask('#OFFSET_J 99', 0.8)
        print(f'[探测] #OFFSET_J 99 -> {probe!r}')
        if not probe:
            print('[中止] 固件无任何回执，判定不支持 #OFFSET_J（未做任何修改）')
            return
        if 'HOMEOFFSET' in probe.upper():
            print('[探测] 命令存在且带越界校验')
        else:
            print('[探测] 回执为通用 ok：命令可能被通用兜底分支吞掉，零位是否生效需由第 4 步校验判定')

        # ---------- 3) 计划 ----------
        eps = args.eps
        lock = [clamp(before[i], i) for i in range(5)]
        target = eps
        batch = (f'#OFFSET_J 6\n'
                 f'>{lock[0]:.2f},{lock[1]:.2f},{lock[2]:.2f},'
                 f'{lock[3]:.2f},{lock[4]:.2f},{target:.2f},{args.speed:.1f}\n')
        print('\n[计划] 同一次 write 内连发:')
        for line in batch.splitlines():
            print('        ' + line)
        print(f'[计划] 零位后 J6 期望读数 ≈ {target:+.2f}°（原读数 {before[5]:.2f}°）')

        if not args.apply:
            print('\n[干跑] 未执行。确认无误后加 --apply 真正写入 EEPROM。')
            return

        # ---------- 4) 执行 ----------
        print('\n[执行] 写入零位 ...')
        ser.drain()
        ser.write_raw(batch.encode())
        time.sleep(0.3)
        ser.read_text(0.3)

        reads = []
        for _ in range(3):
            j = ser.joints()
            if j:
                reads.append(j)
            time.sleep(0.25)

        if not reads:
            print('[失败] 零位后读不到关节角，请人工检查链路')
            return

        after = reads[-1]
        print('[结果] ' + ' '.join(f'{v:.2f}' for v in after))
        d_j6 = after[5] - before[5]
        d_other = max(abs(after[i] - before[i]) for i in range(5))

        # 成功判据：零位后 J6 应落在 0 附近。
        # 容差 1.5° 的原因：ε 目标在 CAN 上可能与 #OFFSET_J 竞争，
        # 极端情况下 ε 会作用在新坐标系的反侧，读数约 +|before|-ε，仍属零位成功。
        if abs(after[5]) <= 1.5 and d_other <= 0.5:
            print(f'[成功] J6 零点已固化：{before[5]:.2f}° -> {after[5]:+.2f}°'
                  f'（目标 {target:+.2f}°，其余关节变化 ≤{d_other:.2f}°）')
            print('       请断电重启后复测: !START -> #GETJPOS，J6 应仍为 ≈0。')
            return

        # ---------- 5) 失败处置 ----------
        print(f'[未生效] J6 变化 {d_j6:+.2f}°，未达到目标 {target:+.2f}°。')
        if abs(d_j6) > 5.0:
            print('[危险] 出现 >5° 意外位移，立即 !STOP')
            ser.ask('!STOP', 1.0)
        else:
            print('       判定 #OFFSET_J 在本机固件未生效（静默忽略）。')
            print('       改用软件偏移兜底: cutter_target_pos_ = 当前实测值, 并禁用固件 !HOME。')
    except KeyboardInterrupt:
        print('\n[中断] 用户中止')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
