#!/usr/bin/env python3
"""
test_chassis_control.py - PickOrangeDummy 移动底盘控制与联调测试脚本

用途:
    用于在 VENTUNO Q (Linux Ubuntu 24.04) 或本地 Mac 开发机上，
    测试与验证小车底盘的 3/4/5 引脚 PWM 控制信号、换向互锁及安全看门狗。

运行模式:
    1. 键盘实时遥控模式 (默认): WASD 驾驶小车，空格刹车，X 停机回正。
    2. 自动标定验证套件 (--auto-test): 自动执行 T0~T5 验收用例并输出报告。
    3. 单次指令模式 (--drive / --set-pwm): 单次下发目标速度或指定微秒脉宽。
    4. 模拟仿真模式 (--mock): 无真机硬件时模拟 STM32 固件与时序，验证全链路逻辑。

接线基准 (VENTUNO Q UNO 排母):
    - 3 号脚: ~D3 (油门/刹车)
    - 4 号脚: ~D5 (转向)
    - 5 号脚: ~D9 (倒车/方向)
    - 地线: GND
"""

import os
import sys
import time
import glob
import select
import termios
import tty
import argparse
import threading
from typing import Optional, Tuple


class ChassisController:
    """底盘上位机控制器，管理串口通信与安全心跳。"""

    def __init__(self, port: Optional[str] = None, baud: int = 115200, mock: bool = False):
        self.port = port
        self.baud = baud
        self.mock = mock
        self.ser = None
        self.running = False
        self.lock = threading.Lock()

        # 当前控制目标
        self.target_speed = 0   # -100 ~ 100
        self.target_steer = 0   # -100 ~ 100
        self.is_braking = False

        # 遥测反馈
        self.last_thr_us = 1500
        self.last_str_us = 1500
        self.last_rev_us = 1500
        self.last_state_msg = "IDLE"

        # 心跳线程
        self.heartbeat_thread = None

        self._init_connection()

    def _find_default_port(self) -> Optional[str]:
        """自动查找 VENTUNO Q 虚拟串口或 USB-CDC 端口。"""
        candidates = [
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyUSB0",
            "/dev/cu.usbmodem*",
            "/dev/serial/by-id/*"
        ]
        for pattern in candidates:
            matches = glob.glob(pattern)
            if matches:
                return matches[0]
        return None

    def _init_connection(self):
        if self.mock:
            print("[INFO] 模拟模式已启用 (Mock Mode: 虚拟 STM32 固件链路已就绪)")
            return

        if not self.port:
            self.port = self._find_default_port()

        if not self.port:
            print("[WARN] 未检测到硬件串口设备，自动降级为仿真模式 (--mock)。")
            self.mock = True
            return

        try:
            import serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
            time.sleep(1.5)  # 等待板载 MCU 复位完成
            print(f"[INFO] 成功连接至 VENTUNO Q 串口: {self.port} @ {self.baud} bps")
        except Exception as e:
            print(f"[WARN] 串口连接失败 ({e})，切换为仿真模式 (--mock)。")
            self.mock = True

    def start_heartbeat(self, freq: float = 20.0):
        """启动后台心跳线程，持续以指定频率发送控制帧喂狗 (防 300ms 超时)。"""
        self.running = True
        interval = 1.0 / freq

        def _loop():
            while self.running:
                with self.lock:
                    speed = self.target_speed
                    steer = self.target_steer
                    brake = self.is_braking

                if brake:
                    self.send_raw_cmd("BRAKE")
                elif speed == 0 and steer == 0:
                    self.send_raw_cmd("STOP")
                else:
                    self.send_raw_cmd(f"DRIVE {speed} {steer}")

                time.sleep(interval)

        self.heartbeat_thread = threading.Thread(target=_loop, daemon=True)
        self.heartbeat_thread.start()

    def stop_heartbeat(self):
        """停止心跳并紧急制动归零。"""
        self.running = False
        if self.heartbeat_thread and self.heartbeat_thread.is_alive():
            self.heartbeat_thread.join(timeout=0.5)
        self.send_raw_cmd("STOP")

    def set_drive(self, speed: int, steer: int):
        """设置期望速度和转向 (-100 ~ 100)。"""
        with self.lock:
            self.target_speed = max(-100, min(100, speed))
            self.target_steer = max(-100, min(100, steer))
            self.is_braking = False

    def trigger_brake(self):
        """触发紧急制动。"""
        with self.lock:
            self.target_speed = 0
            self.is_braking = True

    def trigger_neutral(self):
        """自然停机回正。"""
        with self.lock:
            self.target_speed = 0
            self.target_steer = 0
            self.is_braking = False

    def send_raw_cmd(self, cmd_str: str) -> str:
        """发送单条 ASCII 指令并接收应答。"""
        if self.mock:
            return self._mock_handle(cmd_str)

        if not self.ser or not self.ser.is_open:
            return "ERR_PORT_NOT_OPEN"

        try:
            self.ser.write(f"{cmd_str}\n".encode("utf-8"))
            resp = self.ser.readline().decode("utf-8", errors="ignore").strip()
            self._parse_ack(resp)
            return resp
        except Exception as e:
            return f"ERR_{e}"

    def _parse_ack(self, resp: str):
        # 解析 "ACK DRIVE thr=1600 str=1500 rev=1500"
        if "thr=" in resp and "str=" in resp and "rev=" in resp:
            try:
                parts = resp.split()
                for p in parts:
                    if p.startswith("thr="):
                        self.last_thr_us = int(p.split("=")[1])
                    elif p.startswith("str="):
                        self.last_str_us = int(p.split("=")[1])
                    elif p.startswith("rev="):
                        self.last_rev_us = int(p.split("=")[1])
                self.last_state_msg = resp
            except Exception:
                pass

    def _mock_handle(self, cmd_str: str) -> str:
        """仿真内核：模拟 STM32 固件的换向互锁与脉宽映射。"""
        if cmd_str.startswith("DRIVE"):
            parts = cmd_str.split()
            spd = int(parts[1]) if len(parts) > 1 else 0
            str_val = int(parts[2]) if len(parts) > 2 else 0

            # 转向 1100 ~ 1900
            str_us = 1500 if str_val == 0 else int(1500 + str_val * 4)
            str_us = max(1100, min(1900, str_us))

            # 油门/倒车
            if spd > 0:
                thr_us = int(1525 + spd * 4.75)
                rev_us = 1500
            elif spd < 0:
                thr_us = 1500
                rev_us = int(1525 + abs(spd) * 4.75)
            else:
                thr_us = 1400  # 刹车
                rev_us = 1500

            self.last_thr_us = thr_us
            self.last_str_us = str_us
            self.last_rev_us = rev_us
            resp = f"ACK DRIVE thr={thr_us} str={str_us} rev={rev_us}"
            self.last_state_msg = resp
            return resp
        elif cmd_str.startswith("SET"):
            parts = cmd_str.split()
            if len(parts) >= 4:
                self.last_thr_us = int(parts[1])
                self.last_str_us = int(parts[2])
                self.last_rev_us = int(parts[3])
                resp = f"ACK SET thr={self.last_thr_us} str={self.last_str_us} rev={self.last_rev_us}"
                self.last_state_msg = resp
                return resp
        elif cmd_str == "BRAKE":
            self.last_thr_us = 1400
            self.last_rev_us = 1500
            resp = "ACK BRAKE thr=1400 str=1500 rev=1500"
            self.last_state_msg = resp
            return resp
        elif cmd_str == "STOP":
            self.last_thr_us = 1500
            self.last_str_us = 1500
            self.last_rev_us = 1500
            resp = "ACK STOP thr=1500 str=1500 rev=1500"
            self.last_state_msg = resp
            return resp
        elif cmd_str == "PING":
            return "PONG"

        return "ERR UNKNOWN_CMD"

    def close(self):
        self.stop_heartbeat()
        if self.ser and self.ser.is_open:
            self.ser.close()


def run_interactive_teleop(ctrl: ChassisController):
    """交互式键盘控制模式 (WASD)。"""
    print("\n" + "=" * 65)
    print("   PickOrangeDummy 移动底盘交互测试控制台 (WASD Teleop)")
    print("=" * 65)
    print("  按键说明:")
    print("    [W]   油门递增 (+10%)      [S]   油门递减 / 倒车 (-10%)")
    print("    [A]   向左打舵 (-15%)      [D]   向右打舵 (+15%)")
    print("    [空格] 主动紧急刹车        [X]   全归中位 (回正/停机)")
    print("    [Q]   安全退出程序")
    print("=" * 65)
    print("  ⚠️ 安全提示: 确保小车驱动轮已完全悬空架起！\n")

    ctrl.start_heartbeat(freq=20.0)

    speed = 0
    steer = 0

    # 终端无缓冲按键捕获配置
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            # 打印实时状态行
            mode_tag = "[MOCK]" if ctrl.mock else "[REAL]"
            sys.stdout.write(
                f"\r{mode_tag} 目标:[速:{speed:+4d}%, 舵:{steer:+4d}%] | "
                f"输出脉宽:[3号油门:{ctrl.last_thr_us:4d}us, 4号转向:{ctrl.last_str_us:4d}us, 5号倒车:{ctrl.last_rev_us:4d}us]  "
            )
            sys.stdout.flush()

            # 非阻塞按键检测
            if select.select([sys.stdin], [], [], 0.05)[0]:
                char = sys.stdin.read(1).lower()

                if char == 'w':
                    speed = min(100, speed + 10)
                    ctrl.set_drive(speed, steer)
                elif char == 's':
                    speed = max(-100, speed - 10)
                    ctrl.set_drive(speed, steer)
                elif char == 'a':
                    steer = max(-100, steer - 15)
                    ctrl.set_drive(speed, steer)
                elif char == 'd':
                    steer = min(100, steer + 15)
                    ctrl.set_drive(speed, steer)
                elif char == ' ':
                    speed = 0
                    ctrl.trigger_brake()
                elif char == 'x':
                    speed = 0
                    steer = 0
                    ctrl.trigger_neutral()
                elif char == 'q':
                    print("\n[INFO] 用户退出测试控制台。")
                    break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        ctrl.stop_heartbeat()
        print("\n[INFO] 控制器已安全回中停机。")


def run_auto_test_suite(ctrl: ChassisController):
    """自动执行 docs/chassis_wiring_and_control_spec.md §6 中规定的标准标定流程。"""
    print("\n" + "=" * 65)
    print("   PickOrangeDummy 移动底盘自动标定与验收套件 (T0 ~ T5)")
    print("=" * 65)

    suite = [
        ("T0: 基准中位检查", "STOP", 1500, 1500, 1500, 1.5),
        ("T1: 转向居中与左右限位", "SET 1500 1100 1500", 1500, 1100, 1500, 1.5),
        ("T1-2: 转向右打死限位", "SET 1500 1900 1500", 1500, 1900, 1500, 1.5),
        ("T1-3: 转向回正", "SET 1500 1500 1500", 1500, 1500, 1500, 1.0),
        ("T2: 慢速前进 (30%)", "DRIVE 30 0", 1667, 1500, 1500, 2.0),
        ("T2-2: 中速前进 (60%)", "DRIVE 60 0", 1810, 1500, 1500, 2.0),
        ("T3: 主动刹车测试", "BRAKE", 1400, 1500, 1500, 1.5),
        ("T4: 换向与倒车测试 (40%)", "DRIVE -40 0", 1500, 1500, 1715, 2.0),
        ("T5: 停机归零安全复位", "STOP", 1500, 1500, 1500, 1.0),
    ]

    all_passed = True
    for name, cmd, exp_thr, exp_str, exp_rev, duration in suite:
        print(f"\n[测试项] {name}")
        print(f"  -> 下发指令: {cmd}")
        resp = ctrl.send_raw_cmd(cmd)
        print(f"  <- 固件响应: {resp}")

        # 检查容差 (允许 ±25us 浮点转换偏差)
        pass_thr = abs(ctrl.last_thr_us - exp_thr) <= 25
        pass_str = abs(ctrl.last_str_us - exp_str) <= 25
        pass_rev = abs(ctrl.last_rev_us - exp_rev) <= 25

        if pass_thr and pass_str and pass_rev:
            print(f"  [PASS] 输出脉宽符合规范: Thr={ctrl.last_thr_us}us, Str={ctrl.last_str_us}us, Rev={ctrl.last_rev_us}us")
        else:
            print(f"  [FAIL] 期望 [{exp_thr}, {exp_str}, {exp_rev}], 实际 [{ctrl.last_thr_us}, {ctrl.last_str_us}, {ctrl.last_rev_us}]")
            all_passed = False

        time.sleep(duration)

    print("\n" + "=" * 65)
    if all_passed:
        print("  🎉 全部标定测试用例验证通过！底盘电气时序符合规范。")
    else:
        print("  ❌ 部分测试项未通过，请排查接线或定时器配置。")
    print("=" * 65)


def main():
    parser = argparse.ArgumentParser(description="PickOrangeDummy 移动底盘控制测试工具")
    parser.add_argument("--port", type=str, default=None, help="VENTUNO Q 串口设备路径 (默认自动查找)")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率 (默认 115200)")
    parser.add_argument("--mock", action="store_true", help="强制启用虚拟仿真模式 (无硬件测试)")
    parser.add_argument("--auto-test", action="store_true", help="执行自动化 T0~T5 标定测试套件")
    parser.add_argument("--drive", nargs=2, type=int, metavar=("SPEED", "STEER"), help="单次测试: 设置速度(-100~100)与打角(-100~100)")
    parser.add_argument("--set-pwm", nargs=3, type=int, metavar=("THR", "STR", "REV"), help="单次测试: 直接写入 3/4/5 引脚脉宽 (1000~2000)")

    args = parser.parse_args()

    ctrl = ChassisController(port=args.port, baud=args.baud, mock=args.mock)

    try:
        if args.set_pwm:
            thr, str_val, rev = args.set_pwm
            print(f"[SET] 注入脉宽: 油门={thr}us, 转向={str_val}us, 倒车={rev}us")
            resp = ctrl.send_raw_cmd(f"SET {thr} {str_val} {rev}")
            print(f"响应: {resp}")
        elif args.drive:
            spd, str_val = args.drive
            print(f"[DRIVE] 下发速度: {spd}%, 转向: {str_val}%")
            resp = ctrl.send_raw_cmd(f"DRIVE {spd} {str_val}")
            print(f"响应: {resp}")
        elif args.auto_test:
            run_auto_test_suite(ctrl)
        else:
            run_interactive_teleop(ctrl)
    finally:
        ctrl.close()


if __name__ == "__main__":
    main()
