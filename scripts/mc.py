#!/usr/bin/env python3
"""底盘单步测试工具（跑在 App 容器里，靠 mc.sh 调起）

用法：
    mc.py status                  # 读 MCU 状态
    mc.py stop                    # 立即停
    mc.py steer  40   1.5         # 只打转向 40%，保持 1.5 s（车不该动，只动轮）
    mc.py forward 12  2.0         # 前进 12%，2 s
    mc.py back    12  2.0         # 倒车 12%，2 s
    mc.py drive  12 30 2.0        # 前进 12% + 转向 30%，2 s

所有动作都在 duration 内以 20 Hz 续租约（MCU 租约 300 ms），
结束后一定发 stop。中途 Ctrl+C 也会 stop。
"""

import sys
import time

from arduino.app_utils import Bridge

HZ = 20
PERIOD = 1.0 / HZ


def call(method, *args, timeout=3.0):
    return Bridge.call(method, *args, timeout=timeout)


def status():
    try:
        return call("status")
    except Exception as e:
        return f"<status 取不到: {e}>"


def run_action(thr, steer, dur):
    print(f"目标 throttle={thr}% steer={steer}%  持续 {dur}s")
    t_end = time.monotonic() + float(dur)
    sent = 0
    try:
        while time.monotonic() < t_end:
            call("drive", int(thr), int(steer))
            sent += 1
            time.sleep(PERIOD)
    finally:
        call("stop")
        print(f"已停（下发 {sent} 帧）")
        time.sleep(0.3)
        print("状态:", status())


def run_raw(thr_us, steer_us, gear_us, dur):
    print(f"RAW D3={thr_us}us D5={steer_us}us D6={gear_us}us  持续 {dur}s")
    t_end = time.monotonic() + float(dur)
    sent = 0
    try:
        while time.monotonic() < t_end:
            call("set_pwm", int(thr_us), int(steer_us), int(gear_us))
            sent += 1
            time.sleep(PERIOD)
    finally:
        call("stop")
        print(f"已停（下发 {sent} 帧）")
        time.sleep(0.3)
        print("状态:", status())


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 2

    mode = a[0]

    if mode == "status":
        print(status())
        return 0

    if mode == "stop":
        call("stop")
        print("已停。", status())
        return 0

    try:
        if mode == "steer":
            thr, steer, dur = 0, int(a[1]), float(a[2])
        elif mode == "forward":
            thr, steer, dur = abs(int(a[1])), 0, float(a[2])
        elif mode == "back":
            thr, steer, dur = -abs(int(a[1])), 0, float(a[2])
        elif mode == "drive":
            thr, steer, dur = int(a[1]), int(a[2]), float(a[3])
        elif mode == "raw":
            # raw <thrUs> <steerUs> <gearUs> <dur>   诊断/标定专用
            run_raw(int(a[1]), int(a[2]), int(a[3]), float(a[4]))
            return 0
        else:
            print(f"未知模式: {mode}")
            return 2
    except (IndexError, ValueError):
        print(__doc__)
        return 2

    run_action(thr, steer, dur)
    return 0


if __name__ == "__main__":
    sys.exit(main())
