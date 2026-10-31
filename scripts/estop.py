#!/usr/bin/env python3
"""底盘急停 / 解除 / 状态

用法：
    python3 estop.py            # 锁存急停（立刻硬停；之后所有运动指令都被拒绝）
    python3 estop.py --arm      # 解除锁存，恢复可动
    python3 estop.py --stop     # 只停，不锁存
    python3 estop.py --status   # 看当前状态

为什么是"锁存"：
    普通 stop() 只是把目标清零。控制程序（视觉循环）下一帧还会继续调 drive()，
    车就又走了。estop() 会置一个锁存位，MCU 直接拒绝 drive/forward/back/turn，
    直到显式 arm()。所以这个命令能从另一个进程随时把车按住。

运行环境（二选一）：
    1. App 容器里（自带 arduino.app_utils）
    2. 宿主机 + 已加入组：sudo usermod -aG arduino-router $USER  （之后重新登录）
       并且需要能 import arduino.app_utils
"""

import sys
import time

def _bridge():
    from arduino.app_utils import Bridge
    return Bridge

def _call(method, *args, retries=3, timeout=2.0):
    Bridge = _bridge()
    last = None
    for _ in range(retries):
        try:
            return Bridge.call(method, *args, timeout=timeout)
        except Exception as e:      # 旧固件没有该方法 / 链路抖动
            last = e
            time.sleep(0.05)
    raise last

def main() -> int:
    arg = sys.argv[1] if len(sys.argv) > 1 else ""

    if arg == "--status":
        print(_call("status"))
        return 0

    if arg == "--arm":
        _call("arm")
        print("已解除急停锁存，可以运动")
        return 0

    if arg == "--stop":
        _call("stop")
        print("已停（未锁存）")
        return 0

    # 默认：锁存急停。发三次，防止单包丢失。
    try:
        for _ in range(3):
            _call("estop")
            time.sleep(0.03)
        print("ESTOP 已锁存：车已硬停；运动指令全部被拒绝")
        print(_call("status"))
    except Exception as e:
        # 旧固件没有 estop：退化为连续 stop
        print("estop 方法不可用（固件太旧？），退化为连续 stop：", e)
        for _ in range(20):
            try:
                _call("stop", retries=1)
            except Exception:
                pass
            time.sleep(0.05)
        print("已连发 stop 1 秒")
    return 0

if __name__ == "__main__":
    sys.exit(main())
