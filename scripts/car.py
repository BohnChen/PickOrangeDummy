"""
小车控制接口（Linux 侧）v2 —— 带“指令过期”保护

设计底线：
    没有调用 = 车必须停。
    本类在后台以 20 Hz 持续下发，但只有 **最近 TTL 秒内** 被更新过的目标才会被下发；
    超时后自动改成 0（中位）。这样即使调用方（视觉/脚本）崩了、卡住了、忘了 stop，
    车也会在 TTL 内自己停下。

    MCU 侧还有第二道：租约 300 ms，过期立即回中位。两道独立生效。

用法：
    from car import Car
    car = Car()                 # 后台 20Hz 下发
    car.forward(20)             # 前进 20%（需持续刷新，见下）
    car.keep(2.0)               # 保持当前目标 2 秒（循环内部刷新）
    car.back(15); car.keep(1.5)
    car.turn(20)
    car.stop()                  # 立即停

    连续控制（视觉闭环）时这样写，每个感知周期刷新一次即可：
        car.drive(thr, steer)   # 目标在 TTL=0.25s 内有效

约定：
    throttle: -100..100，正数前进，负数倒车，0 停
    steer:    -100..100，0 居中，**正 = 右、负 = 左**（2026-10-31 实测）

安全：
    - MCU 租约 300 ms：超时立即中位（硬停，不走斜率）
    - 本类 TTL 250 ms：调用方停止刷新即自动下发 0
    - MCU 端斜率 1%/帧（≈50%/秒），换向强制先停 300 ms
"""

import threading
import time

from arduino.app_utils import Bridge


class Car:
    def __init__(self, hz: int = 20, ttl: float = 0.25, timeout: float = 2.0,
                 max_speed: int = 50):
        self._period = 1.0 / hz
        self._ttl = ttl
        self._timeout = timeout
        self._max_speed = max_speed      # 现阶段软限速：慢速先跑通，标定后再放宽
        self._throttle = 0
        self._steer = 0
        self._last_cmd = 0.0
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---------- 对外接口 ----------
    def drive(self, throttle: int, steer: int = 0) -> None:
        """设置目标油门/转向。目标只在 ttl 内有效，之后自动归零。"""
        throttle = max(-self._max_speed, min(self._max_speed, int(throttle)))
        steer = max(-100, min(100, int(steer)))
        with self._lock:
            self._throttle = throttle
            self._steer = steer
            self._last_cmd = time.monotonic()

    def forward(self, speed: int = 30) -> None:
        self.drive(abs(int(speed)), 0)

    def back(self, speed: int = 30) -> None:
        self.drive(-abs(int(speed)), 0)

    def turn(self, steer: int) -> None:
        self.drive(0, steer)

    def stop(self) -> None:
        """立即停（清目标，下一帧就下发 0）。"""
        with self._lock:
            self._throttle = 0
            self._steer = 0
            self._last_cmd = 0.0

    def keep(self, seconds: float) -> None:
        """保持当前目标指定的秒数，然后停。用于“走一段”的临时动作。"""
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            self._touch()
            time.sleep(self._period)
        self.stop()

    def close(self) -> None:
        self.stop()
        time.sleep(self._period * 2)
        self._running = False
        self._thread.join(timeout=1.0)

    # ---------- 内部 ----------
    def _touch(self) -> None:
        """刷新“目标还有效”的时间戳（不改目标值）。"""
        with self._lock:
            self._last_cmd = time.monotonic()

    def _loop(self) -> None:
        while self._running:
            now = time.monotonic()
            with self._lock:
                fresh = (now - self._last_cmd) <= self._ttl
                t, s = (self._throttle, self._steer) if fresh else (0, 0)
            try:
                Bridge.call("drive", t, s, timeout=self._timeout)
            except Exception:
                pass
            time.sleep(self._period)


# 直接运行本文件 = 慢速自检（验证链路 + 保护是否生效）
if __name__ == "__main__":
    car = Car()
    try:
        time.sleep(0.5)
        print("前进 20%，2 秒")
        car.forward(20)
        car.keep(2.0)
        time.sleep(1.0)

        print("倒车 15%，1.5 秒")
        car.back(15)
        car.keep(1.5)
        time.sleep(1.0)

        print("只转向 20%，1 秒")
        car.turn(20)
        car.keep(1.0)

        print("故意不再刷新：应该在 0.25s 内自动停")
        car.drive(20, 0)
        time.sleep(1.5)
    finally:
        car.close()
        print("done")
