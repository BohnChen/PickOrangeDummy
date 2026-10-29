"""
小车控制接口（Linux 侧）

用法：
    from car import Car
    car = Car()                 # 启动后台 20Hz 下发线程
    car.drive(30, 0)            # 油门 30%，直行
    car.drive(30, 20)           # 油门 30%，右转（符号以实测为准）
    car.drive(-25, 0)           # 倒车 25%
    car.stop()                  # 停
    car.close()                 # 结束

约定：
    throttle: -100..100，正数前进，负数倒车，0 停
    steer:    -100..100，0 居中，正负为左右（方向以实测为准）

安全：
    - MCU 端有 500ms 看门狗：超过 500ms 没收到新指令会自动回中位。
      所以这里必须持续下发（本类已经用后台线程以 20Hz 完成）。
    - 指令变化带斜率限制（MCU 端 1%/帧，约 50%/秒），不会突然窜车。
    - 启动/结束都会自动 stop()。
"""

import threading
import time

from arduino.app_utils import Bridge


class Car:
    def __init__(self, hz: int = 20, timeout: float = 2.0):
        self._period = 1.0 / hz
        self._timeout = timeout
        self._throttle = 0
        self._steer = 0
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ---------- 对外接口 ----------
    def drive(self, throttle: int, steer: int = 0) -> None:
        """设置目标油门/转向，立即返回（不阻塞）。"""
        throttle = max(-100, min(100, int(throttle)))
        steer = max(-100, min(100, int(steer)))
        with self._lock:
            self._throttle = throttle
            self._steer = steer

    def stop(self) -> None:
        self.drive(0, 0)

    def close(self) -> None:
        self.stop()
        time.sleep(self._period * 2)
        self._running = False
        self._thread.join(timeout=1.0)

    # ---------- 后台下发 ----------
    def _loop(self) -> None:
        while self._running:
            with self._lock:
                t, s = self._throttle, self._steer
            try:
                Bridge.call("drive", t, s, timeout=self._timeout)
            except Exception:
                pass
            time.sleep(self._period)


# 直接运行本文件 = 一段慢速演示，用来验证链路
if __name__ == "__main__":
    car = Car()
    try:
        time.sleep(0.5)
        print("前进 25%，4 秒")
        car.drive(25, 0)
        time.sleep(4)

        print("停")
        car.stop()
        time.sleep(1.5)

        print("倒车 25%，3 秒")
        car.drive(-25, 0)
        time.sleep(3)

        print("停")
        car.stop()
        time.sleep(1.5)

        print("前进 25% + 转向 25%，3 秒")
        car.drive(25, 25)
        time.sleep(3)

        print("停")
        car.stop()
    finally:
        car.close()
        print("done")
