#!/usr/bin/env python3
"""
direct_chassis_40pin.py - 直接通过 VENTUNO Q 40-Pin GPIO 发送 50Hz PWM 控制小车底盘
当前硬件接线对应：
  - 转向线 (4号): Pin 33 (GPIO 13)
  - 油门线 (3号): Pin 11 (GPIO 17)
  - 倒车线 (5号): Pin 31 (GPIO 6)
  - 地线  (GND): Pin 6  (GND)
"""

import time
import lgpio

PIN_THR = 17  # Pin 11 (3号油门)
PIN_STR = 13  # Pin 33 (4号转向)
PIN_REV = 6   # Pin 31 (5号倒车)

CHIP = 2

def main():
    print("=" * 50)
    print("PickOrangeDummy 移动底盘 40-Pin 实时控制测试")
    print(f"转向通道 (4号): GPIO {PIN_STR} (Pin 33)")
    print(f"油门通道 (3号): GPIO {PIN_THR} (Pin 11)")
    print(f"倒车通道 (5号): GPIO {PIN_REV} (Pin 31)")
    print("=" * 50)

    h = lgpio.gpiochip_open(CHIP)
    lgpio.gpio_claim_output(h, PIN_THR)
    lgpio.gpio_claim_output(h, PIN_STR)
    lgpio.gpio_claim_output(h, PIN_REV)

    # 50Hz 周期 = 20,000us
    # lgpio.tx_servo 支持直接发送微秒级舵机脉宽 (50Hz)
    try:
        print("\n[步骤 1/4] 发送中位停止信号 (1500us)...")
        lgpio.tx_servo(h, PIN_STR, 1500, 50)
        lgpio.tx_servo(h, PIN_THR, 1500, 50)
        lgpio.tx_servo(h, PIN_REV, 1500, 50)
        time.sleep(2)

        print("\n[步骤 2/4] 测试转向舵机：向左打角 (1100us) 持续 2 秒...")
        lgpio.tx_servo(h, PIN_STR, 1100, 50)
        time.sleep(2)

        print("\n[步骤 3/4] 测试转向舵机：向右打角 (1900us) 持续 2 秒...")
        lgpio.tx_servo(h, PIN_STR, 1900, 50)
        time.sleep(2)

        print("\n[步骤 4/4] 转向回中 (1500us)...")
        lgpio.tx_servo(h, PIN_STR, 1500, 50)
        time.sleep(1)

        print("\n[动力测试] 前进油门输出轻微动力 (1580us) 持续 1.5 秒...")
        lgpio.tx_servo(h, PIN_THR, 1580, 50)
        time.sleep(1.5)

        print("\n[停机归零] 刹车停止 (1400us -> 1500us)...")
        lgpio.tx_servo(h, PIN_THR, 1400, 50)
        time.sleep(1)
        lgpio.tx_servo(h, PIN_THR, 1500, 50)

        print("\n测试序列完成！")

    finally:
        # 安全停止所有脉冲输出
        lgpio.tx_pwm(h, PIN_THR, 0, 0)
        lgpio.tx_pwm(h, PIN_STR, 0, 0)
        lgpio.tx_pwm(h, PIN_REV, 0, 0)
        lgpio.gpio_free(h, PIN_THR)
        lgpio.gpio_free(h, PIN_STR)
        lgpio.gpio_free(h, PIN_REV)
        lgpio.gpiochip_close(h)
        print("GPIO 资源已安全释放。")

if __name__ == "__main__":
    main()
