#!/usr/bin/env python3
"""
electrical_diag.py - 电气特性诊断
判断信号线是被强上拉到 5V（推挽）还是弱上拉（可被拉低）
"""

import time
import lgpio

CHIP = 2
PINS = {17: "Pin11(3号油门)", 13: "Pin33(4号转向)", 6: "Pin31(5号倒车)"}
h = lgpio.gpiochip_open(CHIP)

print("=" * 70)
print("【信号线电气特性诊断】")
print("=" * 70)

for pin, name in PINS.items():
    print(f"\n--- {name} (GPIO {pin}) ---")

    # 1. 纯输入（无偏置）读取
    try:
        lgpio.gpio_claim_input(h, pin)
        v_free = lgpio.gpio_read(h, pin)
        lgpio.gpio_free(h, pin)
        print(f"  1) 浮空输入读数       : {v_free}")
    except Exception as e:
        print(f"  1) 读数失败: {e}")
        continue

    # 2. 内部下拉（如果线是弱上拉，拉低后会读 0；强驱动则仍读 1）
    try:
        lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_DOWN)
        time.sleep(0.05)
        v_down = lgpio.gpio_read(h, pin)
        lgpio.gpio_free(h, pin)
        print(f"  2) 内部下拉后读数     : {v_down}  <-- 若仍为1，说明线被强驱动/5V")
    except Exception as e:
        print(f"  2) 下拉失败: {e}")

# 3. 开漏驱动能力测试：输出低电平时能否把线拉下来
print("\n" + "=" * 70)
print("【开漏拉低能力测试】")
print("=" * 70)

for pin, name in PINS.items():
    try:
        lgpio.gpio_claim_output(h, pin, 0)  # 输出低
        time.sleep(0.05)
        v_low = lgpio.gpio_read(h, pin)
        lgpio.gpio_write(h, pin, 1)         # 输出高
        time.sleep(0.05)
        v_high = lgpio.gpio_read(h, pin)
        lgpio.gpio_free(h, pin)
        verdict = "可拉低(我们能控制)" if v_low == 0 else "拉不低(被外部强驱动!)"
        print(f"  {name}: 输出低读到={v_low}, 输出高读到={v_high}  --> {verdict}")
    except Exception as e:
        print(f"  {name}: 测试失败 {e}")

lgpio.gpiochip_close(h)
print("\n诊断完成。")
