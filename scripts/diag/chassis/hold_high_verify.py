#!/usr/bin/env python3
"""
hold_high_verify.py - 三个目标引脚同时输出稳定高电平，供万用表验证
  Pin 12 = GPIO_116
  Pin 33 = GPIO_81
  Pin 13 = GPIO_109
"""

import time
import lgpio

CHIP = 2
PINS = {116: "Pin12(油门)", 81: "Pin33(转向)", 109: "Pin13(倒车)"}
DUR = 90

h = lgpio.gpiochip_open(CHIP)
for p in PINS:
    lgpio.gpio_claim_output(h, p, 1)   # 输出高

print("=" * 60)
print("三个引脚已输出 3.3V 高电平，请现在测量！")
for p, n in PINS.items():
    print(f"  GPIO_{p}  ->  {n}")
print("=" * 60, flush=True)

t0 = time.time()
while time.time() - t0 < DUR:
    print(f"\r剩余 {int(DUR - (time.time() - t0)):2d}s  (黑笔Pin6, 红笔点 Pin12/33/13)", end="", flush=True)
    time.sleep(1)

for p in PINS:
    lgpio.gpio_write(h, p, 0)
    lgpio.gpio_free(h, p)
lgpio.gpiochip_close(h)
print("\n已恢复低电平。")
