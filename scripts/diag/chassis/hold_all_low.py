#!/usr/bin/env python3
"""
hold_all_low.py - 将所有 GPIO 强制拉低并保持，用于万用表测量诊断
运行时长由命令行参数指定（默认 120 秒）
"""

import sys
import time
import lgpio

CHIP = 2
DURATION = 120

h = lgpio.gpiochip_open(CHIP)

claimed = []
for p in range(40):
    try:
        lgpio.gpio_claim_output(h, p, 0)
        claimed.append(p)
    except Exception:
        pass

print(f"已强制拉低 {len(claimed)} 个 GPIO: {claimed}")
print(f"保持 {DURATION} 秒... 请现在用万用表测量！", flush=True)

end = time.time() + DURATION
while time.time() < end:
    time.sleep(5)
    remaining = int(end - time.time())
    print(f"剩余 {remaining}s ...", flush=True)

for p in claimed:
    try:
        lgpio.gpio_free(h, p)
    except Exception:
        pass
lgpio.gpiochip_close(h)
print("已释放全部引脚")
