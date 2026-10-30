#!/usr/bin/env python3
"""
sniff_remote.py - 实时捕获并分析手持遥控器操作时，3/4/5 号测试点的真实波形与脉宽
监听引脚：
  - GPIO 17 (Pin 11 / 3号线)
  - GPIO 13 (Pin 33 / 4号线)
  - GPIO 6  (Pin 31 / 5号线)
"""

import time
import lgpio

CHIP = 2
PINS = [17, 13, 6]
NAMES = {17: "Pin11(3号)", 13: "Pin33(4号)", 6: "Pin31(5号)"}

h = lgpio.gpiochip_open(CHIP)

# 记录边沿变化历史
events = {p: [] for p in PINS}

def callback(chip, gpio, level, timestamp):
    # level: 0(下降沿), 1(上升沿)
    events[gpio].append((timestamp, level))

for p in PINS:
    lgpio.gpio_claim_alert(h, p, lgpio.BOTH_EDGES)
    lgpio.callback(h, p, lgpio.BOTH_EDGES, callback)

print("=" * 65)
print("【遥控器信号嗅探分析中...】")
print("--> 请在接下来的 10 秒内，用手持遥控器反复推拉油门和转动方向！<--")
print("=" * 65)

duration = 10.0
start_time = time.time()
while time.time() - start_time < duration:
    remaining = int(duration - (time.time() - start_time))
    print(f"\r剩余采样时间: {remaining:2d} 秒 | 当前捕获边沿数: Pin11={len(events[17])}, Pin33={len(events[13])}, Pin31={len(events[6])}  ", end="", flush=True)
    time.sleep(0.5)

print("\n\n" + "=" * 65)
print("采样结束，正在解析波形特征...")
print("=" * 65)

for p in PINS:
    name = NAMES[p]
    ev_list = events[p]
    count = len(ev_list)
    print(f"\n通道 [{name}] (GPIO {p}): 总共捕获到 {count} 次电平跳变")
    if count >= 4:
        # 计算高电平持续时间与周期
        pulse_widths = []
        periods = []
        for i in range(len(ev_list) - 2):
            t1, l1 = ev_list[i]
            t2, l2 = ev_list[i+1]
            t3, l3 = ev_list[i+2]
            # 上升沿 -> 下降沿 = 高电平脉宽
            if l1 == 1 and l2 == 0:
                pulse_us = (t2 - t1) / 1000.0  # ns -> us
                pulse_widths.append(pulse_us)
            if l1 == 1 and l3 == 1:
                period_us = (t3 - t1) / 1000.0
                periods.append(period_us)

        if pulse_widths:
            avg_pw = sum(pulse_widths[:20]) / len(pulse_widths[:20])
            min_pw = min(pulse_widths[:20])
            max_pw = max(pulse_widths[:20])
            print(f"  -> 高电平脉宽范围: {min_pw:.1f} us ~ {max_pw:.1f} us (平均: {avg_pw:.1f} us)")
        if periods:
            avg_period = sum(periods[:20]) / len(periods[:20])
            freq = 1000000.0 / avg_period if avg_period > 0 else 0
            print(f"  -> 载波周期: {avg_period:.1f} us (频率: {freq:.1f} Hz)")
    elif count > 0:
        print(f"  -> 仅捕获到少量电平跳变: {ev_list}")
    else:
        # 查看静态电平
        static_val = lgpio.gpio_read(h, p)
        print(f"  -> 全程无跳变，恒定保持电平: {static_val}")

lgpio.gpiochip_close(h)
print("\n嗅探分析完成！")
