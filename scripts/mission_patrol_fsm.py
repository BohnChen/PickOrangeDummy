#!/usr/bin/env python3
"""
mission_patrol_fsm.py - 移动采摘巡航寻果状态机 (L3 级核心演示)

行为逻辑：
1. CRUISE (巡航寻果):
   - 小车以 20% 安全慢速向前巡航；
   - 侧向相机实时运行 YOLO/HSV 感知流水线；
   - 超时保护：若连续巡航 10 秒仍未发现果实，立即主动刹车停止，进入 TIMEOUT 状态！
2. STOP (发现果实停车):
   - 一旦在甜区视野内检测到目标果实，立刻切断油门停稳 (stop)。
   - 输出发现目标坐标与提示，准备对接采摘能力层！
"""

import sys
import os
import time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_pick

CHASSIS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src/pickorange_chassis"))
if CHASSIS_DIR not in sys.path:
    sys.path.insert(0, CHASSIS_DIR)
from pickorange_chassis.rpc import RpcClient

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--ignore-vision", action="store_true", help="强制忽略视觉，专门测试 10 秒超时巡航安全停车")
    parser.add_argument("--speed", type=int, default=25, help="巡航速度百分比（推荐 25%）")
    parser.add_argument("--disable-sonar-guard", action="store_true", default=True, help="架空/近距离调试时关闭超声波自动刹停")
    args = parser.parse_args()

    print("==================================================")
    print(">>> 启动 L3 整机巡航寻果编排状态机 <<<")
    if args.ignore_vision:
        print(f"模式: 【超时巡航测试】车速={args.speed}%，强制忽略视觉，验证满 10 秒底盘自动刹停！")
    else:
        print(f"规则: 小车以 {args.speed}% 慢速巡航前进，实时搜寻目标；")
        print("      若发现目标立刻停车；若 10 秒未发现目标自动超时停车。")
    print("==================================================")
    
    client = RpcClient('tcp:127.0.0.1:7501')
    try:
        # 架空调试防误触：关闭前向声呐近距拦截
        if args.disable_sonar_guard:
            client.call('sonar_guard', 0, 0, 0)
            
        # 确保起始状态安全居中
        client.call('turn', 0)
        client.call('stop')
        time.sleep(0.5)
        
        start_time = time.time()
        timeout_limit = 10.0
        patrol_speed = args.speed
        
        found_target = False
        print("\n>>> 进入 CRUISE 巡航状态，小车开始平滑连续前进...")
        
        step_count = 0
        while time.time() - start_time < timeout_limit:
            step_count += 1
            # 持续给底盘续租约（前进 20%）
            client.call('forward', patrol_speed)
            
            elapsed = time.time() - start_time
            print(f"  [巡航中] 耗时: {elapsed:4.1f}s / {timeout_limit:.0f}s (车速={patrol_speed}%)...", end="\r", flush=True)
            
            # 每隔约 0.6 秒进行一次视觉探寻
            if not args.ignore_vision and step_count % 12 == 0:
                print(f"\n  [视觉探寻] 正在扫描侧向视野...")
                fruit = arm_pick.detect_fruit()
                if fruit is not None:
                    # 找到了目标！
                    found_target = True
                    print(f"\n🎯 >>> 发现目标果实！<<<")
                    cls_name = fruit.get('class', fruit.get('source', 'target'))
                    print(f"   来源: {cls_name}, 像素: {fruit['pixel']}, 深度: {fruit['depth']*1000:.0f}mm")
                    print(f"   底座物理估计坐标: {[round(x, 3) for x in fruit['p_base']]}")
                    break
            
            time.sleep(0.04) # 25Hz 循环
            
        print("\n>>> 正在切断底盘动力 (STOP)...")
        client.call('stop')
        time.sleep(0.2)
        
        if found_target:
            print("==================================================")
            print(">>> 任务判定: 成功发现目标并准确定位停车！")
            print(">>> 触发下一步: 锁存底盘，准备调用机械臂套果采摘！")
            print("==================================================")
        else:
            print("==================================================")
            print(f">>> 任务判定: 巡航满 10 秒未检测到目标，触发超时安全停车！")
            print(">>> 状态机安全退出，等待下一步人工或语音调度。")
            print("==================================================")
            
    finally:
        client.call('stop')
        client.close()

if __name__ == '__main__':
    main()
