#!/usr/bin/env python3
"""
calibrate_offset.py - 采摘闭环零样本真实物理偏差一键校准工具

操作原理：
1. 视觉系统先对当前视野中的目标进行检测，记录原始视觉识别坐标 P_vision (base_link)。
2. 用户在机械臂失能或手动微调下，将机械臂末端罩口【真正套中小黄鸭/柑橘】。
3. 脚本读取机械臂实测关节角，通过正运动学 (FK) 严格计算此时罩口中心的真实物理坐标 P_physical (base_link)。
4. 真实物理偏差即为:
       OFFSET = P_physical - P_vision
5. 脚本自动将该偏移向量固化写入 config/pick_offset.json，并在 arm_pick.py 中全局生效！
"""

import os
import sys
import json
import time
import math
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_pick
import arm_kin
import calib_eye_to_hand as cal

OFFSET_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "pick_offset.json"
)

def main():
    print("==================================================")
    print(">>> 采摘闭环：真实物理偏差一键校准系统 <<<")
    print("==================================================")
    
    # 1. 采集当前视野中的视觉坐标
    print("\n[第一步] 正在通过相机采集目标的原始视觉定位坐标...")
    # 强制关闭任何历史补偿，获取纯净的原始视觉外参投影点
    fruit = arm_pick.detect_fruit()
    if fruit is None:
        print("!! 错误：当前相机未能检测到目标，请确保目标在视野中且光照充足！")
        return 1
    
    p_vision = np.array(fruit["p_base"], dtype=float)
    print(f"   [视觉检测到目标] 深度={fruit['depth']*1000:.0f}mm, 像素={fruit['pixel']}")
    print(f"   原始视觉 base_link 坐标: X={p_vision[0]:+.4f}, Y={p_vision[1]:+.4f}, Z={p_vision[2]:+.4f}")
    
    # 2. 读取当前机械臂物理姿态
    print("\n[第二步] 正在读取当前机械臂物理关节位置（请确认你已经将末端罩口套住了目标）...")
    ser = cal.agx.RefSerial(cal.agx.PORT_CANDIDATES[0] if os.path.exists(cal.agx.PORT_CANDIDATES[0]) else cal.agx.PORT_CANDIDATES[1])
    try:
        ser.cmd("!START", 0.8)
        js = ser.joints()
        if js is None:
            print("!! 错误：无法从串口读取到机械臂关节角！")
            return 1
        print(f"   机械臂实测关节角 (fw): {[round(x, 2) for x in js]}")
        
        # 3. 计算罩口中心物理坐标
        model_q = arm_kin.model_from_fw(js)
        T_full = arm_kin.fk_full(model_q)
        tcp_pos = T_full[:3, 3]
        tool_z_axis = T_full[:3, 2]
        # 罩口中心在 TCP(法兰) 沿工具 Z 轴前方 125mm
        mouth_center = tcp_pos + arm_pick.CONE_LEN_M * tool_z_axis
        print(f"   当前机械臂 TCP 位置:   X={tcp_pos[0]:+.4f}, Y={tcp_pos[1]:+.4f}, Z={tcp_pos[2]:+.4f}")
        print(f"   当前机械臂罩口中心位置: X={mouth_center[0]:+.4f}, Y={mouth_center[1]:+.4f}, Z={mouth_center[2]:+.4f}")
        
        # 4. 计算偏差
        offset = mouth_center - p_vision
        print("\n==================================================")
        print(">>> 真实物理偏差解算结果 <<<")
        print(f"   ΔX (前后补偿) : {offset[0]*1000:+.1f} mm")
        print(f"   ΔY (侧向补偿) : {offset[1]*1000:+.1f} mm")
        print(f"   ΔZ (高度补偿) : {offset[2]*1000:+.1f} mm")
        print(f"   空间综合直线偏差: {np.linalg.norm(offset)*1000:.1f} mm")
        print("==================================================")
        
        # 5. 落盘固化
        cfg = {
            "offset_x_m": round(float(offset[0]), 4),
            "offset_y_m": round(float(offset[1]), 4),
            "offset_z_m": round(float(offset[2]), 4),
            "total_err_mm": round(float(np.linalg.norm(offset)*1000.0), 1),
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "note": "由真实套入物理实验反算得到，包含相机标定外参漂移与末端悬臂重力形变"
        }
        os.makedirs(os.path.dirname(OFFSET_CONFIG_PATH), exist_ok=True)
        with open(OFFSET_CONFIG_PATH, "w") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        print(f"\n[完成] 补偿参数已固化至: {OFFSET_CONFIG_PATH}")
        print("后续所有采摘与进刀脚本将自动全局引用此补偿！")
        return 0
    finally:
        ser.close()

if __name__ == "__main__":
    sys.exit(main())
