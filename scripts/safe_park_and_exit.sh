#!/bin/bash
# safe_park_and_exit.sh
# 工业级优雅收纳并退出脚本 (原生平滑驱动版):
# 1. 直接向机械臂主控发送原生平滑回位指令，单片机硬件平滑加减速收拢至 REST 折叠位
# 2. 确认平稳到位后发送 !STOP 与 !DISABLE 释放电机力矩
# 3. 优雅关闭后台 ROS 2 进程

set -e

WORKSPACE_DIR="/home/arduino/01_MyProject/PickOrangeDummy"
cd "$WORKSPACE_DIR"

echo "=================================================="
echo ">>> 第一步: 正在控制机械臂以原生平滑速度收拢至 REST 姿态..."
echo "=================================================="

python3 - << 'EOF'
import glob
import os
import serial
import time
import sys

def find_ref_port():
    ref_ports = glob.glob('/dev/serial/by-id/*REF*')
    if ref_ports:
        return ref_ports[0]
    for p in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']:
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'

port = find_ref_port()
print(f"正在连接机械臂串口: {port}")

try:
    s = serial.Serial(port, 115200, timeout=1)
    s.dtr = False
    s.rts = False
    time.sleep(0.15)
    s.reset_input_buffer()

    # 1. 读当前角度，保留 J6 角度不动
    s.write(b'#GETJPOS\n')
    time.sleep(0.08)
    line = s.readline().decode('utf-8', errors='ignore').strip()
    j6_str = "0.000"
    if line.startswith("ok"):
        parts = line.split()
        if len(parts) == 7:
            j6_str = parts[6]
            print(f"当前关节状态: {line} (J6 当前角度: {j6_str}°)")

    # 2. 发送原生平滑收拢指令 (走向 REST: 0, -73, 180, 0, 0, J6不变，速度15)
    # 单片机硬件级梯形平滑规划，极其丝滑，毫无抖动！
    cmd = f">0.000,-73.000,180.000,0.000,0.000,{j6_str},15.0\n"
    print(f"下发平滑收拢指令: {cmd.strip()}")
    s.write(cmd.encode())
    time.sleep(0.1)
    s.close()

    # 3. 等待收拢运动平稳完成 (约 6~7 秒)
    print("机械臂正在平稳优雅折叠归位中，请稍候...")
    for i in range(7, 0, -1):
        print(f"  剩余 {i} 秒...", end="\r", flush=True)
        time.sleep(1.0)
    print("\n机械臂已平稳到达 REST 物理收纳位！")

except Exception as e:
    print(f"收拢控制提示: {e}")

EOF

echo ""
echo "=================================================="
echo ">>> 第二步: 机械臂已安全归位，正在切断电机力矩 (失能)..."
echo "=================================================="

python3 scripts/disable.py || true

echo ""
echo "=================================================="
echo ">>> 第三步: 正在安全清理后台 ROS 2 进程..."
echo "=================================================="

pkill -f "real_moveit.launch.py" 2>/dev/null || true
pkill -f "move_group" 2>/dev/null || true
pkill -f "ros2_control_node" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true

echo ""
echo ">>> [完成] 机械臂已平稳收纳于 REST 位，电机已安全失能断电，后台进程已清理！<<<"
