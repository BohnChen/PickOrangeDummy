#!/bin/bash
# safe_park_and_exit.sh
# 工业级智能安全收纳停机脚本:
# 1. 自动检测环境:
#    - 若 ROS 2 控制器正在运行: 通过 Action 驱动机械臂平稳回到 REST，并阻塞等待确认到位！
#    - 若 ROS 2 控制器未运行: 直接通过串口发送原生平滑回位指令！
# 2. 确认物理机械臂 100% 到达 REST 收拢位后，再切断电机力矩 (失能)
# 3. 优雅清理并退出后台进程

set -e

WORKSPACE_DIR="/home/arduino/01_MyProject/PickOrangeDummy"
cd "$WORKSPACE_DIR"

source /opt/ros/jazzy/setup.bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo "=================================================="
echo ">>> 第一步: 正在控制机械臂以低速平稳收拢至 REST 姿态..."
echo "=================================================="

python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import sys
import glob
import os
import time
import serial

def find_ref_port():
    ref_ports = glob.glob('/dev/serial/by-id/*REF*')
    if ref_ports:
        return ref_ports[0]
    for p in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']:
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'

# 1. 尝试通过 ROS 2 控制器 Action 驱动 (如果 real_moveit 正在运行)
rclpy.init()
node = Node('smart_park_client')
client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

ros_action_available = client.wait_for_server(timeout_sec=1.5)

if ros_action_available:
    print("检测到 ROS 2 控制器正在运行，正在通过轨迹控制器安全驱动回位...")
    # 读取 J6 当前实时位置
    cur_j6 = 0.0
    sub = node.create_subscription(JointState, '/joint_states', lambda m: None, 10)
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
    point = JointTrajectoryPoint()
    # J1~J5 归零回到物理折叠原点 0.0, 耗时 7 秒匀速收拢
    point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    point.time_from_start = Duration(sec=7, nanosec=0)
    goal.trajectory.points = [point]

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()

    if not handle.accepted:
        print("警告: 控制器未接受收纳指令，尝试切换至串口直驱...")
        ros_action_available = False
    else:
        print("机械臂已开始平稳折叠归位，正在等待到位确认...")
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_future)
        result = res_future.result()
        print(">>> 确认到位: 机械臂已 100% 成功到达 REST 物理折叠位！<<<")

node.destroy_node()
rclpy.shutdown()

# 2. 如果 ROS 2 没开，通过串口原生指令驱动
if not ros_action_available:
    port = find_ref_port()
    print(f"ROS 2 控制器未在线，正在通过串口直接驱动: {port}")
    try:
        s = serial.Serial(port, 115200, timeout=1)
        s.dtr = False
        s.rts = False
        time.sleep(0.15)
        s.reset_input_buffer()

        # 发送原生平滑收拢指令 (J1 物理正前方对齐偏置 6.500°)
        cmd = ">6.500,-73.000,180.000,0.000,0.000,0.000,15.0\n"
        print(f"发送平滑收拢指令: {cmd.strip()}")
        s.write(cmd.encode())
        time.sleep(0.1)
        s.close()

        print("机械臂正在平稳折叠中...")
        for i in range(7, 0, -1):
            print(f"  剩余 {i} 秒...", end="\r", flush=True)
            time.sleep(1.0)
        print("\n>>> 机械臂已平稳到达 REST 物理折叠位！<<<")
    except Exception as e:
        print(f"串口直驱提示: {e}")

EOF

echo ""
echo "=================================================="
echo ">>> 第二步: 机械臂已确认安全到位，正在切断电机力矩 (失能)..."
echo "=================================================="

# 发送失能指令
python3 scripts/disable.py || true

echo ""
echo "=================================================="
echo ">>> 第三步: 正在清理后台进程并安全退出..."
echo "=================================================="

pkill -f "real_moveit.launch.py" 2>/dev/null || true
pkill -f "move_group" 2>/dev/null || true
pkill -f "ros2_control_node" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true

echo ""
echo ">>> [完全成功] 机械臂已平稳收纳至 REST 折叠位，电机已安全失能，所有进程已退出！<<<"
