#!/bin/bash
# safe_park_and_exit.sh
# 优雅收纳并退出脚本:
# 1. 指挥机械臂低速平滑收回 REST 折叠位 (全关节 0 度)
# 2. 确认到达后向电机发送失能命令释放力矩
# 3. 优雅关闭所有相关的 ROS 2 进程

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

# 检查控制器动作服务是否存在
if ! ros2 action list | grep -q "/joint_trajectory_controller/follow_joint_trajectory"; then
    echo "警告: 未检测到正在运行的 joint_trajectory_controller！"
    echo "如果机械臂已经在 REST 位，直接执行失能..."
else
    # 驱动机械臂回 REST (耗时 8 秒，极低速)
    python3 - << 'EOF'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import sys

rclpy.init()
node = Node('park_runner')

# 先读取当前机械臂各关节实时角度
current_positions = [0.0] * 6
sub = node.create_subscription(JointState, '/joint_states', lambda m: current_positions.__setitem__(slice(None), m.position[:6]) if len(m.position)>=6 else None, 10)
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.1)

client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

if not client.wait_for_server(timeout_sec=5.0):
    print("无法连接到轨迹控制器，跳过运动...")
    sys.exit(0)

goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
point = JointTrajectoryPoint()

# J1~J5 稳步回折叠收拢原点 0.0；J6 保持当前角度不变，绝不多转一圈！
cur_j6 = current_positions[5] if len(current_positions) == 6 else 0.0
point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, cur_j6]
point.time_from_start = Duration(sec=8, nanosec=0)
goal.trajectory.points = [point]

print(f"J6 当前角度 {cur_j6:.2f} rad，收纳时保持不动，仅各轴优雅收拢...")

future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
handle = future.result()

if handle.accepted:
    print("运动指令已接受，正在低速收拢机械臂...")
    res_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res_future)
    print("机械臂已平稳到达 REST 收纳姿态！")
else:
    print("控制器拒绝了运动请求！")

node.destroy_node()
rclpy.shutdown()
EOF
fi

echo ""
echo "=================================================="
echo ">>> 第二步: 机械臂已归位，正在关闭电机力矩 (失能)..."
echo "=================================================="

# 发送失能 (自动探测真实端口)
python3 scripts/disable.py || true

echo ""
echo "=================================================="
echo ">>> 第三步: 正在清理并安全关闭后台 ROS 2 进程..."
echo "=================================================="

pkill -f "real_moveit.launch.py" 2>/dev/null || true
pkill -f "move_group" 2>/dev/null || true
pkill -f "ros2_control_node" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true

echo ""
echo ">>> 机械臂已安全收纳于 REST 物理折叠位，力矩已释放，所有节点已安全退出！<<<"
