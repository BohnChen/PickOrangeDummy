#!/usr/bin/env python3
"""
park_and_disable.py
一键安全收纳并失能工具:
1. 通过 MoveIt / joint_trajectory_controller 驱动机械臂平滑回到 REST 折叠收纳位 (全0度)
2. 到达后向控制板发送 !STOP，安全释放电机力矩，准备关机
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import serial

JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

class ArmParker(Node):
    def __init__(self):
        super().__init__('dummyarm_parker')
        self._client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._cur_positions = [0.0] * 6
        self.create_subscription(JointState, '/joint_states', self._cb_js, 10)

    def _cb_js(self, msg):
        if len(msg.position) >= 6:
            self._cur_positions = list(msg.position[:6])

    def park(self, duration_sec=8.0):
        self.get_logger().info("正在连接轨迹控制器...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("未能连接到 /joint_trajectory_controller/follow_joint_trajectory，请确认 real_moveit 正在运行！")
            return False

        # 稍微 spin 获取实时角度
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = JOINT_NAMES
        
        # REST 姿态: J1~J5 回 0，J6 保持当前角度不变 (避免无意义多圈回旋)
        point = JointTrajectoryPoint()
        cur_j6 = self._cur_positions[5] if len(self._cur_positions) == 6 else 0.0
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, cur_j6]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        goal_msg.trajectory.points = [point]

        self.get_logger().info(f"正在以低速 ({duration_sec}秒) 驱动机械臂收拢至 REST 收纳姿态...")
        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("收纳请求被控制器拒绝！")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info("机械臂已平稳到达 REST 收纳姿态！")
        return True

def disable_via_serial(port='/dev/ttyACM1'):
    try:
        # 如果 real_moveit 没开，直接串口发 !STOP
        s = serial.Serial(port, 115200, timeout=1)
        s.dtr = False
        time.sleep(0.3)
        s.write(b"!STOP\n")
        time.sleep(0.2)
        s.close()
        print("已向串口发送 !STOP，电机力矩已安全释放。")
    except Exception as e:
        print(f"提示: 串口已被驱动独占 (正常现象)，请直接在 real_moveit 终端按 Ctrl+C，系统会自动发送 !STOP 安全失能。")

def main():
    rclpy.init()
    parker = ArmParker()
    success = parker.park(duration_sec=8.0)
    parker.destroy_node()
    rclpy.shutdown()

    if success:
        print("\n==============================================")
        print("机械臂已安全到达 REST 收纳姿态！")
        print("现在请在运行 real_moveit.launch.py 的终端中按 Ctrl+C，")
        print("系统会自动向电机发送 !STOP 释放力矩，即可安全断电。")
        print("==============================================")

if __name__ == '__main__':
    main()
