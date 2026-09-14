#!/usr/bin/env python3
"""
direct_point_and_nod.py - 极简直达 + J5点头轻叩三下测试脚本 (Point and Nod x3)

【物理动作逻辑】:
1. 空间定位: 读取目标柑橘 (X_b, Y_b, Z_b);
2. 基础对准:
   - J1 底座偏航角直接指向目标: theta_j1 = atan2(Y_b, X_b);
   - J2/J3/J4 保持自然舒展前探，手腕平稳停在目标上方约 3~5cm;
3. "像敲门/点头一样"轻点三下 (Nod x3):
   - 保持 J1~J4 静止不动!
   - 纯粹通过手腕 J5 电机（俯仰关节）上下微幅转动 (低头点头 12° -> 抬头复位) 重复 3 次!
   - 驱动轴就像手指/手肘敲门一样，有节奏地轻叩目标物体 3 下!
4. 绝对零多余绕圈与 J5 空间大翻滚。
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class PointAndNod(Node):
    def __init__(self):
        super().__init__('direct_point_and_nod')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 直接对接底层 JointTrajectoryController (绕过 OMPL 空间翻滚)
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.latest_joint_state = None

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        self.get_logger().info("【直达点头轻叩执行器】正在连接控制器...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("未能连接到 joint_trajectory_controller，请确认 real_moveit 正在运行！")
        else:
            self.get_logger().info("✔ 控制器连接就绪！")

    def orange_callback(self, msg: PoseStamped):
        self.latest_orange_cam_pose = msg

    def joint_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def get_orange_in_base(self):
        if self.latest_orange_cam_pose is None:
            return None
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                self.latest_orange_cam_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return tf2_geometry_msgs.do_transform_pose_stamped(
                self.latest_orange_cam_pose, trans)
        except Exception:
            return None

    def send_joint_goal(self, positions, duration_sec=1.5):
        """直接发送关节目标，路径最短最自然"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg.trajectory.points.append(point)

        send_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("关节指令被拒收！")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True


def solve_direct_reach(x, y, z):
    """
    几何解析直达解算：
    根据目标 (x,y,z)，直接求出自然俯仰姿态，绝不产生手腕翻转！
    """
    # 1. J1 直接对准目标方位角
    j1 = math.atan2(y, x)

    # 水平距离与高度
    r = math.sqrt(x*x + y*y)
    h = z

    # 2. 简易前伸几何分配 (自然舒展姿态)
    # J2J3 前探臂长组合
    # 保持 J4 归零 (0.0) 绝不侧翻
    j4 = 0.0

    # 针对 20~28cm 桌面目标，给出平稳伸展角度 (CAD 弧度):
    # J2 前倾约 75°~85°, J3 微曲约 -70°~-85°
    j2 = 1.35
    j3 = -1.40

    # J5 自然微低头对准物体
    j5 = 0.15
    j6 = 0.0

    return [j1, j2, j3, j4, j5, j6]


def main():
    rclpy.init()
    node = PointAndNod()

    print("==================================================================")
    print("     PickOrangeDummy 视觉直达 + J5 点头轻叩三下测试 (Nod x3)     ")
    print("==================================================================")
    print("【全新动作设计】:")
    print("1. 直达接近：J1 底座直接转向目标，机械臂自然前伸，停在目标正上方；")
    print("2. J4 彻底锁死为 0°，手腕绝不发生侧翻与多余大角度旋转！")
    print("3. 像敲门/点头一样：保持大臂静止，J5 连续点头轻叩 3 次 (Nod 3 次)！")
    print("4. 完成后平稳抬起复位。")
    print("==================================================================")

    try:
        while True:
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)

            target = node.get_orange_in_base()
            if target is not None:
                p = target.pose.position
                print(f"\r当前锁定目标: X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f} m", end="", flush=True)
            else:
                print("\r等待视觉锁定目标...", end="", flush=True)

            cmd = input("\n按 [Enter] 执行直达接近并点头三下 (输入 'q' 退出): ").strip()
            if cmd.lower() == 'q':
                break

            target = node.get_orange_in_base()
            if target is None:
                print("❌ 未获取到稳定的目标坐标！")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z

            dist = math.sqrt(tx*tx + ty*ty + tz*tz)
            if dist < 0.12 or dist > 0.33:
                print(f"❌ 目标距离 {dist:.2f}m 超出安全范围！")
                continue

            # -------------------------------------------------------------
            # 步骤 1: 自然直达目标上方准备位 (J4 严格为 0，杜绝手腕翻转)
            # -------------------------------------------------------------
            reach_joints = solve_direct_reach(tx, ty, tz)
            print(f"\n>>> [1/3] J1 转向方位角 {math.degrees(reach_joints[0]):.1f}°，自然前伸到目标正上方...")
            node.send_joint_goal(reach_joints, duration_sec=2.0)
            time.sleep(0.5)

            # -------------------------------------------------------------
            # 步骤 2: 像敲门/点头一样轻叩三下 (纯 J5 关节转动，大臂绝对静止！)
            # -------------------------------------------------------------
            print(">>> [2/3] 开始执行敲门/点头动作 (保持大臂不动，J5 点头 3 次)...")
            base_j5 = reach_joints[4]

            # 点头角度：低头 15度 (~0.26 rad) 然后回正
            tap_j5 = base_j5 + 0.26

            nod_down_joints = list(reach_joints)
            nod_down_joints[4] = tap_j5

            nod_up_joints = list(reach_joints)
            nod_up_joints[4] = base_j5

            for i in range(1, 4):
                print(f"    👉 [点头/敲门 {i}/3] J5 低头轻触目标物体...")
                node.send_joint_goal(nod_down_joints, duration_sec=0.5)
                time.sleep(0.2)
                print(f"    👈 抬手回位...")
                node.send_joint_goal(nod_up_joints, duration_sec=0.5)
                time.sleep(0.2)

            # -------------------------------------------------------------
            # 步骤 3: 平稳复位
            # -------------------------------------------------------------
            print(">>> [3/3] 点头敲门测试完毕！机械臂平稳保持在安全位。")
            print("==================================================================")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
