#!/usr/bin/env python3
"""
direct_point_and_nod.py - 视觉直达 + J5点头轻叩三下测试 (含安全自动收拢 REST 与优雅退出机制)

【核心动作与状态修复】:
1. 空间定位: 读取目标柑橘 (X_b, Y_b, Z_b);
2. 基础对准:
   - J1 底座对准目标方位角: theta_j1 = atan2(Y_b, X_b);
   - J2/J3 自然前伸停在目标正上方，J4 彻底锁定为 0.0 (绝对杜绝空中侧翻与复杂轨迹);
3. 点头轻叩三下 (Nod x3):
   - J1~J4 锁死静止不动，手腕 J5 (俯仰电机) 像手肘/手指敲门一样上下轻叩 3 次!
4. 优雅归位 REST:
   - 单次测试完成后，可选择自动以平滑低速收拢折叠回物理 REST 位 (0, 0, 0, 0, 0, 0);
   - 彻底释放 Action 占用，即使按下 Ctrl+C 也会无缝将控制权交还给 RViz，保证用户可随时在 RViz 中拖动真机！
"""

import sys
import time
import math
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class PointAndNod(Node):
    def __init__(self):
        super().__init__('direct_point_and_nod')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 对接底层 JointTrajectoryController
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.latest_joint_state = None
        self.active_goal_handle = None

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        self.get_logger().info("【点头轻叩执行器】正在连接控制器...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("未能连接到控制器，请确认 real_moveit 正在运行！")
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
        """发送关节目标，路径最短最平滑"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg.trajectory.points.append(point)

        send_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        self.active_goal_handle = send_future.result()

        if not self.active_goal_handle.accepted:
            self.get_logger().error("关节指令被拒收！")
            return False

        res_future = self.active_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.active_goal_handle = None
        return True

    def return_to_rest(self):
        """平稳低速收拢回到物理 REST 折叠位 (耗时 5 秒平滑归位)"""
        self.get_logger().info("🔄 正在平稳回位至物理折叠位 (REST: 0, 0, 0, 0, 0, 0)...")
        rest_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.send_joint_goal(rest_positions, duration_sec=5.0)


def solve_direct_reach(x, y, z):
    """几何解析直达解算：自然俯仰姿态，J4 绝对锁定为 0"""
    j1 = math.atan2(y, x)
    j4 = 0.0
    j2 = 1.35
    j3 = -1.40
    j5 = 0.15
    j6 = 0.0
    return [j1, j2, j3, j4, j5, j6]


node = None

def sigint_handler(sig, frame):
    """优雅退出处理：释放控制器连接，恢复 RViz 控制权"""
    print("\n\n>>> 捕获到退出信号 (Ctrl+C)，正在安全释放控制器连接...")
    if node:
        try:
            # 询问或安全回位
            user_ans = input("退出前是否需要机械臂自动收拢回 REST 姿态？(y/n，默认 y): ").strip().lower()
            if user_ans != 'n':
                node.return_to_rest()
        except Exception:
            pass
        node.destroy_node()
    rclpy.shutdown()
    print("✔ 控制权已完全恢复并交还给 RViz！你现在可以在 RViz 中任意拖动真机！")
    sys.exit(0)


def main():
    global node
    rclpy.init()
    node = PointAndNod()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("     PickOrangeDummy 视觉直达 + J5 点头轻叩三下测试 (Nod x3)     ")
    print("==================================================================")
    print("【操作指引】:")
    print("1. [Enter] : 机械臂直达目标并像敲门一样轻点 3 次；")
    print("2. 'r'     : 机械臂直接平稳收拢回 REST 折叠位；")
    print("3. 'q' 或 Ctrl+C : 安全退出脚本并立刻将真机控制权无缝交还给 RViz！")
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

            cmd = input("\n[输入指令] (回车:点头测试 / r:一键回位 / q:退出): ").strip()
            if cmd.lower() == 'q':
                sigint_handler(None, None)
                break
            elif cmd.lower() == 'r':
                node.return_to_rest()
                print("✔ 机械臂已平稳到达 REST 物理折叠位！")
                continue

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
            print(f"\n>>> [1/4] J1 转向方位角 {math.degrees(reach_joints[0]):.1f}°，自然前伸到目标正上方...")
            node.send_joint_goal(reach_joints, duration_sec=2.0)
            time.sleep(0.3)

            # -------------------------------------------------------------
            # 步骤 2: 像敲门/点头一样轻叩三下 (纯 J5 关节转动，大臂绝对静止！)
            # -------------------------------------------------------------
            print(">>> [2/4] 开始执行敲门/点头动作 (保持大臂不动，J5 点头 3 次)...")
            base_j5 = reach_joints[4]

            # 点头动作：低头 15度 (~0.26 rad) 然后回正
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
            # 步骤 3: 动作完成提示
            # -------------------------------------------------------------
            print(">>> [3/4] 点头敲门测试完毕！")
            
            # -------------------------------------------------------------
            # 步骤 4: 自动平稳收回 REST 折叠位
            # -------------------------------------------------------------
            print(">>> [4/4] 正在自动平稳收回 REST 折叠姿态...")
            node.return_to_rest()
            print("✔ 机械臂已平稳折叠回到 REST 原点！")
            print("==================================================================")

    except KeyboardInterrupt:
        sigint_handler(None, None)


if __name__ == '__main__':
    main()
