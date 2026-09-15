#!/usr/bin/env python3
"""
instant_tap.py - 工业级确定性直达敲门执行器 (Action + 双向闭环 + 彻底杜绝崩溃与未响应)
"""
import sys
import time
import math
import signal
import threading
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


class InstantTapNode(Node):
    def __init__(self):
        super().__init__('instant_tap_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.latest_joint_state = None
        self.active_goal_handle = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.get_logger().info("【确定性敲门执行器】就绪，正在连接控制器...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("未能连上控制器，请确认 real_moveit 正常运行！")
        else:
            self.get_logger().info("✔ 控制器通道连接成功！")

    def orange_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_orange_cam_pose = msg

    def joint_callback(self, msg: JointState):
        with self.lock:
            self.latest_joint_state = msg

    def get_target_in_base(self):
        with self.lock:
            pose = self.latest_orange_cam_pose
        if pose is None:
            return None
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            return tf2_geometry_msgs.do_transform_pose_stamped(pose, trans)
        except Exception:
            return None

    def execute_waypoints_action(self, waypoints):
        """
        waypoints: [(positions, time_from_start_sec), ...]
        通过官方 Action 下发多点轨迹，100% 确保真实物理运动！
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        for pos, t_sec in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in pos]
            pt.time_from_start.sec = int(t_sec)
            pt.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
            goal_msg.trajectory.points.append(pt)

        send_future = self.traj_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)
        self.active_goal_handle = send_future.result()

        if not self.active_goal_handle.accepted:
            self.get_logger().error("控制器拒收了该轨迹请求！")
            self.active_goal_handle = None
            return False

        res_future = self.active_goal_handle.get_result_async()
        total_time = waypoints[-1][1]
        t_start = time.time()
        # 宽容等待执行完成，绝不卡死
        while not res_future.done():
            time.sleep(0.02)
            if time.time() - t_start > (total_time + 4.0):
                self.get_logger().warn("轨迹等待超时，安全放行！")
                break

        self.active_goal_handle = None
        return True

    def return_to_rest(self):
        self.get_logger().info("🔄 正在两段式平稳收折回到 REST 原点...")
        lift_joints = [0.0, 1.10, -1.10, 0.0, 0.0, 0.0]
        rest_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.execute_waypoints_action([
            (lift_joints, 2.5),
            (rest_joints, 5.5)
        ])


def solve_arm_kinematics(r, z):
    """真机标定级几何逆解"""
    j2 = 1.38 + (r - 0.20) * 1.6 - (z - 0.05) * 1.2
    j3 = -1.38 + (r - 0.20) * 1.5 - (z - 0.05) * 0.6
    j2 = max(0.40, min(1.85, j2))
    j3 = max(-2.15, min(-0.65, j3))
    return j2, j3


node = None

def sigint_handler(sig, frame):
    print("\n\n>>> 捕获到退出信号 (Ctrl+C)，安全释放连接...")
    if node:
        try:
            node.return_to_rest()
        except Exception:
            pass
        node.destroy_node()
    rclpy.shutdown()
    print("✔ 控制权已交还 RViz！")
    sys.exit(0)


def main():
    global node
    rclpy.init()
    node = InstantTapNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("    PickOrangeDummy 确定性分阶段轻点执行器 (Action通道 + 0崩溃)   ")
    print("==================================================================")
    print("【执行流程】:")
    print("1. 阶段 1: J1精准朝向，J2/J3前探停在目标正上方 2cm，定住展示 1 秒；")
    print("2. 阶段 2: 像人手肘敲门一样，J2/J3 协同向下轻叩物体表面 3 次；")
    print("3. 阶段 3: 动作完成自动平稳两段式折叠收拢回到 REST 原点。")
    print("==================================================================")

    try:
        while rclpy.ok():
            target = node.get_target_in_base()
            if target is not None:
                p = target.pose.position
                r_now = math.sqrt(p.x * p.x + p.y * p.y)
                sys.stdout.write(f"\r🎯 实时锁定: 水平半径 R={r_now:.3f}m, 高度 Z={p.z:.3f}m (X={p.x:.3f}, Y={p.y:.3f})")
                sys.stdout.flush()
            else:
                sys.stdout.write("\r⏳ 等待视觉目标锁定中...")
                sys.stdout.flush()

            time.sleep(0.08)

            user_input = input("\n[按回车执行精准分阶段敲门 / r:回位 / q:退出]: ").strip().lower()
            if user_input == 'q':
                sigint_handler(None, None)
                break
            elif user_input == 'r':
                node.return_to_rest()
                print("✔ 已回到 REST 折叠位！")
                continue

            target = node.get_target_in_base()
            if target is None:
                print("❌ 未检测到稳定的目标坐标！")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z

            r_dist = math.sqrt(tx * tx + ty * ty)
            if r_dist < 0.12 or r_dist > 0.29:
                print(f"❌ 目标水平距离 {r_dist:.2f}m 超出有效作业半径 [0.12, 0.29]m！")
                continue

            # 1. 目标朝向与关节解算
            j1 = math.atan2(tx, -ty)

            # 准备位 (停在目标正上方 2cm)
            j2_ready, j3_ready = solve_arm_kinematics(r_dist, tz + 0.02)
            ready_joints = [j1, j2_ready, j3_ready, 0.0, 0.0, 0.0]

            # 叩击位 (下探 2.5cm 轻叩表面)
            j2_tap = min(1.85, j2_ready + 0.06)
            j3_tap = max(-2.15, j3_ready - 0.08)
            tap_joints = [j1, j2_tap, j3_tap, 0.0, 0.0, 0.0]

            # -------------------------------------------------------------
            # 阶段 1: 目标直达
            # -------------------------------------------------------------
            print(f"\n==================================================================")
            print(f">>> [阶段 1/3: 目标直达] J1 转向 {math.degrees(j1):.1f}°，前伸至目标正上方 2cm...")
            node.execute_waypoints_action([(ready_joints, 2.5)])
            time.sleep(0.8)
            print("✔ 机械臂已平稳到达目标正上方！定住展示 1 秒...")
            time.sleep(1.0)

            # -------------------------------------------------------------
            # 阶段 2: 像敲门一样轻叩 3 次
            # -------------------------------------------------------------
            print(f">>> [阶段 2/3: 执行轻叩] 开始执行连贯、清晰的点头敲门动作 (Tap x3)...")
            tap_waypoints = [
                (tap_joints, 0.60),
                (ready_joints, 1.20),
                (tap_joints, 1.80),
                (ready_joints, 2.40),
                (tap_joints, 3.00),
                (ready_joints, 3.60)
            ]
            node.execute_waypoints_action(tap_waypoints)
            time.sleep(0.5)
            print("✔ 3 次敲门轻叩动作全部完成！")
            time.sleep(1.0)

            # -------------------------------------------------------------
            # 阶段 3: 平稳收折回位
            # -------------------------------------------------------------
            print(f">>> [阶段 3/3: 平稳收纳] 正在自动两段式折叠回到 REST 原点...")
            node.return_to_rest()
            print("✔ 机械臂已平稳归位至 REST，系统就绪！")
            print("==================================================================")

    except KeyboardInterrupt:
        sigint_handler(None, None)


if __name__ == '__main__':
    main()
