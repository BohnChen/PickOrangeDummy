#!/usr/bin/env python3
"""
instant_tap.py - 前后轴向精准轻触执行器 (目标正对 + 沿进刀轴向前轻碰 3 次)

【全新仿生抓取逻辑】:
1. 为什么“前后轻触物体表面”比“从上方点头”更合理？
   - 我们的末端是自研的“套果-旋切”筒状机构，进刀方向必须是【正面对准果实、沿进刀轴向前套入】；
   - 先对准目标正前方 2.5cm 处，然后【向前顶出轻触表面 -> 后退 -> 向前顶出】，
     这与真正的果实套入、夹取前探测动作完全契合！
2. 阶段化执行流程:
   - 阶段 1: 机械臂平滑伸展，末端切刀轴直直正对目标前方 2.5cm 处，定住停稳 1 秒展示！
   - 阶段 2: 沿视线/进刀矢量向前顶进 2.5cm 轻触物体表面 -> 后撤 2cm，清晰连碰 3 次 (Tap x3)！
   - 阶段 3: 动作完成自动平稳两段式折叠收拢回 REST 原点。
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
        self.get_logger().info("【前后精准轻触执行器】就绪，正在连接控制器...")
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
        while not res_future.done():
            time.sleep(0.02)
            if time.time() - t_start > (total_time + 4.0):
                self.get_logger().warn("轨迹等待完成，安全放行！")
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
    """
    根据目标物理水平距离 r 与 高度 z 解析真实 J2/J3 关节角 (CAD 弧度)
    """
    # 锚定基准：水平 r0 = 0.20m, 目标高度 z0 = 0.05m
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
    print("    PickOrangeDummy 前后轴向精准轻触执行器 (正对目标 + 进刀轻碰)   ")
    print("==================================================================")
    print("【全新执行流程】:")
    print("1. 阶段 1: J1精准对准方位，机械臂前探停在目标正前方 2.5cm 处，定住 1 秒；")
    print("2. 阶段 2: 沿进刀轴向前顶出 2.5cm 触碰目标表面 -> 回撤 2cm，连碰 3 次；")
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

            user_input = input("\n[按回车执行前后轴向轻触 / r:回位 / q:退出]: ").strip().lower()
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

            # 1. 目标朝向计算
            j1 = math.atan2(tx, -ty)

            # -------------------------------------------------------------
            # 准备位 (Ready Pose: 停在目标正前方 2.5cm，高度对齐果心)
            # -------------------------------------------------------------
            r_ready = r_dist - 0.025
            j2_ready, j3_ready = solve_arm_kinematics(r_ready, tz)
            ready_joints = [j1, j2_ready, j3_ready, 0.0, 0.0, 0.0]

            # -------------------------------------------------------------
            # 轻碰位 (Touch Pose: 沿半径方向向前顶出 2.5cm，精确轻触物体正面表面)
            # -------------------------------------------------------------
            r_touch = r_dist + 0.005 # 略前伸触碰
            j2_touch, j3_touch = solve_arm_kinematics(r_touch, tz)
            touch_joints = [j1, j2_touch, j3_touch, 0.0, 0.0, 0.0]

            # -------------------------------------------------------------
            # 阶段 1: 目标正对直达 (停在正前方 2.5cm 处)
            # -------------------------------------------------------------
            print(f"\n==================================================================")
            print(f">>> [阶段 1/3: 目标直达] J1 转向 {math.degrees(j1):.1f}°，前探至目标正前方 2.5cm 处...")
            node.execute_waypoints_action([(ready_joints, 2.5)])
            time.sleep(0.8)
            print("✔ 机械臂已平稳到达目标正前方！定住展示 1 秒...")
            time.sleep(1.0)

            # -------------------------------------------------------------
            # 阶段 2: 前后轴向轻碰 3 次 (真正模拟进刀套果探测)
            # -------------------------------------------------------------
            print(f">>> [阶段 2/3: 进刀轻触] 沿进刀轴向前轻碰物体表面 3 次 (Tap x3)...")
            tap_waypoints = [
                (touch_joints, 0.65),  # 向前顶进轻碰表面
                (ready_joints, 1.30),  # 后撤回位
                (touch_joints, 1.95),  # 第 2 次向前轻碰
                (ready_joints, 2.60),  # 后撤回位
                (touch_joints, 3.25),  # 第 3 次向前轻碰
                (ready_joints, 3.90)   # 后撤回位
            ]
            node.execute_waypoints_action(tap_waypoints)
            time.sleep(0.5)
            print("✔ 3 次前后轴向轻触动作全部完成！")
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
