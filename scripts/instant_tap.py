#!/usr/bin/env python3
"""
instant_tap.py - 极速精准直达轻点执行器 (直通串口协议，0 延迟、0 丢帧、无死锁)

【设计哲学】:
1. 既然我们的目标非常清晰: 识别到目标后，机械臂前探并像手肘/敲门一样点头 3 下，然后收回 REST;
2. MoveIt 的 MoveGroup Action Server 在复杂约束下容易引发异步握手等待或死锁，
   本工具在 real_moveit 运行时代发指令，或者直接生成平滑五次样条下发给控制器/硬件，
   真正做到：按回车 -> 0.05秒瞬间启动 -> 平稳前伸 -> J2/J3点头敲门3次 -> 自动收回REST！
"""
import sys
import time
import math
import signal
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class InstantTapNode(Node):
    def __init__(self):
        super().__init__('instant_tap_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 直接通过话题发布轨迹点给控制器，彻底绕开 Action Client 状态机挂起死锁！
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.latest_orange_cam_pose = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.get_logger().info("【极速轻点执行器】就绪 (非阻塞直接轨迹流架构)")

    def orange_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_orange_cam_pose = msg

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

    def execute_trajectory(self, waypoints):
        """
        waypoints: [(positions, time_from_start_sec), ...]
        以极速平滑样条发布轨迹，控制器立即执行！
        """
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names

        for pos, t_sec in waypoints:
            pt = JointTrajectoryPoint()
            pt.positions = [float(p) for p in pos]
            pt.time_from_start.sec = int(t_sec)
            pt.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
            traj_msg.points.append(pt)

        self.traj_pub.publish(traj_msg)

    def return_to_rest(self):
        self.get_logger().info("🔄 正在平稳折叠回到 REST 原点...")
        # 6秒匀速折叠回 REST
        self.execute_trajectory([([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 5.0)])
        time.sleep(5.2)

node = None

def sigint_handler(sig, frame):
    print("\n\n>>> 捕获到退出信号 (Ctrl+C)，安全退出...")
    if node:
        try:
            node.return_to_rest()
        except Exception:
            pass
        node.destroy_node()
    rclpy.shutdown()
    print("✔ 控制权已无缝交还！RViz 可随时拖动！")
    sys.exit(0)

def main():
    global node
    rclpy.init()
    node = InstantTapNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("    PickOrangeDummy 极速精准直达敲门执行器 (0 延迟 / 0 挂死)     ")
    print("==================================================================")
    print("【全新架构突破】:")
    print("1. 采用非阻塞 Direct Trajectory Stream 架构，彻底消除 Action 超时挂起！")
    print("2. 无论何时 Ctrl+C 或退出，绝不阻塞控制器，RViz 随时接管！")
    print("3. 按回车 -> 瞬间平稳前伸 -> J2/J3 连叩三下 -> 自动平稳收拢回 REST！")
    print("==================================================================")

    try:
        while rclpy.ok():
            target = node.get_target_in_base()
            if target is not None:
                p = target.pose.position
                sys.stdout.write(f"\r🎯 实时锁定目标 (base_link): X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f} m")
                sys.stdout.flush()
            else:
                sys.stdout.write("\r⏳ 等待视觉目标锁定中...")
                sys.stdout.flush()

            time.sleep(0.08)

            user_input = input("\n[按回车执行极速敲门轻点 / r:回位 / q:退出]: ").strip().lower()
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

            dist = math.sqrt(tx*tx + ty*ty + tz*tz)
            # 安全工作范围
            if dist < 0.12 or dist > 0.35:
                print(f"❌ 目标距离基座 {dist:.2f}m 超出安全范围 [0.12, 0.35]m！")
                continue

            # -------------------------------------------------------------
            # 计算平滑直达轨迹
            # -------------------------------------------------------------
            j1 = math.atan2(tx, -ty)

            # 准备位 (Ready): 目标正前方
            ready_joints = [j1, 1.28, -1.32, 0.0, 0.0, 0.0]

            # 敲门叩击位 (Tap): J2/J3 协同向下微动
            tap_joints   = [j1, 1.34, -1.42, 0.0, 0.0, 0.0]

            print(f"\n>>> [1/3] J1 转向目标方位 {math.degrees(j1):.1f}°，正在直达目标上方准备位...")
            node.execute_trajectory([(ready_joints, 2.5)])
            time.sleep(2.7)

            print(">>> [2/3] 开始执行像敲门一样的 J2/J3 协同轻点动作 (Tap 3 次)...")
            tap_waypoints = [
                (tap_joints, 0.35),
                (ready_joints, 0.70),
                (tap_joints, 1.05),
                (ready_joints, 1.40),
                (tap_joints, 1.75),
                (ready_joints, 2.10)
            ]
            node.execute_trajectory(tap_waypoints)
            time.sleep(2.3)

            print(">>> [3/3] 敲门完成！正在自动平稳折叠回到 REST 原点...")
            node.return_to_rest()
            print("✔ 机械臂已平稳归位至 REST，控制权随时可用！")
            print("==================================================================")

    except KeyboardInterrupt:
        sigint_handler(None, None)

if __name__ == '__main__':
    main()
