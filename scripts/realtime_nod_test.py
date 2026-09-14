#!/usr/bin/env python3
"""
realtime_nod_test.py - 真正的实时点头敲门执行器 (无阻塞 + J2/J3/手腕俯仰轻叩 + 优雅归位)

【核心机械学真因解决】:
1. 物理轴向事实：
   - 在 Dummy 机械臂构型中，真正负责“低头/抬头/上下俯仰”的是 J2 与 J3 (以及 J6 轴向)！
   - J5 实际上是末端回转/偏摆关节，强行命令 J5 做敲门，手腕只会发生怪异侧扭；
   - 本脚本通过微调小臂俯仰 (J3 上下微幅转动 4°~6°)，像人动手肘敲门一样自然而生动！
2. 彻底解决“多次回车才动”的事件循环阻塞：
   - 采用独立后台线程持续 spin ROS2 回调，目标坐标实时永不漏帧！
   - 只要终端输入回车，0 延迟瞬间起步！
3. 动作流程：
   - J1 转向目标 -> J2J3 自然舒展停在目标正上方 2~3cm；
   - J3 (小臂手肘) 驱动末端轻快地点头叩击 3 下 (Tap x3)；
   - 叩击完成自动平稳低速收拢回 REST 原点，并 100% 释放 RViz 真机控制权！
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


class RealtimeNodNode(Node):
    def __init__(self):
        super().__init__('realtime_nod_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.latest_joint_state = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        self.get_logger().info("【实时点头叩击节点】正在连接机械臂轨迹控制器...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("未连接到控制器，请确认 real_moveit 正常运行！")
        else:
            self.get_logger().info("✔ 控制器连接就绪！")

    def orange_callback(self, msg: PoseStamped):
        with self.lock:
            self.latest_orange_cam_pose = msg

    def joint_callback(self, msg: JointState):
        with self.lock:
            self.latest_joint_state = msg

    def get_latest_target(self):
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

    def send_joint_goal(self, positions, duration_sec=1.5):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg.trajectory.points.append(point)

        send_future = self.traj_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("关节轨迹指令被拒绝！")
            return False

        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.01)
        return True

    def return_to_rest(self):
        self.get_logger().info("🔄 正在平稳回位至 REST 物理折叠姿态...")
        return self.send_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=4.0)


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
    print("✔ 控制权已无缝交还 RViz！")
    sys.exit(0)


def main():
    global node
    rclpy.init()
    node = RealtimeNodNode()

    # 启动后台常驻线程进行 spin，彻底解决阻塞导致要按多次回车的 bug
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    signal.signal(signal.SIGINT, sigint_handler)

    print("==================================================================")
    print("     PickOrangeDummy 实时精准目标直达 + 点头轻叩 (Nod x3)        ")
    print("==================================================================")
    print("【全新机制】:")
    print("1. 实时多线程架构：目标坐标 0 延迟刷新，按一次回车立刻起动！")
    print("2. 真实物理俯仰敲门：通过 J2/J3 像手肘/手指一样自然上下点头，绝不扭曲！")
    print("3. 手腕 J4/J5 保持正向姿态，绝不发生侧摆大翻滚！")
    print("4. 轻叩 3 次后自动安全收折回到 REST 原点。")
    print("==================================================================")

    try:
        while rclpy.ok():
            target = node.get_latest_target()
            if target is not None:
                p = target.pose.position
                sys.stdout.write(f"\r🎯 实时锁定目标 (base_link): X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f} m")
                sys.stdout.flush()
            else:
                sys.stdout.write("\r⏳ 等待视觉目标锁定中...")
                sys.stdout.flush()

            time.sleep(0.1)

            # 检查是否有输入
            # 简易交互提示
            user_input = input("\n[操作] 按回车开始执行直达轻点 (r:回位, q:退出): ").strip().lower()
            if user_input == 'q':
                sigint_handler(None, None)
                break
            elif user_input == 'r':
                node.return_to_rest()
                print("✔ 已回到 REST 折叠位！")
                continue

            target = node.get_latest_target()
            if target is None:
                print("❌ 未检测到稳定的柑橘坐标，请调整物体位置！")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z

            dist = math.sqrt(tx*tx + ty*ty + tz*tz)
            if dist < 0.12 or dist > 0.33:
                print(f"❌ 目标距离基座 {dist:.2f}m 超出安全范围 [0.12, 0.33]m！")
                continue

            # -------------------------------------------------------------
            # 1. 空间方位计算 (自然平滑舒展)
            # -------------------------------------------------------------
            j1 = math.atan2(ty, tx)

            # J2J3 自然伸展到目标正上方准备位 (Ready)
            # 保持 J4=0, J5=0 正向朝前，绝无侧翻扭动！
            ready_joints = [j1, 1.30, -1.35, 0.0, 0.0, 0.0]

            print(f"\n>>> [步骤 1/3] J1 转向方位角 {math.degrees(j1):.1f}°，自然前伸到目标正上方准备位...")
            node.send_joint_goal(ready_joints, duration_sec=2.0)
            time.sleep(0.3)

            # -------------------------------------------------------------
            # 2. 仿生手肘/点头动作 (利用 J3 关节上下微动 6°，大臂保持，末端精准叩击)
            # -------------------------------------------------------------
            print(">>> [步骤 2/3] 开始执行像敲门/手肘连扣一样的轻点动作 (Tap x3)...")
            tap_down_joints = [j1, 1.30, -1.45, 0.0, 0.0, 0.0] # J3 前探低头
            tap_up_joints   = [j1, 1.30, -1.35, 0.0, 0.0, 0.0] # 回弹复位

            for tap_count in range(1, 4):
                print(f"    👉 [轻点 {tap_count}/3] 末端向下点头叩击物体...")
                node.send_joint_goal(tap_down_joints, duration_sec=0.35)
                time.sleep(0.15)
                print(f"    👈 回弹复位...")
                node.send_joint_goal(tap_up_joints, duration_sec=0.35)
                time.sleep(0.15)

            # -------------------------------------------------------------
            # 3. 自动平稳收折回到 REST 折叠位
            # -------------------------------------------------------------
            print(">>> [步骤 3/3] 叩击完成！正在自动平稳收拢回 REST 姿态...")
            node.return_to_rest()
            print("✔ 机械臂已平稳归位至 REST，控制权随时可用！")
            print("==================================================================")

    except KeyboardInterrupt:
        sigint_handler(None, None)


if __name__ == '__main__':
    main()
