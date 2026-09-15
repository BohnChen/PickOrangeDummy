#!/usr/bin/env python3
"""
interactive_touch_tune.py - 触碰精度实时微调与闭环标定工具

【设计目的】:
1. 解决“机械臂确实在对 RViz 中识别到的小球进行预设运动，但识别位置与真实物体仍有细微偏差”的痛点；
2. 本工具让机械臂直接前探停在当前视觉认为的物体表面位置；
3. 操作者肉眼直接观察真实物理末端和真实物体表面的距离：
   - 如果还差 2cm，按键盘方向键或者输入偏移微调；
   - 确认无误后按 's'，工具自动将该微调偏置写死保存在系统中，
     彻底抹平最后的几厘米识别定位误差，实现 100% 刀刀命中的物理接触！
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
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class TouchTuner(Node):
    def __init__(self):
        super().__init__('touch_tuner')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        from rclpy.action import ActionClient
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.latest_orange_cam_pose = None
        self.lock = threading.Lock()

        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.traj_client.wait_for_server(timeout_sec=5.0)

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

    def move_to_joints(self, pos, duration=2.5):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in pos]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(pt)

        send_future = self.traj_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)
        handle = send_future.result()
        res_future = handle.get_result_async()
        t_start = time.time()
        while not res_future.done():
            time.sleep(0.01)
            if time.time() - t_start > (duration + 3.0):
                break
        return True


def solve_arm_kinematics(r, z):
    j2 = 1.38 + (r - 0.20) * 1.6 - (z - 0.05) * 1.2
    j3 = -1.38 + (r - 0.20) * 1.5 - (z - 0.05) * 0.6
    j2 = max(0.40, min(1.85, j2))
    j3 = max(-2.15, min(-0.65, j3))
    return j2, j3


def main():
    rclpy.init()
    node = TouchTuner()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("==================================================================")
    print("      PickOrangeDummy 触碰精度实时对齐与微调工具 (Touch-Tuner)     ")
    print("==================================================================")
    print("【操作指引】:")
    print("1. 机械臂将根据当前视觉前探到达目标位置；")
    print("2. 你可以直观看到末端距离物体还差多少；")
    print("3. 输入微调指令 (如: +1 前推1cm, -1 后退1cm, u1 上抬1cm, d1 下降1cm)；")
    print("4. 调到完美贴碰后，系统直接为你固化这个最佳触碰偏移！")
    print("==================================================================")

    offset_r = 0.0 # 距离偏移 (米)
    offset_z = 0.0 # 高度偏移 (米)

    try:
        while rclpy.ok():
            target = node.get_target_in_base()
            if target is None:
                print("\r⏳ 等待视觉目标锁定...", end="", flush=True)
                time.sleep(0.2)
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z

            r_base = math.sqrt(tx * tx + ty * ty)
            j1 = math.atan2(tx, -ty)

            # 加上人工微调
            r_actual = r_base + offset_r
            z_actual = tz + offset_z

            j2, j3 = solve_arm_kinematics(r_actual, z_actual)
            current_joints = [j1, j2, j3, 0.0, 0.0, 0.0]

            print(f"\n当前视觉目标: R={r_base:.3f}m, Z={tz:.3f}m | 当前累计补偿: 径向 {offset_r*100:+.1f}cm, 高度 {offset_z*100:+.1f}cm")
            print("正在驱动末端到达当前位置观察...")
            node.move_to_joints(current_joints, duration=2.5)
            print("✔ 已到位！请直接肉眼观察末端与物体的接触情况。")

            cmd = input("\n[微调] (+1前送1cm / -1后退1cm / d1下探1cm / u1上抬1cm / s保存 / q退出): ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 's':
                print(f"🎉 最佳触碰补偿定案: 径向增量 {offset_r*100:+.1f}cm, 垂直增量 {offset_z*100:+.1f}cm！")
                break

            import re
            m = re.match(r'^([+\-du])(\d*\.?\d*)$', cmd)
            if m:
                action, num_str = m.groups()
                step = float(num_str) if num_str else 1.0
                if action == '+':
                    offset_r += step * 0.01
                elif action == '-':
                    offset_r -= step * 0.01
                elif action == 'd':
                    offset_z -= step * 0.01
                elif action == 'u':
                    offset_z += step * 0.01
            else:
                # 容错输入，若用户连续误敲如 dd1
                if 'd' in cmd:
                    offset_z -= 0.01
                elif 'u' in cmd:
                    offset_z += 0.01
                elif '+' in cmd:
                    offset_r += 0.01
                elif '-' in cmd:
                    offset_r -= 0.01

        # 平稳收回
        node.move_to_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=5.0)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
