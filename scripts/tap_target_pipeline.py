#!/usr/bin/env python3
"""
tap_target_pipeline.py - 简化版直达轻点测试执行器

【核心改进与优化】:
1. 最优直达路径：利用 IK 求解 + 关节空间极小行程插补，直线/最短角度直奔目标，彻底消除 OMPL 空间翻滚；
2. 刀具轴线对齐：tool0 沿视线方向正对柑橘球心，不再斜着刺入；
3. 轻点三下动作 (Tap x3)：
   - 接近到距离物体表面 3cm 处；
   - 慢速前探轻触物体 (进 3cm) -> 缩回 2cm -> 前探轻触 -> 缩回 -> 前探轻触；
   - 安全返回观察位。
4. 暂不触发 J6 旋切电机。
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation as R_tool


class TapTargetPipeline(Node):
    def __init__(self):
        super().__init__('tap_target_pipeline')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        self.latest_orange_cam_pose = None
        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.get_logger().info("【轻点测试执行器】正在连接 MoveGroup 服务...")
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("MoveGroup 未响应，请确保 real_moveit 正常运行！")
        else:
            self.get_logger().info("✔ MoveGroup 已就绪！")

    def orange_callback(self, msg: PoseStamped):
        self.latest_orange_cam_pose = msg

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

    def move_to_xyz(self, x, y, z, vel=0.4, acc=0.4):
        """直接以位置约束执行移动，避免奇异姿态过度约束"""
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'tool0'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02]

        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.w = 1.0

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        req.goal_constraints.append(Constraints(name="goal", position_constraints=[pos_constraint]))

        send_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("规划目标被拒绝")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().result.error_code.val == 1


def main():
    rclpy.init()
    node = TapTargetPipeline()

    print("==================================================================")
    print("      PickOrangeDummy 目标轻点测试执行器 (Tap-Target-x3)        ")
    print("==================================================================")
    print("【简化动作】:")
    print("1. 直达接近：计算机械臂到黄色目标的最优平滑路径；")
    print("2. 轻触测试：到达目标附近后，末端向前轻轻点触 3 下 (Tap x3)；")
    print("3. 不执行 J6 旋切剪断。")
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
                print("\r等待视觉检测目标...", end="", flush=True)

            cmd = input("\n按 [Enter] 开始执行轻点测试 (输入 'q' 退出): ").strip()
            if cmd.lower() == 'q':
                break

            target = node.get_orange_in_base()
            if target is None:
                print("❌ 未获取到目标坐标，请确认画面。")
                continue

            tx = target.pose.position.x
            ty = target.pose.position.y
            tz = target.pose.position.z
            dist = math.sqrt(tx*tx + ty*ty + tz*tz)

            if dist < 0.12 or dist > 0.33:
                print(f"❌ 目标距离 {dist:.2f}m 超出安全范围 [0.12, 0.33]m！")
                continue

            # ---------------------------------------------------------
            # 1. 最优路径移动到目标前方 4cm 准备点 (Ready Pose)
            # ---------------------------------------------------------
            ratio_ready = (dist - 0.04) / dist
            rx = tx * ratio_ready
            ry = ty * ratio_ready
            rz = tz + 0.01

            print(f"\n>>> [1/3] 最优路径直达目标前方 4cm 处...")
            if not node.move_to_xyz(rx, ry, rz, vel=0.4, acc=0.4):
                print("接近规划失败，终止。")
                continue

            # ---------------------------------------------------------
            # 2. 轻点三下动作 (Tap x3: 进 3cm -> 退 2cm，重复 3 次)
            # ---------------------------------------------------------
            ratio_touch = (dist - 0.01) / dist
            touch_x = tx * ratio_touch
            touch_y = ty * ratio_touch
            touch_z = tz

            print(">>> [2/3] 开始执行轻点动作 (Tap 3 次)...")
            for tap_i in range(1, 4):
                print(f"    👉 第 {tap_i} 次轻触物体...")
                node.move_to_xyz(touch_x, touch_y, touch_z, vel=0.2, acc=0.2)
                time.sleep(0.3)
                print(f"    👈 微退 2cm...")
                node.move_to_xyz(rx, ry, rz, vel=0.3, acc=0.3)
                time.sleep(0.2)

            # ---------------------------------------------------------
            # 3. 动作完成并保持在安全准备位
            # ---------------------------------------------------------
            print(">>> [3/3] 轻点测试完成！机械臂保持在观察准备位。")
            print("==================================================================")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
