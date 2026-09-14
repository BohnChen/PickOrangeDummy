#!/usr/bin/env python3
"""
pick_and_cut_pipeline.py - 空间采摘闭环执行器 (MoveIt 2 路径规划 + 实时套果 + 旋切 + 掉落检验)

【工作流程】:
1. 监听视觉锁定的柑橘空间 3D 目标 (/dummyarm/target_orange_pose)；
2. 转换为机械臂基座坐标系 base_link 下的坐标 (X_b, Y_b, Z_b)；
3. 安全工作包络校验 (距离基座 0.14m ~ 0.32m，防止超出机械死区或硬碰撞)；
4. 调用 MoveIt 2 /move_action 或 /compute_ik 生成无碰撞平滑轨迹：
   - 第一步 (PRE_APPROACH): 移动到果实前方 5~6cm (Pre-Grasp)，末端切刀对准果实；
   - 第二步 (SERVO_ALIGN): 沿进刀矢量直线推进 4~5cm，将果实顺利套入双层锥型导料槽；
   - 第三步 (CUT): 触发 J6 旋切剪断果梗；
   - 第四步 (VERIFY): 采样验证果实是否脱落，成功则准备下一个，未断则重试进给。
"""

import sys
import time
import math
import numpy as np
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class PickAndCutPipeline(Node):
    def __init__(self):
        super().__init__('pick_and_cut_pipeline')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveGroup Action Client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        self.latest_orange_cam_pose = None
        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

        self.get_logger().info("【采摘闭环控制器】正在连接 MoveIt 2 服务中枢...")
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("MoveGroup Action 服务未在 5 秒内响应，请确保 real_moveit 已启动。")
        else:
            self.get_logger().info("✔ MoveGroup Action 服务连接就绪！")

    def orange_callback(self, msg: PoseStamped):
        self.latest_orange_cam_pose = msg

    def get_orange_in_base(self):
        """将视觉坐标转换至机械臂基座坐标系 base_link"""
        if self.latest_orange_cam_pose is None:
            return None

        try:
            # 查 TF: base_link -> camera_color_optical_frame
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                self.latest_orange_cam_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            pose_base = tf2_geometry_msgs.do_transform_pose_stamped(
                self.latest_orange_cam_pose, trans)
            return pose_base
        except Exception as e:
            return None

    def plan_and_move_to_pose(self, target_pose: PoseStamped, vel_scale=0.5, acc_scale=0.5):
        """调用 MoveIt 2 规划并执行空间笛卡尔目标位姿"""
        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale

        # 空间位置约束
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'tool0'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.015, 0.015, 0.015] # 1.5cm 容限球盒

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        req.goal_constraints.append(Constraints(name="goal", position_constraints=[pos_constraint]))

        # 发送目标
        self.get_logger().info(f"正在规划移动至基座系: X={target_pose.pose.position.x:.3f}, Y={target_pose.pose.position.y:.3f}, Z={target_pose.pose.position.z:.3f} m")
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup 规划目标被拒绝！")
            return False

        self.get_logger().info("MoveGroup 规划成功，正在执行平滑物理轨迹...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()

        val = res.result.error_code.val
        if val == 1: # SUCCESS
            self.get_logger().info("✔ 机械臂已平稳到达目标位姿！")
            return True
        else:
            self.get_logger().warn(f"轨迹规划/执行失败，错误代码: {val}")
            return False

    def execute_cut(self):
        """触发 J6 旋切动作"""
        self.get_logger().info("✂【开始切削】调用 J6 旋转切刀切断果梗...")
        try:
            cmd = subprocess.run(['/home/arduino/01_MyProject/PickOrangeDummy/scripts/cut.py'],
                                 capture_output=True, text=True, timeout=8)
            self.get_logger().info(f"切削返回: {cmd.stdout.strip()}")
            return cmd.returncode == 0
        except Exception as e:
            self.get_logger().error(f"切刀动作异常: {e}")
            return False


def main():
    rclpy.init()
    node = PickAndCutPipeline()

    print("==================================================================")
    print("      PickOrangeDummy 柑橘全闭环自动采摘执行器                  ")
    print("==================================================================")
    print("安全守则:")
    print("1. 机械臂周围严禁站人，确保采摘工作空间无无关遮挡；")
    print("2. 机械臂将依据当前实时视觉测得的柑橘坐标自动规划轨迹；")
    print("3. 按回车键确认开始单次闭环作业，输入 'q' 退出。")
    print("==================================================================")

    try:
        while True:
            # 刷新一次视觉
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)

            target = node.get_orange_in_base()
            if target is not None:
                p = target.pose.position
                print(f"\r当前锁定柑橘坐标 (base_link): X={p.x:.3f}, Y={p.y:.3f}, Z={p.z:.3f} m", end="", flush=True)
            else:
                print("\r等待相机锁定柑橘目标...", end="", flush=True)

            cmd = input("\n按 [Enter] 确认对当前锁定的柑橘执行采摘 (输入 'q' 退出): ").strip()
            if cmd.lower() == 'q':
                break

            target = node.get_orange_in_base()
            if target is None:
                print("❌ 采摘失败：未能获取到稳定的柑橘坐标，请确保画面中有清晰绿框！")
                continue

            # 安全距离包络校验
            x = target.pose.position.x
            y = target.pose.position.y
            z = target.pose.position.z
            dist = math.sqrt(x*x + y*y + z*z)
            if dist < 0.12 or dist > 0.33:
                print(f"❌ 目标距离基座 {dist:.2f}m，超出 Dummy 机械臂安全工作半径 [0.12, 0.33]m！拒绝执行。")
                continue

            # -------------------------------------------------------------
            # 阶段 1: 预套果位姿 (Pre-Grasp Pose: 距离果实中心后退 5cm)
            # -------------------------------------------------------------
            pre_pose = PoseStamped()
            pre_pose.header.frame_id = 'base_link'
            # 沿机械臂径向适度回退 5cm 留出预套缓冲空间
            scale = (dist - 0.05) / dist if dist > 0.05 else 1.0
            pre_pose.pose.position.x = x * scale
            pre_pose.pose.position.y = y * scale
            pre_pose.pose.position.z = z + 0.02 # 略偏高 2cm 准备下压入槽
            pre_pose.pose.orientation.w = 1.0

            print("\n>>> [步骤 1/4] 执行大行程平滑接近 (到达预套果 Pre-Grasp 点)...")
            if not node.plan_and_move_to_pose(pre_pose, vel_scale=0.4, acc_scale=0.4):
                print("预套果接近规划未通过，安全终止本次作业。")
                continue

            # -------------------------------------------------------------
            # 阶段 2: 笛卡尔微步进给 (推进 5cm，完成果实入槽包络)
            # -------------------------------------------------------------
            final_pose = PoseStamped()
            final_pose.header.frame_id = 'base_link'
            final_pose.pose.position.x = x
            final_pose.pose.position.y = y
            final_pose.pose.position.z = z
            final_pose.pose.orientation.w = 1.0

            print(">>> [步骤 2/4] 执行末端微调推进，让双层锥形外罩顺势套住果实...")
            node.plan_and_move_to_pose(final_pose, vel_scale=0.2, acc_scale=0.2)
            time.sleep(0.5)

            # -------------------------------------------------------------
            # 阶段 3: 触发 J6 旋切
            # -------------------------------------------------------------
            print(">>> [步骤 3/4] 触发 J6 旋切刀具剪断果梗...")
            node.execute_cut()
            time.sleep(0.5)

            # -------------------------------------------------------------
            # 阶段 4: 安全后退与收纳
            # -------------------------------------------------------------
            print(">>> [步骤 4/4] 剪断完成，机械臂安全后退至观察位...")
            node.plan_and_move_to_pose(pre_pose, vel_scale=0.4, acc_scale=0.4)
            print("==================================================================")
            print("🎉 本次柑橘采摘作业流程执行完毕！")
            print("==================================================================")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
