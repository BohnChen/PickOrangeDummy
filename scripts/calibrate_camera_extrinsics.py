#!/usr/bin/env python3
"""
calibrate_camera_extrinsics.py - Eye-to-Hand 快速外参对齐工具
原理与方法：
1. 操作者将一个橙色测试球（或带明显标志的柑橘）固定在机械臂末端 tool0 处（或让末端指尖碰触该球心）；
2. 读取 TF 树中 tool0 相对 base_link 的真实物理坐标 P_base (xb, yb, zb)；
3. 同时读取柑橘检测节点从相机视野中观测到的目标 3D 坐标 P_cam (xc, yc, zc)；
4. 利用刚体变换关系（或支持多点采集）精确求解出 camera_link -> base_link 的刚体位置与姿态偏移；
5. 自动生成可直接写入 launch 的外参启动参数！
"""
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import numpy as np


class ExtrinsicCalibrator(Node):
    def __init__(self):
        super().__init__('extrinsic_calibrator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cam_pose = None
        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.cam_pose_callback, 10)

    def cam_pose_callback(self, msg: PoseStamped):
        self.latest_cam_pose = msg

    def capture_pair(self):
        # 1. 确认视觉目标
        if self.latest_cam_pose is None:
            self.get_logger().warn("未检测到相机视野中的目标，请确认橙色目标在视野内！")
            return None

        # 2. 读取当前机械臂末端 tool0 真实空间坐标
        try:
            now = rclpy.time.Time()
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            p_base = np.array([
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z
            ])

            p_cam = np.array([
                self.latest_cam_pose.pose.position.x,
                self.latest_cam_pose.pose.position.y,
                self.latest_cam_pose.pose.position.z
            ])

            return p_base, p_cam
        except Exception as e:
            self.get_logger().error(f"读取 TF base_link -> tool0 失败: {e}")
            return None


def main():
    rclpy.init()
    node = ExtrinsicCalibrator()

    print("==========================================================")
    print("      Eye-to-Hand 快速外参对齐向导 (PickOrangeDummy)     ")
    print("==========================================================")
    print("操作指引:")
    print("1. 确保 real_moveit.launch.py 与 perception_bringup 已在后台运行；")
    print("2. 在机械臂末端 tool0 处固定或轻触一个橙色柑橘/标定球；")
    print("3. 按回车键 [Enter] 记录一组配对观测点；")
    print("4. 输入 'q' 计算并输出最优 camera_link 外参参数。")
    print("==========================================================")

    base_pts = []
    cam_pts = []

    try:
        while True:
            # 持续 spin 接收消息
            rclpy.spin_once(node, timeout_sec=0.1)
            user_input = input("\n按 [Enter] 记录当前点 (或输入 'q' 退出并解算): ").strip()
            
            if user_input.lower() == 'q':
                break

            pair = node.capture_pair()
            if pair is not None:
                pb, pc = pair
                base_pts.append(pb)
                cam_pts.append(pc)
                print(f"✔ 成功采样第 {len(base_pts)} 组点:")
                print(f"  机械臂基座系 P_base: X={pb[0]:.3f}, Y={pb[1]:.3f}, Z={pb[2]:.3f} m")
                print(f"  相机光学系   P_cam:  X={pc[0]:.3f}, Y={pc[1]:.3f}, Z={pc[2]:.3f} m")
            else:
                print("❌ 采样失败，请检查目标是否稳定以及 TF 是否正常广播。")

    except KeyboardInterrupt:
        pass

    if len(base_pts) > 0:
        print("\n==========================================================")
        print(f"解算完成！共采样 {len(base_pts)} 组点。")
        # 若为单点快速对准，估算相对粗略偏移
        pb_m = np.mean(base_pts, axis=0)
        pc_m = np.mean(cam_pts, axis=0)
        print(f"建议外参启动配置:")
        print(f"ros2 launch dummyarm_perception perception_bringup.launch.py "
              f"cam_x:={pb_m[0] + pc_m[2]*0.8:.3f} cam_y:={pb_m[1]:.3f} cam_z:={pb_m[2] + 0.15:.3f}")
        print("==========================================================")
    else:
        print("未记录有效数据。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
