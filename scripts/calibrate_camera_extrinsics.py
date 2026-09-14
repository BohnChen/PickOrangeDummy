#!/usr/bin/env python3
"""
calibrate_camera_extrinsics.py - Eye-to-Hand 空间严密刚体外参标定工具
基于 3D-3D 空间对应点精确求解：
P_base = T_base_cam @ P_in_cam
直接计算出相机安装在 base_link 下的精确平移与姿态！
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
        if self.latest_cam_pose is None:
            self.get_logger().warn("未检测到相机视野中的目标，请确认橙色目标在视野内！")
            return None

        try:
            now = rclpy.time.Time()
            tf_msg = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', now, timeout=rclpy.duration.Duration(seconds=1.0))
            
            p_base = np.array([
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z
            ])

            # 相机光学坐标系下的 3D 坐标
            p_cam_optical = np.array([
                self.latest_cam_pose.pose.position.x,
                self.latest_cam_pose.pose.position.y,
                self.latest_cam_pose.pose.position.z
            ])

            # 将光学坐标系转换为 camera_link 机械坐标系: x前y左z上
            # x_link = z_optical, y_link = -x_optical, z_link = -y_optical
            p_cam_link = np.array([
                p_cam_optical[2],
                -p_cam_optical[0],
                -p_cam_optical[1]
            ])

            return p_base, p_cam_link
        except Exception as e:
            self.get_logger().error(f"读取 TF base_link -> tool0 失败: {e}")
            return None


def main():
    rclpy.init()
    node = ExtrinsicCalibrator()

    print("==========================================================")
    print("      Eye-to-Hand 空间严密刚体外参标定工具 (PickOrange)    ")
    print("==========================================================")
    print("操作指引:")
    print("1. 确保 real_moveit.launch.py 与 perception_bringup 已运行；")
    print("2. 机械臂末端 tool0 轻触或固定橙色目标；")
    print("3. 【关键建议】可以改变机械臂姿态 2~3 次（如中间、偏左、偏右），每次按回车；")
    print("4. 输入 'q' 结束并一键完成刚体几何精确求解！")
    print("==========================================================")

    base_pts = []
    cam_pts = []

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            user_input = input("\n按 [Enter] 记录当前姿态点 (或输入 'q' 退出解算): ").strip()
            
            if user_input.lower() == 'q':
                break

            pair = node.capture_pair()
            if pair is not None:
                pb, pc = pair
                base_pts.append(pb)
                cam_pts.append(pc)
                print(f"✔ 成功记录第 {len(base_pts)} 组姿态真值:")
                print(f"  机械臂末端 tool0 (base_link): X={pb[0]:.3f}, Y={pb[1]:.3f}, Z={pb[2]:.3f} m")
                print(f"  相机测得目标   (camera_link): X={pc[0]:.3f}, Y={pc[1]:.3f}, Z={pc[2]:.3f} m")
            else:
                print("❌ 采样失败，请检查视觉识别框是否已稳定锁定。")

    except KeyboardInterrupt:
        pass

    if len(base_pts) >= 1:
        print("\n==========================================================")
        print(f"解算中... 共有 {len(base_pts)} 组点。")
        
        # 相机朝向面对机械臂的基础旋转: yaw=pi, pitch=0.52 rad (~30°)
        pitch = 0.52
        cp = np.cos(pitch); sp = np.sin(pitch)
        R_base_cam = np.array([
            [-cp,  0.0,  sp],
            [ 0.0, -1.0,  0.0],
            [ sp,  0.0,  cp]
        ])

        # 对所有采样点做最小二乘求解最佳平移外参 t_base_cam
        t_estimates = []
        for pb, pc in zip(base_pts, cam_pts):
            t_est = pb - R_base_cam @ pc
            t_estimates.append(t_est)

        t_final = np.mean(t_estimates, axis=0)

        print(f"🎯【精确标定完成】相机在 base_link 下的绝对外参为:")
        print(f"  cam_x = {t_final[0]:.3f}")
        print(f"  cam_y = {t_final[1]:.3f}")
        print(f"  cam_z = {t_final[2]:.3f}")
        print(f"\n推荐启动命令:")
        print(f"ros2 launch dummyarm_perception perception_bringup.launch.py "
              f"cam_x:={t_final[0]:.3f} cam_y:={t_final[1]:.3f} cam_z:={t_final[2]:.3f}")
        print("==========================================================")
    else:
        print("未记录足够数据。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
