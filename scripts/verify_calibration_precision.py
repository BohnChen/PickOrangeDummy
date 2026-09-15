#!/usr/bin/env python3
"""
verify_calibration_precision.py - 手眼标定精度直观定量自检工具

【终极验证方法】:
如何确定标定结果准不准？不需要猜测，只需一个最简单、绝对无可辩驳的物理动作：
1. 把黄色物体固定在机械臂末端 tool0 正中心（或者用末端 J6 尖端顶住物体）；
2. 运行本工具，它会同时读取：
   - 机械臂运动学报告的真实物理末端坐标: P_robot_tool0 (base_link)
   - 相机通过手眼外参矩阵推算出的物体坐标: P_camera_target (base_link)
3. 打印两者之间的三维空间欧氏距离差（误差）：
   - 如果误差在 < 10mm (1厘米以内)，说明标定极其精准！
   - 如果误差在 100~200mm (10~20厘米)，说明标定外参存在平移或旋转偏差，并直接给出具体的偏置分量 (dX, dY, dZ)！
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class CalibrationPrecisionVerifier(Node):
    def __init__(self):
        super().__init__('calib_verifier')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_orange_cam_pose = None
        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.orange_callback, 10)

    def orange_callback(self, msg: PoseStamped):
        self.latest_orange_cam_pose = msg

    def check_alignment(self):
        if self.latest_orange_cam_pose is None:
            return None, None, "未检测到黄色目标，请确保目标在相机画面内且有绿框！"

        # 1. 查相机推算到 base_link 的目标位置
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                self.latest_orange_cam_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            p_cam_base = tf2_geometry_msgs.do_transform_pose_stamped(
                self.latest_orange_cam_pose, trans).pose.position
        except Exception as e:
            return None, None, f"TF 查找失败: {e}"

        # 2. 查机械臂当前的物理末端 tool0 位置
        try:
            trans_tool = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            p_robot = trans_tool.transform.translation
        except Exception as e:
            return None, None, f"读取机械臂末端 TF 失败: {e}"

        return p_robot, p_cam_base, None


def main():
    rclpy.init()
    node = CalibrationPrecisionVerifier()

    print("==================================================================")
    print("      PickOrangeDummy 手眼标定精度直观定量验证工具              ")
    print("==================================================================")
    print("【检验原理】:")
    print("把黄色目标直接贴在机械臂末端第 6 轴 (tool0) 处：")
    print("- 若标定准确，相机算出的坐标应与机械臂末端坐标 100% 重合 (误差 < 1cm)！")
    print("- 若存在偏差，本工具将实时打印出具体的偏差方向 (X, Y, Z 各差多少厘米)！")
    print("==================================================================")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            p_robot, p_cam, err_msg = node.check_alignment()

            if err_msg:
                print(f"\r⏳ {err_msg}", end="", flush=True)
                time.sleep(0.3)
                continue

            # 计算差值
            dx = p_cam.x - p_robot.x
            dy = p_cam.y - p_robot.y
            dz = p_cam.z - p_robot.z
            dist_err = math.sqrt(dx*dx + dy*dy + dz*dz) * 100.0 # 厘米

            print(f"\r[当前真值对比] 末端tool0:[{p_robot.x:.3f}, {p_robot.y:.3f}, {p_robot.z:.3f}] | "
                  f"相机推算:[{p_cam.x:.3f}, {p_cam.y:.3f}, {p_cam.z:.3f}] | "
                  f"总空间偏差: {dist_err:.1f} cm (dx:{dx*100:+.1f}, dy:{dy*100:+.1f}, dz:{dz*100:+.1f})",
                  end="", flush=True)

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
