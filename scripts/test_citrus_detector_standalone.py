#!/usr/bin/env python3
"""
test_citrus_detector_standalone.py
独立测试柑橘感知算法（无需预先标定和机械臂动作）
订阅实时相机流，打印检测到的果心 3D 坐标与时域滤波状态
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class PerceptionValidator(Node):
    def __init__(self):
        super().__init__('perception_validator')
        self.sub = self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.pose_callback, 10)
        self.pose_count = 0

    def pose_callback(self, msg: PoseStamped):
        self.pose_count += 1
        pos = msg.pose.position
        print(f"[{self.pose_count}] ✔ 检测到稳定目标! 相机系坐标: X={pos.x:.3f}m, Y={pos.y:.3f}m, 深度Z={pos.z:.3f}m (Frame: {msg.header.frame_id})")

def main():
    rclpy.init()
    node = PerceptionValidator()
    print(">>> 正在监听目标柑橘 3D 位姿话题 /dummyarm/target_orange_pose ...")
    print(">>> 提示：可以在相机前方 0.3m ~ 1.0m 范围内放置一个橙色物体（如柑橘、橘黄色杯子、玩具等）进行测试。")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
