#!/usr/bin/env python3
"""
orange_detector_node.py - 实时柑橘视觉检测、深度反投影与时域稳定滤波节点

功能特点：
1. 订阅 RealSense RGB 图像与对齐深度图 (aligned_depth_to_color)；
2. 基于 HSV 色彩空间自适应过滤与形态学开闭运算提取柑橘果实连通域；
3. 基于相机内参 (fx, fy, cx, cy) 将 2D 像素坐标 (u, v) 反投影为相机坐标系下的 3D 空间坐标 (X, Y, Z)；
4. 内置时域滑动窗口滤波（Median/Mean 滤波），当目标稳定存在超过 N 帧且波动 < 5mm 时锁定发布；
5. 输出 PoseStamped、PointStamped 及 RViz 可视化 3D Marker。
"""

import math
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge


class OrangeDetectorNode(Node):
    def __init__(self):
        super().__init__('orange_detector_node')

        # 声明可配置参数
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        
        # 默认柑橘橙色 HSV 区间 (可根据现场自然光调节)
        self.declare_parameter('hsv_h_min', 5)
        self.declare_parameter('hsv_h_max', 25)
        self.declare_parameter('hsv_s_min', 120)
        self.declare_parameter('hsv_s_max', 255)
        self.declare_parameter('hsv_v_min', 80)
        self.declare_parameter('hsv_v_max', 255)

        # 空间范围过滤 (单位：米)
        self.declare_parameter('min_depth', 0.15)
        self.declare_parameter('max_depth', 1.20)
        self.declare_parameter('min_radius_px', 15)

        # 时域稳定性滤波队列长度
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('stability_threshold_m', 0.01) # 1cm 抖动容限

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.camera_frame_id = 'camera_color_optical_frame'

        # 缓存最新帧（线程安全双缓冲机制，单向读写）
        self.latest_depth_img = None
        self.history_positions = deque(maxlen=self.get_parameter('filter_window_size').value)

        # 发布器
        self.target_pose_pub = self.create_publisher(PoseStamped, '/dummyarm/target_orange_pose', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/dummyarm/target_orange_marker', 10)
        self.debug_image_pub = self.create_publisher(Image, '/dummyarm/vision_debug_image', 10)

        # 订阅器 (使用 Best Effort / SensorData 模式适配真实相机流)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, sensor_qos)
        self.color_sub = self.create_subscription(
            Image, self.color_topic, self.color_callback, sensor_qos)

        self.get_logger().info("【柑橘感知节点】已启动，正在等待相机数据流...")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_intrinsics is None:
            # 内参矩阵 K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            fx = msg.k[0]
            cx = msg.k[2]
            fy = msg.k[4]
            cy = msg.k[5]
            self.camera_intrinsics = (fx, fy, cx, cy)
            self.camera_frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_color_optical_frame'
            self.get_logger().info(
                f"【内参锁定】成功获取相机内参: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}, Frame: {self.camera_frame_id}")

    def depth_callback(self, msg: Image):
        try:
            # RealSense 对齐深度图格式为 16UC1 (单位: 毫米 mm)
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"深度图像解析异常: {e}")

    def color_callback(self, msg: Image):
        if self.camera_intrinsics is None or self.latest_depth_img is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"彩色图像解析异常: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 读取动态色彩阈值
        h_min = self.get_parameter('hsv_h_min').value
        h_max = self.get_parameter('hsv_h_max').value
        s_min = self.get_parameter('hsv_s_min').value
        s_max = self.get_parameter('hsv_s_max').value
        v_min = self.get_parameter('hsv_v_min').value
        v_max = self.get_parameter('hsv_v_max').value

        lower_orange = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper_orange = np.array([h_max, s_max, v_max], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # 形态学滤波：消除小斑点并平滑边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_target = None
        max_area = 0
        min_radius = self.get_parameter('min_radius_px').value

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < (math.pi * min_radius * min_radius):
                continue

            (u, v), radius = cv2.minEnclosingCircle(cnt)
            if radius < min_radius:
                continue

            # 圆形度检查，滤除狭长背景
            perimeter = cv2.arcLength(cnt, True)
            if perimeter > 0:
                circularity = 4 * math.pi * (area / (perimeter * perimeter))
                if circularity < 0.45:
                    continue

            if area > max_area:
                max_area = area
                best_target = (int(u), int(v), int(radius))

        debug_img = cv_image.copy()

        if best_target is not None:
            u, v, r = best_target
            # 绘制检测框与圆形
            cv2.circle(debug_img, (u, v), r, (0, 255, 0), 2)
            cv2.circle(debug_img, (u, v), 3, (0, 0, 255), -1)

            # 在深度图中提取中心区域的中位数深度（抗边缘空洞与反光干扰）
            h, w = self.latest_depth_img.shape
            u_min = max(0, u - 5)
            u_max = min(w, u + 5)
            v_min = max(0, v - 5)
            v_max = min(h, v + 5)

            roi_depth = self.latest_depth_img[v_min:v_max, u_min:u_max]
            valid_depths = roi_depth[roi_depth > 0]

            if len(valid_depths) > 0:
                depth_mm = np.median(valid_depths)
                depth_m = depth_mm / 1000.0

                min_d = self.get_parameter('min_depth').value
                max_d = self.get_parameter('max_depth').value

                if min_d <= depth_m <= max_d:
                    # 反投影求解相机坐标系下的 3D 坐标
                    fx, fy, cx, cy = self.camera_intrinsics
                    X_c = (u - cx) * depth_m / fx
                    Y_c = (v - cy) * depth_m / fy
                    Z_c = depth_m

                    self.process_spatial_measurement(X_c, Y_c, Z_c, msg.header.stamp, debug_img, (u, v))

        # 发布调试图像
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            pass

    def process_spatial_measurement(self, x, y, z, stamp, debug_img, uv):
        # 加入时域滤波队列
        self.history_positions.append(np.array([x, y, z]))

        window_size = self.get_parameter('filter_window_size').value
        if len(self.history_positions) < window_size:
            return

        pts = np.array(self.history_positions)
        # 计算滑动中位数
        median_pos = np.median(pts, axis=0)

        # 计算抖动偏差
        deviations = np.linalg.norm(pts - median_pos, axis=1)
        max_dev = np.max(deviations)

        stability_thresh = self.get_parameter('stability_threshold_m').value
        if max_dev > stability_thresh:
            cv2.putText(debug_img, f"Unstable (dev={max_dev*100:.1f}cm)", (uv[0] - 40, uv[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            return

        # 稳定目标锁定发布
        target_x, target_y, target_z = median_pos

        self.get_logger().info(
            f"🎯【目标锁定发布】柑橘果心坐标: X={target_x:.3f}m, Y={target_y:.3f}m, 深度Z={target_z:.3f}m (Frame: {self.camera_frame_id})",
            throttle_duration_sec=1.0
        )

        cv2.putText(debug_img, f"LOCKED: X={target_x:.2f} Y={target_y:.2f} Z={target_z:.2f}m",
                    (uv[0] - 60, uv[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 1. 发布 PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame_id
        pose_msg.pose.position.x = float(target_x)
        pose_msg.pose.position.y = float(target_y)
        pose_msg.pose.position.z = float(target_z)
        # 默认姿态对齐相机光轴
        pose_msg.pose.orientation.w = 1.0

        self.target_pose_pub.publish(pose_msg)

        # 2. 发布 RViz Marker
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.camera_frame_id
        marker.ns = "citrus"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        # 柑橘尺寸 (直径约 6.5cm)
        marker.scale.x = 0.065
        marker.scale.y = 0.065
        marker.scale.z = 0.065
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.9

        self.target_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = OrangeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
