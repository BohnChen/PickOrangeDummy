#!/usr/bin/env python3
"""
camera_smoke_test.py - 相机点检与帧率、分辨率测试脚本
验证 RealSense D435i 底层数据流是否正常输出 RGB、深度和对齐数据
"""
import time
import pyrealsense2 as rs
import numpy as np

def main():
    print(">>> 正在初始化 RealSense 管道...")
    pipeline = rs.pipeline()
    config = rs.config()

    # 启用彩色与深度流 (640x480 @ 30fps)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    try:
        profile = pipeline.start(config)
        print(">>> 管道启动成功！正在捕获数据帧...")

        align_to = rs.stream.color
        align = rs.align(align_to)

        frames_tested = 0
        depth_center_list = []

        start_time = time.time()
        while frames_tested < 30:
            frames = pipeline.wait_for_frames(5000)
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # 读取中心点深度值
            w, h = depth_frame.get_width(), depth_frame.get_height()
            dist = depth_frame.get_distance(w // 2, h // 2)
            depth_center_list.append(dist)
            frames_tested += 1

        elapsed = time.time() - start_time
        fps = frames_tested / elapsed

        avg_dist = np.mean([d for d in depth_center_list if d > 0]) if depth_center_list else 0.0

        print(f"=====================================")
        print(f"✔ 相机点检通过！")
        print(f"✔ 分辨率: 640 x 480 (对齐 RGB + Depth)")
        print(f"✔ 实测采样帧率: {fps:.2f} FPS")
        print(f"✔ 画面中心当前测量距离: {avg_dist:.3f} 米")
        print(f"=====================================")

    except Exception as e:
        print(f"❌ 测试出现异常: {e}")
    finally:
        pipeline.stop()
        print(">>> 管道已安全关闭。")

if __name__ == '__main__':
    main()
