#!/usr/bin/env python3
"""双相机实时预览（浏览器四分割 MJPEG 流）

订阅 Astra（/camera/*）+ D435i（/d435i/d435i/*），合成四分割画面，
通过 http://<板子IP>:8888 在任意浏览器查看。

用法：
    ./scripts/web_dual_cam.sh
    # 或
    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws_orbbec/install/setup.bash
    python3 scripts/web_dual_cam.py

注意：
  - 需要两台相机的节点已经在跑（Astra 用 orbbec_camera，D435i 用 realsense2_camera
    并加 camera_name:=d435i camera_namespace:=d435i，否则会撞在 /camera 上）
  - 端口被占用时先 pkill -f web_dual_cam.py
"""

import sys, time, threading
import cv2, numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

latest_jpeg = None
lock = threading.Lock()

class DualCamSub(Node):
    def __init__(self):
        super().__init__('web_dual_cam_sub')
        self.ac = None; self.ad = None; self.dc = None; self.dd = None
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_ac, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', self.cb_ad, qos_profile_sensor_data)
        self.create_subscription(Image, '/d435i/d435i/color/image_raw', self.cb_dc, qos_profile_sensor_data)
        self.create_subscription(Image, '/d435i/d435i/aligned_depth_to_color/image_raw', self.cb_dd, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.compose)

    def cb_ac(self, m): self.ac = m
    def cb_ad(self, m): self.ad = m
    def cb_dc(self, m): self.dc = m
    def cb_dd(self, m): self.dd = m

    def compose(self):
        global latest_jpeg
        def to_col(m, rgb=False):
            if m is None: return np.zeros((240, 320, 3), dtype=np.uint8)
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            if rgb: img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return cv2.resize(img, (320, 240))
        def to_dep(m, mx):
            if m is None: return np.zeros((240, 320, 3), dtype=np.uint8)
            a = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width).astype(np.float32)
            v = np.clip(a, 0, mx) / float(mx)
            vis = cv2.applyColorMap((255*(1-v)).astype(np.uint8), cv2.COLORMAP_JET)
            vis[a == 0] = (0, 0, 0)
            return cv2.resize(vis, (320, 240))

        ac = to_col(self.ac, rgb=True)
        ad = to_dep(self.ad, 3500)
        dc = to_col(self.dc, rgb=True)
        dd = to_dep(self.dd, 1500)

        cv2.putText(ac, "Astra Front Color", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(ad, "Astra Front Depth", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(dc, "D435i Side Color", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(dd, "D435i Side Depth", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        quad = np.vstack([np.hstack([ac, ad]), np.hstack([dc, dd])])
        ok, buf = cv2.imencode('.jpg', quad, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with lock:
                latest_jpeg = buf.tobytes()

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write('''<!DOCTYPE html>
<html><head><title>PickOrangeDualCam</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>body{background:#111;color:#fff;margin:0;display:flex;flex-direction:column;align-items:center;font-family:sans-serif;}
img{max-width:100%;height:auto;border:2px solid #444;border-radius:8px;margin-top:10px;}
</style></head>
<body><h2>PickOrange 双相机实时四分割画面</h2>
<img src="/stream.mjpg" />
</body></html>'''.encode('utf-8'))
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with lock:
                    frame = latest_jpeg
                if frame:
                    try:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    except Exception:
                        break
                time.sleep(0.04)
        else:
            self.send_error(404)
    def log_message(self, *args): pass

def ros_spin():
    rclpy.init()
    node = DualCamSub()
    rclpy.spin(node)

t = threading.Thread(target=ros_spin, daemon=True)
t.start()

server = HTTPServer(('0.0.0.0', 8888), StreamHandler)
print("Server ready on port 8888", flush=True)
server.serve_forever()
