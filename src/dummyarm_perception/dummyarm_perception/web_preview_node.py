#!/usr/bin/env python3
"""
ros2_web_preview.py - 基于 ROS2 图像流的实时 Web 监视器与柑橘检测回显
直接订阅 ROS 2 Jazzy 的 /camera/camera/color/image_raw 与 /dummyarm/vision_debug_image 话题，
并在浏览器 (http://<IP>:8080) 实时展示！
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

latest_jpeg = None
lock = threading.Lock()
target_info = "Waiting for Camera / Target..."

class WebPreviewNode(Node):
    def __init__(self):
        super().__init__('web_preview_node')
        self.bridge = CvBridge()
        self.create_subscription(
            Image, '/dummyarm/vision_debug_image', self.image_callback, 10)
        self.create_subscription(
            PoseStamped, '/dummyarm/target_orange_pose', self.pose_callback, 10)

    def image_callback(self, msg: Image):
        global latest_jpeg
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, jpeg = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ret:
                with lock:
                    latest_jpeg = jpeg.tobytes()
        except Exception as e:
            pass

    def pose_callback(self, msg: PoseStamped):
        global target_info
        p = msg.pose.position
        target_info = f"🎯 柑橘锁定! X={p.x:.3f}m, Y={p.y:.3f}m, 深度Z={p.z:.3f}m"

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8">
                <title>PickOrangeDummy 实时视觉监视器</title>
                <style>
                    body {{ background: #181818; color: #f0f0f0; font-family: -apple-system, sans-serif; text-align: center; margin: 0; padding: 20px; }}
                    h2 {{ color: #ff9900; margin-bottom: 5px; }}
                    .box {{ display: inline-block; border: 3px solid #333; border-radius: 10px; overflow: hidden; box-shadow: 0 8px 24px rgba(0,0,0,0.6); margin-top: 10px; }}
                    img {{ display: block; max-width: 100%; height: auto; }}
                    .status {{ margin-top: 15px; font-size: 18px; font-weight: bold; color: #00ff88; }}
                    .tip {{ color: #888; font-size: 14px; margin-top: 8px; }}
                </style>
            </head>
            <body>
                <h2>🍊 PickOrangeDummy 实时视觉监视面板</h2>
                <div class="box">
                    <img src="/stream.mjpg" width="640" height="480"/>
                </div>
                <div class="status" id="stat">{target_info}</div>
                <p class="tip">提示：绿框为算法提取出的柑橘轮廓，带 3D 深度反投影与时域滤波。<br>刷新浏览器或查看画面变化。</p>
            </body>
            </html>
            """
            self.wfile.write(html.encode('utf-8'))
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with lock:
                        frame = latest_jpeg
                    if frame:
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.04)
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()

def run_server():
    server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
    server.serve_forever()

def main():
    rclpy.init()
    node = WebPreviewNode()
    t = threading.Thread(target=run_server, daemon=True)
    t.start()
    print(">>> ROS 2 Web 视觉实时预览服务器已启动: http://0.0.0.0:8080")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
