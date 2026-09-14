#!/usr/bin/env python3
"""
web_camera_preview.py - 轻量局域网 Web 实时相机预览服务器
在浏览器 (Mac/手机/PC) 中打开 http://<板子IP>:8080 即可直接查看相机实时画面和橙色柑橘识别框！
无需配置 X11 转发或桌面 GUI，超低延迟 (<50ms)。
"""
import cv2
import numpy as np
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

class CameraStreamer:
    def __init__(self, device_id=4):
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.current_frame = None
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # 柑橘/橙色目标识别标定框实时叠加
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 80], dtype=np.uint8)
            upper_orange = np.array([25, 255, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 画辅助十字瞄准线 (画面正中心)
            h, w, _ = frame.shape
            cx, cy = w // 2, h // 2
            cv2.drawMarker(frame, (cx, cy), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)
            cv2.putText(frame, "Optical Center", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

            target_found = False
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 400: # 过滤杂色小点
                    (x, y), radius = cv2.minEnclosingCircle(cnt)
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)
                    cv2.putText(frame, f"Orange Area:{int(area)}", (int(x)-40, int(y)-int(radius)-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    target_found = True

            status_text = "Target LOCKED" if target_found else "Searching Orange..."
            status_color = (0, 255, 0) if target_found else (0, 165, 255)
            cv2.putText(frame, status_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            ret_jpg, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ret_jpg:
                with self.lock:
                    self.current_frame = jpeg.tobytes()

            time.sleep(0.03) # ~30 FPS

    def get_frame(self):
        with self.lock:
            return self.current_frame

    def stop(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()

streamer = None

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>RealSense D435i 实时预览与感知监视器</title>
                <style>
                    body { background-color: #1a1a1a; color: #eee; font-family: sans-serif; text-align: center; margin: 0; padding: 20px; }
                    h2 { margin-bottom: 10px; }
                    .video-box { display: inline-block; border: 2px solid #444; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
                    img { display: block; max-width: 100%; height: auto; }
                    .tip { margin-top: 15px; color: #aaa; font-size: 14px; }
                </style>
            </head>
            <body>
                <h2>🍊 RealSense D435i 实时画面监视器</h2>
                <div class="video-box">
                    <img src="/stream.mjpg" width="640" height="480"/>
                </div>
                <p class="tip">提示：将柑橘或橙色物体置于相机视野中，绿色线框吸附表示已被视觉锁定识别。<br>白十字为相机光学中心轴。</p>
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
                    frame = streamer.get_frame() if streamer else None
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

def main():
    global streamer
    print(">>> 启动相机捕获模块...")
    streamer = CameraStreamer(device_id=4)
    server_address = ('', 8080)
    server = HTTPServer(server_address, StreamingHandler)
    print(">>> 实时预览 Web 服务器已启动: http://0.0.0.0:8080")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if streamer:
            streamer.stop()
        server.server_close()

if __name__ == '__main__':
    main()
