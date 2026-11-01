#!/usr/bin/env python3
"""眼在手外（eye-to-hand）标定：D435i ↔ 机械臂 base_link

做法：
  1. 末端系着黄色标记，让机械臂走到 6 个已知空间点（IK 求解）
  2. 每个点：相机检测黄色标记 → 反投影成相机坐标系 3D 点
             同时用 FK 算出 tool0 在 base_link 的坐标
  3. SVD 求刚体变换 T(base_link ← camera_color_optical_frame)
  4. 换算成 transforms.yaml 里 d435i_link 的位姿并写回

为什么能这么标：
  6 个位姿的 J4/J5 都很接近 0，末端姿态基本恒定，
  所以"标记相对 tool0 的固定偏移"会被整体吸收进平移量，不污染旋转。

用法：
    ./scripts/calib_eye_to_hand.py --collect     # 采点（会动机械臂）
    ./scripts/calib_eye_to_hand.py --solve       # 只用已采数据求解
"""

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from instant_tap import solve_exact_tool0_ik, forward_kinematics_tool0

import importlib.util
spec = importlib.util.spec_from_file_location(
    "agx", os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_goto_xyz.py"))
agx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agx)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R

DATA = "/tmp/opencode/calib_points.json"
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_SIZE_M = 0.040          # 打印宽度 40mm；换尺寸必须改这里
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
# 默认参数在这台相机/光照下太严（标记只占 ~50px、画面偏暗），实测会漏检。
# 放宽阈值 + 亚像素角点精化：既能检出，也能把角点定位精度提到 0.1px 级。
ARUCO_PARAMS.adaptiveThreshConstant = 7
ARUCO_PARAMS.minMarkerPerimeterRate = 0.02
ARUCO_PARAMS.maxMarkerPerimeterRate = 4.0
ARUCO_PARAMS.polygonalApproxAccuracyRate = 0.05
ARUCO_PARAMS.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# 6 个标定位姿（base_link, 米）。都在相机可见的左前方，J3 保持 >45°
POSES = [
    (0.00, -0.28, 0.06),
    (0.00, -0.28, 0.10),
    (0.00, -0.28, 0.14),
    (0.00, -0.28, 0.16),
    (-0.07, -0.28, 0.10),
    (0.07, -0.28, 0.10),
]


class Cam(Node):
    def __init__(self):
        super().__init__("calib_cam")
        self.c = None
        self.d = None
        self.k = None
        self.create_subscription(Image, "/d435i/d435i/color/image_raw",
                                 lambda m: setattr(self, "c", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/d435i/d435i/aligned_depth_to_color/image_raw",
                                 lambda m: setattr(self, "d", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/d435i/d435i/color/camera_info",
                                 self._info, 10)

    def _info(self, msg):
        if self.k is None:
            self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])  # fx, fy, cx, cy


def grab(cam, timeout=6.0):
    cam.c = None
    cam.d = None
    t0 = time.time()
    while time.time() - t0 < timeout and (cam.c is None or cam.d is None or cam.k is None):
        rclpy.spin_once(cam, timeout_sec=0.1)
    if cam.c is None or cam.d is None or cam.k is None:
        return None, None
    img = np.frombuffer(cam.c.data, dtype=np.uint8).reshape(cam.c.height, cam.c.width, 3)
    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    dep = np.frombuffer(cam.d.data, dtype=np.uint16).reshape(cam.d.height, cam.d.width)
    return bgr, dep


def find_aruco(bgr, k):
    """ArUco + PnP：返回 (u, v, tvec) —— tvec 即标记中心在相机系的位置

    比"彩色质心+深度"稳得多：用 4 个已知角点解位姿，
    不依赖深度图质量，半遮挡也能识别。
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
    if ids is None or len(ids) == 0:
        return None
    idx = 0
    if len(ids) > 1:
        # 多个标记时取面积最大的
        areas = [cv2.contourArea(c.reshape(-1, 2).astype(np.float32)) for c in corners]
        idx = int(np.argmax(areas))
    c = corners[idx].reshape(4, 2)
    h = ARUCO_SIZE_M / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    K = np.array([[k[0], 0, k[2]], [0, k[1], k[3]], [0, 0, 1]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, c.astype(np.float64), K, np.zeros((1, 5)),
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    u = int(c[:, 0].mean())
    v = int(c[:, 1].mean())
    return (u, v, tvec.reshape(3))


def make_T_bl_opt():
    """假设外参：base_link ← camera_optical（来自 transforms.yaml）"""
    T_bf_bl = np.eye(4)
    T_bf_bl[:3, :3] = R.from_euler("xyz", [0, 0, math.pi]).as_matrix()
    T_bf_bl[:3, 3] = [0.600, 0.040, 0.202]
    T_bf_d4 = np.eye(4)
    T_bf_d4[:3, :3] = R.from_euler("xyz", [0, 0, math.radians(100)]).as_matrix()
    T_bf_d4[:3, 3] = [0.786, 0.0, 0.435]
    R_link_opt = R.from_euler("xyz", [0, 0, -math.pi / 2]).as_matrix() @ \
                 R.from_euler("xyz", [-math.pi / 2, 0, 0]).as_matrix()
    T_lo = np.eye(4)
    T_lo[:3, :3] = R_link_opt
    return np.linalg.inv(T_bf_bl) @ T_bf_d4 @ T_lo


def predict_pixel(p_base, T_bl_opt, k):
    """已知 base_link 点 → 预测像素 (u,v)，用于就近匹配标记"""
    fx, fy, cx, cy = k
    pc = T_bl_opt[:3, :3].T @ (np.array(p_base) - T_bl_opt[:3, 3])
    if pc[2] <= 0.05:
        return None
    return (fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy)


def find_marker(bgr, dep, expect=None):
    """返回 (u, v, depth_m, area) 或 None。

    选法：所有近距离黄色块里**深度最小**的那个。
    机械臂是画面里离相机最近的物体，背景黄物（木色、纸箱）都在 1.5m 以外，
    所以"最近"比"预测像素"可靠得多——预测本身依赖还没标定的外参，是循环论证。
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # 标记是"小黄鸭/熟香蕉"那种高饱和橙黄：实测 S≈172 V≈219，
    # 而背景里的木色/纸箱 S 只有 70~80。用高饱和+高亮度就能把它们完全分开。
    mask = cv2.inRange(hsv, np.array([12, 120, 110]), np.array([42, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for ct in cnts:
        a = cv2.contourArea(ct)
        if a < 400:          # 标记实测面积 2000 左右；小块噪点直接丢
            continue
        M = cv2.moments(ct)
        if M["m00"] <= 0:
            continue
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        # 贴边的不取：质心会被裁切带偏
        if u < 12 or u > 628 or v < 12 or v > 468:
            continue
        r = 6
        roi = dep[max(0, v - r):v + r, max(0, u - r):u + r]
        vv = roi[roi > 0]
        if vv.size < 8:
            continue
        dmm = float(np.median(vv))
        if not (150 < dmm < 900):
            continue
        if best is None or dmm < best[2]:
            best = (u, v, dmm / 1000.0, a)
    return best


def pixel_to_cam(u, v, z, k):
    fx, fy, cx, cy = k
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def wait_motion(ser, target_fw, timeout=90.0, tol=0.8):
    """等到实测关节角与目标角差 < tol 度。

    固件在运动期间不响应查询，所以读不到就继续等。**不能**用"两次读数相同"
    作为到位判据——慢速时相邻读数天然很接近，会误判（今天踩过）。
    位姿之间必须确认真的到位，否则会采到重复点，SVD 直接解出垃圾。
    """
    last = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        js = ser.joints()
        if js is None:
            time.sleep(0.4)
            continue
        last = js
        d = max(abs(js[i] - target_fw[i]) for i in range(6))
        if d < tol:
            time.sleep(0.5)      # 再稳一下
            return js
        time.sleep(0.3)
    return last


def collect():
    rclpy.init()
    cam = Cam()
    ser = agx.RefSerial(agx.PORT_CANDIDATES[0] if os.path.exists(agx.PORT_CANDIDATES[0])
                        else agx.PORT_CANDIDATES[1])
    ser.cmd("!START", 0.8)
    recs = []
    try:
        for i, p in enumerate(POSES):
            tgt = np.array(p)
            q, res = solve_exact_tool0_ik(tgt, seed=[0.3, 0.5, -0.3, 0, 0, 0])
            if res > 3.0:
                print(f"[{i}] {p} IK 残差 {res:.1f}mm，跳过")
                continue
            fw = agx.clamp_fw(agx.model_rad_to_fw(q))
            if not (45 < fw[2] < 179):
                print(f"[{i}] {p} J3={fw[2]:.1f}° 不安全，跳过")
                continue
            # 固件在运动中会丢弃新指令，所以必须"下发→校验→必要时重发"
            js = None
            for attempt in range(3):
                ser.write(">" + ",".join(f"{v:.3f}" for v in fw) + ",25.0")
                js = wait_motion(ser, fw, timeout=45.0)
                if js is not None and max(abs(js[i] - fw[i]) for i in range(6)) < 2.0:
                    break
                print(f"[{i}] 第 {attempt+1} 次下发未到位，重发")
                time.sleep(1.0)
            if js is None:
                print(f"[{i}] 运动后读不到角度，中止")
                break
            time.sleep(0.4)

            # 运动刚停时可能有运动模糊/自动曝光未稳定，连拍几帧取第一帧能识别的
            bgr = dep = None
            ar = None
            for _try in range(5):
                bgr, dep = grab(cam)
                if bgr is None:
                    time.sleep(0.2)
                    continue
                ar = find_aruco(bgr, cam.k)
                if ar is not None:
                    break
                time.sleep(0.35)
            if bgr is None:
                print(f"[{i}] 拍照失败")
                continue
            if ar is not None:
                u, v, p_cam = ar
                p_cam = np.array(p_cam, dtype=float)
                z = float(p_cam[2])
                area = 0.0
                src = "aruco"
            else:
                exp = predict_pixel(p, make_T_bl_opt(), cam.k)
                mk = find_marker(bgr, dep, expect=exp)
                if mk is None:
                    print(f"[{i}] {p} 到位但没看到标记（ArUco 也没识别到）")
                    cv2.imwrite(f"/tmp/opencode/calib_fail_{i}.png", bgr)
                    continue
                u, v, z, area = mk
                p_cam = np.array(pixel_to_cam(u, v, z, cam.k))
                src = "color"

            q_model = agx.fw_to_model_rad(js)
            p_base = forward_kinematics_tool0(q_model)
            recs.append({
                "target": list(p),
                "joints_fw": list(js),
                "pixel": [u, v],
                "depth_m": z,
                "p_cam": p_cam.tolist(),
                "p_base": list(map(float, p_base)),
                "area": area,
                "src": src,
            })
            print(f"[{i}] {p} [{src}] 像素=({u},{v}) 深度={z*1000:.0f}mm "
                  f"相机系=({p_cam[0]:+.3f},{p_cam[1]:+.3f},{p_cam[2]:+.3f}) "
                  f"FK=({p_base[0]:+.3f},{p_base[1]:+.3f},{p_base[2]:+.3f})")
    finally:
        ser.close()
        os.makedirs(os.path.dirname(DATA), exist_ok=True)
        with open(DATA, "w") as f:
            json.dump({"intrinsics": cam.k, "points": recs}, f, indent=2)
        print(f"\n已保存 {len(recs)} 个点 -> {DATA}")
        cam.destroy_node()
        rclpy.try_shutdown()
    return recs


def solve():
    with open(DATA) as f:
        blob = json.load(f)
    pts = blob["points"]
    if len(pts) < 3:
        print(f"点太少（{len(pts)}），至少需要 3 个")
        return 1
    A = np.array([p["p_cam"] for p in pts])   # camera
    B = np.array([p["p_base"] for p in pts])  # base_link
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    Rm = Vt.T @ D @ U.T
    t = cb - Rm @ ca

    pred = (Rm @ A.T).T + t
    err = np.linalg.norm(pred - B, axis=1) * 1000
    print("刚体变换 T(base_link ← camera_optical):")
    print("  R =\n", np.array2string(Rm, precision=4))
    print("  t =", np.array2string(t, precision=4))
    print(f"残差(mm): 每点 {np.round(err, 1).tolist()}")
    print(f"          平均 {err.mean():.1f}  最大 {err.max():.1f}")
    print(f"  旋转角 {math.degrees(np.arccos(max(-1, min(1, (np.trace(Rm)-1)/2)))):.1f}°")

    # 换算成 transforms.yaml 里的 d435i_link 位姿（base_footprint 系）
    # T_bf←bl : base_link 在 base_footprint 下的位姿（当前 yaw=180°）
    T_bf_bl = np.eye(4)
    T_bf_bl[:3, :3] = R.from_euler("xyz", [0, 0, math.pi]).as_matrix()
    T_bf_bl[:3, 3] = [0.600, 0.040, 0.202]

    T_bl_cam = np.eye(4)
    T_bl_cam[:3, :3] = Rm
    T_bl_cam[:3, 3] = t

    # optical → link 的标准旋转：R = Rz(-90°)·Rx(-90°)
    R_link_opt = R.from_euler("xyz", [0, 0, -math.pi / 2]).as_matrix() @ \
                 R.from_euler("xyz", [-math.pi / 2, 0, 0]).as_matrix()
    T_opt_link = np.eye(4)
    T_opt_link[:3, :3] = R_link_opt.T

    T_bf_link = T_bf_bl @ T_bl_cam @ T_opt_link
    xyz = T_bf_link[:3, 3]
    rpy = R.from_matrix(T_bf_link[:3, :3]).as_euler("xyz")
    print("\n→ transforms.yaml 里 d435i 应改为:")
    print(f"  xyz: [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]")
    print(f"  rpy: [{rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f}]   (yaw={math.degrees(rpy[2]):.1f}°)")
    out = "/tmp/opencode/calib_result.json"
    with open(out, "w") as f:
        json.dump({"T_base_link_camera": np.vstack([np.hstack([Rm, t.reshape(3, 1)]),
                                                    [0, 0, 0, 1]]).tolist(),
                   "d435i_xyz": xyz.tolist(),
                   "d435i_rpy": rpy.tolist(),
                   "residual_mm": err.tolist()}, f, indent=2)
    print(f"结果已保存 -> {out}")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--collect", action="store_true")
    ap.add_argument("--solve", action="store_true")
    args = ap.parse_args()
    if args.collect:
        collect()
        return solve()
    if args.solve:
        return solve()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
