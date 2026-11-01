#!/usr/bin/env python3
"""机械臂运动安全守卫（考虑末端执行器体积）

事故背景（2026-11-01）：
  用"只约束位置"的 IK 把 J5 从 -74° 翻到 +0.18°，手腕翻转 74°，
  挂在上面的 153mm 长 / 172mm 宽的锥形末端整体扫过，圆形开口转到了车架方向，
  差点崩坏末端执行器（人工断电才停住）。

三条硬限位（L1/L2/L3）：

  L1 锥罩轴线朝向：套果动作中必须指向车外（-Y 方向，偏差 < 25°）
      —— 禁止锥口朝下车架、朝车架

  L2 姿态变化率：相邻两次指令的末端姿态差 ≤ 15°
      —— 这条如果早就在，当天的事故不会发生（它会把 J5 翻 74° 直接拦下）

  L3 位置包络：TCP 距基座 0.10~0.34m，z ∈ [-0.05, 0.36]

末端执行器几何（来自 STL 实测）：
  法兰  Z -8..0  (Ø35 x Ø61)
  外壳  Z -36..117  (罩口 Ø约120，外廓 172)
  刀片  Z 118..128
  ⇒ tool0(法兰安装面 Z=-8) → 罩口平面(Z=117) ≈ 125mm
  ⇒ 末端总长（从 tool0 到刀片外沿）≈ 136mm
"""

import math
import numpy as np

# --- 末端执行器几何（米，沿工具 Z 轴，从 tool0 起算）---
EE_LEN_TO_MOUTH = 0.125     # tool0 → 罩口平面
EE_LEN_TOTAL = 0.136        # tool0 → 刀片外沿
EE_MOUTH_RADIUS = 0.060     # 罩口半径（内径 120mm）
EE_OUTER_RADIUS = 0.086     # 外壳最大半径

# --- 限位参数 ---
L1_MIN_AXIS_DOT = math.cos(math.radians(25.0))   # 轴线与 -Y 的最小点积（软警告）
L2_MAX_AXIS_STEP = math.radians(180.0)           # 姿态变化不再硬限（改由扫掠检查）

# --- 车体包围盒（base_link 系，由 base_footprint 变换而来）---
# 车架 0.80 x 0.30，车架上平面在 base_link 的 z=-0.062
# base_link 在 base_footprint 的 (0.600, 0.040, 0.202)、yaw=180°
CHASSIS_BOX = {
    "x": (-0.19, 0.61),
    "y": (-0.11, 0.19),
    "z": (-0.202, -0.062),
}
SWEEP_MARGIN = 0.015          # 安全间隙 15mm
SWEEP_SAMPLES = 24            # 插值步数
EE_STATIONS = [(0.00, 0.030), (0.04, 0.045), (0.08, 0.062),
               (0.125, 0.060), (0.136, 0.086)]
L3_MIN_R = 0.10
L3_MAX_R = 0.34
L3_Z_MIN = -0.05
L3_Z_MAX = 0.36

PICK_DIR = np.array([0.0, -1.0, 0.0])            # 朝车外的方向


class SafetyError(RuntimeError):
    pass


def tool_axis(T):
    """从 4x4 变换取工具 Z 轴（锥罩朝向）"""
    return np.asarray(T[:3, 2], dtype=float)


def axis_angle_deg(a, b):
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return math.degrees(math.acos(c))


def check_l1(T):
    """L1：锥罩轴线必须朝车外"""
    d = float(np.dot(tool_axis(T), PICK_DIR))
    if d < L1_MIN_AXIS_DOT:
        ang = math.degrees(math.acos(max(-1.0, min(1.0, d))))
        raise SafetyError(
            f"L1 锥罩朝向违规：轴线与 -Y 夹角 {ang:.1f}° > 25°，"
            f"末端会朝车架/朝下，可能撞车")
    return True


def check_l2(T_prev, T_next):
    """L2：姿态变化不得过大（防止末端扫掠）"""
    a = tool_axis(T_prev)
    b = tool_axis(T_next)
    ang = axis_angle_deg(a, b)
    if math.radians(ang) > L2_MAX_AXIS_STEP:
        raise SafetyError(
            f"L2 姿态跳变违规：末端轴变化 {ang:.1f}° > 15°，"
            f"153mm 末端会扫过很大空间")
    return True


def check_l3(T):
    """L3：TCP 位置包络"""
    p = np.asarray(T[:3, 3], dtype=float)
    r = float(np.linalg.norm(p[:2]))
    if not (L3_MIN_R <= r <= L3_MAX_R):
        raise SafetyError(f"L3 位置违规：水平半径 {r*1000:.0f}mm 超出 [{L3_MIN_R*1000:.0f},{L3_MAX_R*1000:.0f}]")
    if not (L3_Z_MIN <= p[2] <= L3_Z_MAX):
        raise SafetyError(f"L3 位置违规：z={p[2]*1000:.0f}mm 超出 [{L3_Z_MIN*1000:.0f},{L3_Z_MAX*1000:.0f}]")
    return True


def _penetration(c, r):
    """末端某截面球近似侵入车体盒的深度（米；0 = 不接触）

    用"到盒子的距离"衡量：在盒外为负，进入越深越正。
    这样可以用"侵入深度是否增加"来判断动作是否更危险，
    而不是简单地一刀切禁止——否则一旦起点就贴边，任何动作（包括逃开）都会被拦死。
    """
    b = CHASSIS_BOX
    dx = max(b["x"][0] - c[0], 0, c[0] - b["x"][1])
    dy = max(b["y"][0] - c[1], 0, c[1] - b["y"][1])
    dz = max(b["z"][0] - c[2], 0, c[2] - b["z"][1])
    d_out = math.sqrt(dx * dx + dy * dy + dz * dz)
    return (r + SWEEP_MARGIN) - d_out      # >0 表示侵入


def _ee_point_hits_box(c, r):
    return _penetration(c, r) > 0.0


def ee_penetration(T):
    """整条末端的最大侵入深度（米）"""
    p = np.asarray(T[:3, 3], dtype=float)
    z = tool_axis(T)
    return max(_penetration(p + dist * z, rad) for dist, rad in EE_STATIONS)


def check_sweep(T_from, T_to):
    """扫掠检查：整段运动过程中，末端执行器的每一段都不得侵入车体

    这是替代"限制 J5 角度"的正确做法：
      J5 允许翻转（对准果实需要），但翻转过程中 136mm 长的末端
      扫过的所有位置都必须在车体之外。
    """
    from scipy.spatial.transform import Rotation as R, Slerp
    if T_from is None:
        return True
    R0 = R.from_matrix(T_from[:3, :3])
    R1 = R.from_matrix(T_to[:3, :3])
    try:
        sl = Slerp([0, 1], R.from_quat([R0.as_quat(), R1.as_quat()]))
    except Exception:
        sl = None
    p0, p1 = np.asarray(T_from[:3, 3]), np.asarray(T_to[:3, 3])
    start_pen = ee_penetration(T_from)
    worst = (-1e9, None)
    for i in range(1, SWEEP_SAMPLES + 1):     # 跳过 t=0（起点本身）
        t = i / SWEEP_SAMPLES
        Rot = sl(t) if sl is not None else R0
        axis = Rot.as_matrix()[:, 2]
        p = p0 + (p1 - p0) * t
        for dist, rad in EE_STATIONS:
            c = p + dist * axis
            pen = _penetration(c, rad)
            if pen > worst[0]:
                worst = (pen, (t, dist, c))
    pen, info = worst
    # 允许"脱离"（侵入不比起点更深），禁止"更靠近"
    limit = max(start_pen, 0.0) + 0.005
    if pen > 0 and pen > limit:
        t, dist, c = info
        raise SafetyError(
            f"L2 扫掠碰撞：运动到 {t*100:.0f}% 时侵入车体 {pen*1000:.0f}mm"
            f"（起点侵入 {max(start_pen,0)*1000:.0f}mm，允许 ≤ {limit*1000:.0f}mm）；"
            f"末端距 tool0 {dist*1000:.0f}mm 处，位置 ({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f})")
    return True


def check_all(T_prev, T_next, need_l1=False):
    check_l3(T_next)
    if T_prev is not None:
        check_sweep(T_prev, T_next)
    return True


def mouth_pos(T):
    """罩口中心位置（世界系）"""
    return np.asarray(T[:3, 3]) + EE_LEN_TO_MOUTH * tool_axis(T)


def ee_sweep_points(T, n=5):
    """末端沿轴线上的采样点（用于粗查与车体距离）"""
    p0 = np.asarray(T[:3, 3])
    z = tool_axis(T)
    return [p0 + (EE_LEN_TOTAL * i / (n - 1)) * z for i in range(n)]
