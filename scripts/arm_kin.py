#!/usr/bin/env python3
"""机械臂运动学扩展：完整 FK（含姿态）+ 带轴向约束的位姿 IK

为什么需要它：
  原有的 solve_exact_tool0_ik 只约束位置，姿态靠"J4/J5 靠 0"的正则项，
  所以锥形末端永远水平朝前。瞄准略低/略偏的果实时，锥壁会先碰到果实
  把它推走，而不是套进去。

  这里增加"工具 Z 轴指向"的约束：锥罩轴线始终对准目标，
  进刀时像"舀"一样把果实兜进来。

用法：
    from arm_kin import fk_full, solve_pose_ik
    q, res = solve_pose_ik(target_xyz, axis_dir, seed)
"""

import math
import numpy as np
from scipy.optimize import minimize

from instant_tap import (get_dh_trans, get_rot_z,
                         T_BASE_J1, T_J1_J2, T_J2_J3,
                         T_J3_J4, T_J4_J5, T_J5_J6)

# 固件限位换算到模型角（弧度），与 arm_goto_xyz.py 保持一致
BOUNDS = [
    (-2.96, 2.96),   # J1
    (-0.02, 2.84),   # J2
    (-2.53, 0.03),   # J3  ← 红线
    (-3.14, 3.14),   # J4
    (-1.66, 1.66),   # J5
    (-12.56, 12.56), # J6
]


def bounds_locked(lock_j6=None):
    """返回限位表；给定 lock_j6 时把第6轴钉死在该值（弧度）"""
    b = [list(x) for x in BOUNDS]
    if lock_j6 is not None:
        v = float(lock_j6)
        b[5] = [v, v]
    return b


def fk_full(q):
    """完整正解：返回 4x4 变换（base_link ← tool0）"""
    T = np.eye(4)
    T = T @ T_BASE_J1 @ get_rot_z(q[0])
    T = T @ T_J1_J2 @ get_rot_z(q[1])
    T = T @ T_J2_J3 @ get_rot_z(q[2])
    T = T @ T_J3_J4 @ get_rot_z(q[3])
    T = T @ T_J4_J5 @ get_rot_z(q[4])
    T = T @ T_J5_J6 @ get_rot_z(q[5])
    return T


def solve_pose_ik(target_xyz, axis_des, seed=None,
                  pos_tol_m=0.001, axis_tol=0.05, posture_w=0.0002,
                  lock_j6=None):
    """位置 + 工具 Z 轴方向约束的 IK

    target_xyz : base_link 下的目标位置 (m)
    axis_des   : 期望的"工具 Z 轴"方向（单位向量），锥罩开口朝向
    seed       : 初值（6 维，模型角）
    返回 (q, pos_err_mm, axis_err_deg)
    """
    target_xyz = np.asarray(target_xyz, dtype=float)
    axis_des = np.asarray(axis_des, dtype=float)
    axis_des = axis_des / (np.linalg.norm(axis_des) + 1e-12)

    if seed is None:
        seed = [math.atan2(target_xyz[0], -target_xyz[1]), 0.8, -0.8, 0.0, 0.0, 0.0]

    w_pos = 1.0 / (pos_tol_m ** 2)
    w_axis = 1.0 / (axis_tol ** 2)

    def obj(q):
        T = fk_full(q)
        pe = float(np.sum((T[:3, 3] - target_xyz) ** 2))
        ze = float(np.sum((T[:3, 2] - axis_des) ** 2))
        return w_pos * pe + w_axis * ze + posture_w * float(np.sum(np.square(np.asarray(q[3:]))))

    s0 = np.asarray(seed, dtype=float).copy()
    if lock_j6 is not None:
        s0[5] = float(lock_j6)
    res = minimize(obj, s0, bounds=bounds_locked(lock_j6),
                   method="L-BFGS-B",
                   options={"ftol": 1e-12, "gtol": 1e-10, "maxiter": 400})
    T = fk_full(res.x)
    pos_err_mm = float(np.linalg.norm(T[:3, 3] - target_xyz) * 1000.0)
    cosang = float(np.clip(np.dot(T[:3, 2], axis_des), -1.0, 1.0))
    axis_err_deg = float(math.degrees(math.acos(cosang)))
    return res.x, pos_err_mm, axis_err_deg


def fw_from_model(m):
    d = [math.degrees(v) for v in m]
    return [d[0], d[1] - 73.0, d[2] + 180.0, d[3], -d[4], d[5]]


def model_from_fw(fw):
    m = [fw[0], fw[1] + 73.0, fw[2] - 180.0, fw[3], -fw[4], fw[5]]
    return [math.radians(v) for v in m]


# ---------------------------------------------------------------------------
# 完整 6 自由度位姿 IK + 出料口朝向约束
#
# 背景：末端外壳的 −Y_tool 方向是圆形出料口（STL 外壳 Y ∈ −128..+88，
# 多出的 40mm 就是出料槽）。出料口要接管道往下排果，所以必须朝下。
# 只约束"锥罩轴线"是不够的——绕轴线的滚转（J4/J6）仍可把出料口翻到朝天。
#
# 约定：出料口方向 = −Y_tool；要求 Y_tool 尽量朝上（+Z）。
# ---------------------------------------------------------------------------

def tool_frame_from_axis(axis_des):
    """给定锥罩指向，构造"出料口朝下"的末端目标姿态

    z = 锥罩轴线
    x = 水平（与世界 Z 垂直）
    y = z × x  → 指向尽可能朝天，因此出料口 (−y) 尽可能朝地
    """
    z = np.asarray(axis_des, dtype=float)
    z = z / (np.linalg.norm(z) + 1e-12)
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z, up))) > 0.95:
        up = np.array([0.0, -1.0, 0.0])
    x = np.cross(up, z)
    x /= (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0] = x
    T[:3, 1] = y
    T[:3, 2] = z
    return T


def rot_error_rad(Ra, Rb):
    """两个旋转矩阵之间的夹角（弧度）"""
    R = Ra.T @ Rb
    tr = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(tr)


def solve_full_ik(target_T, seed=None, pos_tol_m=0.001, rot_tol=0.05,
                  posture_w=0.0002, roll_w=1.0, lock_j6=None):
    """完整 6 自由度 IK：位置 + 姿态（含滚转）

    target_T : 4x4 目标位姿（base_link ← tool0）
    返回 (q, pos_err_mm, rot_err_deg)
    """
    target_T = np.asarray(target_T, dtype=float)
    p_des = target_T[:3, 3]
    R_des = target_T[:3, :3]

    if seed is None:
        seed = [math.atan2(p_des[0], -p_des[1]), 0.8, -0.8, 0.0, 0.0, 0.0]

    w_pos = 1.0 / (pos_tol_m ** 2)
    w_rot = roll_w / (rot_tol ** 2)

    def obj(q):
        T = fk_full(q)
        pe = float(np.sum((T[:3, 3] - p_des) ** 2))
        re = rot_error_rad(R_des, T[:3, :3])
        return w_pos * pe + w_rot * re * re + posture_w * float(np.sum(np.square(np.asarray(q[3:]))))

    if lock_j6 is not None:
        seed = list(seed); seed[5] = float(lock_j6)
    best = None
    for s in (seed, [seed[0], 0.8, -0.8, 0.0, 0.6, seed[5]],
              [seed[0], 0.5, 0.0, 1.2, 0.6, seed[5]],
              [seed[0], 1.0, -1.2, -1.2, 0.3, seed[5]]):
        res = minimize(obj, np.asarray(s, dtype=float), bounds=bounds_locked(lock_j6),
                       method="L-BFGS-B",
                       options={"ftol": 1e-12, "gtol": 1e-10, "maxiter": 600})
        if best is None or res.fun < best.fun:
            best = res
    T = fk_full(best.x)
    pos_err_mm = float(np.linalg.norm(T[:3, 3] - p_des) * 1000.0)
    rot_err_deg = float(math.degrees(rot_error_rad(R_des, T[:3, :3])))
    return best.x, pos_err_mm, rot_err_deg


def port_down_deg(T):
    """出料口朝上/朝下的角度：0° = 正下方，180° = 正上方"""
    y = np.asarray(T[:3, 1], dtype=float)
    return float(math.degrees(math.acos(np.clip(y[2], -1.0, 1.0))))
