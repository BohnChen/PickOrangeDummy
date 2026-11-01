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
                  pos_tol_m=0.001, axis_tol=0.05, posture_w=0.0002):
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

    res = minimize(obj, np.asarray(seed, dtype=float), bounds=BOUNDS,
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
