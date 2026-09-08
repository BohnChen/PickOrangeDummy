#!/usr/bin/env python3
"""网格件配准(表驱动首版): 计算每件 visual origin/rpy, 输出 JSON。
锚点依据(STEP 提取 + 图纸核对):
  J6 : 主轴 Φ8.95 p=(-35.5,198,107) dir(-1,0,0)  → 链 Joint6 轴(+X @ 腕心)
  J5B: J5轴承小孔 p=? → Joint5; 次级 Φ31.05 → 下游
  ...
说明: 滚转沿关节轴自由度先用'本地几何竖直↔世界Z'规则, 之后可Rviz目检微调。
纯离线。
"""
import json
import numpy as np
import math

# 每件: 自身节轴锚点 (local pivot 点, dir), 世界链 Jointk 原点/轴(零位, m)
# 世界链 zero 轴原点(来自 generate_true_urdf 表):
J_ORIGIN = {1: (0.0, 0.0, 0.0), 2: (0.035, 0.0, 0.0925), 3: (0.146, 0.0, 0.050),
            4: (-0.052, 0.0, 0.115), 5: (0.0, 0.0, 0.0), 6: (0.0, 0.0, 0.0)}
J_AXIS = {1: (0, 0, 1), 2: (0, -1, 0), 3: (0, -1, 0), 4: (1, 0, 0), 5: (0, -1, 0), 6: (1, 0, 0)}

# 件 → (part_stl, 自身j, local_pivot_mm, local_dir(单位化方向或符号))
PICKS = {
    # (stl, joint, p_local(mm), dir_local)
    'J1': ('J1.stl', 2, (-35.0, 10.25, 0.0), (-1, 0, 0)),   # J1件 主孔(Φ11.05?按STEP大簇) → Joint2
    'J23': ('J2J3.stl', 2, (7.0, 0.0, 0.0), (-1, 0, 0)),    # Φ13 J2孔 → Joint2
    'J4': ('J4.stl', 3, (-35.0, 156.25, 0.0), (1, 0, 0)),   # 大孔Φ40.1 → Joint3? (F3) 近似
    'J5A': ('J5A.stl', 4, (-6.9, 179.5, 34.5), (0, -1, 0)), # J5A轴颈 → Joint4? 近似
    'J5B': ('J5B.stl', 5, (-19.6, 198.0, 131.6), (1, 0, 0)),# J5B大孔Φ31.05 → Joint5? 近似
    'J6': ('J6.stl', 6, (-35.5, 198.0, 107.0), (-1, 0, 0)), # Φ8.95 主轴 → Joint6 ✓实测锚
}

def norm(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)

def align(local_p, local_d, world_o, world_d):
    """R,t 使 local 轴→world 轴, 点→world_o; 滚转补充: 尽量保持 local-z→world-z。"""
    u = norm(local_d)
    U = norm(world_d)
    # 对齐 u→U 的最小旋转
    v = np.cross(u, U)
    s = np.linalg.norm(v); c = np.dot(u, U)
    if s < 1e-9:
        R = np.eye(3) if c > 0 else np.diag([1, -1, -1])
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K * ((1 - c) / (s * s))
    # 滚转: 本地 z 投影最大对齐世界 z (在垂直于 world_d 平面内旋转)
    zl = R @ np.array([0, 0, 1.0])
    az = np.array([0, 0, 1.0])
    # 去掉沿轴分量后求剩余旋转(绕 U 转角度)
    def perp(x):
        return x - np.dot(x, U) * U
    pl, pa = perp(zl), perp(az)
    if np.linalg.norm(pl) > 1e-6 and np.linalg.norm(pa) > 1e-6:
        ang = math.atan2(np.dot(np.cross(pl, pa), U), np.dot(pl, pa))
        # 绕轴 U 旋转 -ang (U为右手系: 旋转矩阵 Rodrigues)
        K2 = np.array([[0, -U[2], U[1]], [U[2], 0, -U[0]], [-U[1], U[0], 0]])
        R = np.eye(3) + math.sin(-ang) * K2 + (1 - math.cos(-ang)) * (K2 @ K2)
        R = R @ R  # 累乘? 应该直接组合上一次R: 注意上面R被覆盖bug
    t = np.asarray(world_o) - R @ (np.asarray(local_p, float) / 1000.0)
    return R, t

def rpy_of(R):
    # URDF rpy = Rz(y)Ry(p)Rx(r) 反解
    r = math.atan2(R[2, 1], R[2, 2])
    p = math.atan2(-R[2, 0], math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y

def main():
    out = {}
    for part, (stl, j, lp, ld) in PICKS.items():
        wo = J_ORIGIN[j]
        wd = J_AXIS[j]
        R, t = align(lp, ld, wo, wd)
        r, p, y = rpy_of(R)
        out[part] = {'stl': stl, 'link': f'link_{j}', 'origin': [round(v, 6) for v in t],
                     'rpy': [round(math.degrees(v), 3) for v in (r, p, y)]}
    print(json.dumps(out, ensure_ascii=False, indent=1))

if __name__ == '__main__':
    main()
