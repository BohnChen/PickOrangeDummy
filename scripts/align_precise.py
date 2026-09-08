#!/usr/bin/env python3
"""逐件精确视觉配准(确定性): 用 STEP 实测轴孔方向+链世界轴方向 解唯一 R。

每件: 上游轴孔 (p_up, d_up) → 世界链 Joint k (P_up_w, D_up_w)
      下游轴孔 (p_dn, d_dn) → 世界 Joint k+1 的轴方向 D_dn_w (位置仅提供Δ约束)
R = Kabsch(两个方向对) → 唯一; 若方向对平行(如J6, 双轴平行) 则回退: 主轴贴合 + Δ垂直分量朝世界Z。

输出: 各件 (link, xyz_m, rpy_rad)。纯离线。
"""
import json, math
import numpy as np

# ---- 链世界(零位) ----
J2w  = np.array([0.035, 0, 0.0925]);  D2w = np.array([0, -1, 0])
J3w  = np.array([0.181, 0, 0.1425]);  D3w = np.array([0, -1, 0])
J4w  = np.array([0.129, 0, 0.2575]);  D4w = np.array([1, 0, 0])
J5w  = np.array([0.129, 0, 0.2575]);  D5w = np.array([0, -1, 0])
J6w  = np.array([0.129, 0, 0.2575]);  D6w = np.array([1, 0, 0])

def norm(v):
    v = np.array(v, float); n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v

def kabsch_dirs(pairs):
    """pairs: [(d_local, d_world)]; 求 R 使 R@d_local ≈ d_world (方向, 加权平均→正交化)"""
    A = np.zeros((3, 3))
    for d, w in pairs:
        A += np.outer(norm(d), norm(w))
    U, _, Vt = np.linalg.svd(A)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1
    return R

def roll_align(R, U, local_ref, world_ref):
    """绕 U 滚转, 使 R·local_ref 与 world_ref 同向 (对平行双孔时的回退)"""
    U = norm(U)
    def rotU(Rm, ang):
        K = np.array([[0, -U[2], U[1]], [U[2], 0, -U[0]], [-U[1], U[0], 0]])
        return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)
    best = None
    for ang in np.linspace(-math.pi, math.pi, 721):
        Rt = rotU(np.eye(3), ang) @ R
        # 部件Δ(垂直分量) 映射到 world_ref 方向投影
        dq = norm(local_ref - np.dot(local_ref, norm(U_guess(local_ref[0] if False else local_ref))) * 0)  # noqa
        score = np.dot(norm(Rt @ local_ref), norm(world_ref))
        if best is None or score > best[0]:
            best = (score, Rt)
    return best[1]

def U_guess(x):
    return x

def rpy_of(R):
    r = math.atan2(R[2, 1], R[2, 2]); p = math.atan2(-R[2, 0], math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)); y = math.atan2(R[1, 0], R[0, 0])
    return r, p, y

# 配准表: 每件 = (link, 上游轴孔(local p up + dir), 上游世界(P,D), 下游轴孔(local dir, 世界dir), Δ方向回退候选(ref_part, ref_world))
TASKS = [
    # J1(件): 自身=J2轴孔 Φ11.05 ≈ (-35,10.25,0) dirX → Joint2(-Y)@J2w; 下游主向: J1件垂直安装孔→世界+Z(z向上)
    dict(link='link_1', pup=(-35.0, 10.25, 0.0), dup=(1, 0, 0), wup=(J2w, D2w),
         ddn=np.array([0, 0, 1.0]), wdn=np.array([0, 0, 1.0])),
    # J23: J2Φ13(7,0,0)dirX→Joint2; J3Φ13(-37,146,0)dirX→Joint3方向(-Y=世界X? link2 内下游轴仍−Y世界)
    dict(link='link_2', pup=(7.0, 0.0, 0.0), dup=(-1, 0, 0), wup=(J2w, D2w),
         ddn=(-1, 0, 0), wdn=np.array([0, -1, 0])),
    # J4(件): Φ40.1(-35,156.25,0)dirX→Joint3(-Y)@J3w; 下游: J4自身内部竖直孔→世界+Z
    dict(link='link_3', pup=(-35.0, 156.25, 0.0), dup=(1, 0, 0), wup=(J6w, D6w),
         ddn=np.array([0, 0, 1.0]), wdn=np.array([0, 0, 1.0])),
    # J5A: Φ11.95(-6.9,179.5,34.5)dir-Y→Joint5(-Y)@J5w; 下游孔(±Z)→世界+Z
    dict(link='link_4', pup=(-6.9, 179.5, 34.5), dup=(0, -1, 0), wup=(J5w, D5w),
         ddn=np.array([0, 0, 1.0]), wdn=np.array([0, 1, 0])),
    # J5B: Φ31.05(-19.6,198,131.6) dirX→(减速器轴= 世界+X? J6侧) 应用J6轴D6w; 上游=J5轴? J5B 自身装 J6减速器:
    dict(link='link_5', pup=(-19.6, 198.0, 131.6), dup=(1, 0, 0), wup=(J6w, D6w),
         ddn=np.array([0, 0, 1.0]), wdn=np.array([0, 0, 1.0])),
    # J6: Φ8.95(-35.5,198,107)dirX→Joint6(+X)@J6w; 平行孔→回退: Δ 垂直分量→世界+Z
    dict(link='link_6', pup=(-35.5, 198.0, 107.0), dup=(-1, 0, 0), wup=(J6w, D6w),
         ddn=np.array([0, 0, 1.0]), wdn=np.array([0, 0, 1.0])),
]


# 链零位各 link 世界原点
PL = {'link_1': np.array([0,0,0]),
      'link_2': np.array([0.035, 0, 0.0925]),
      'link_3': np.array([0.181, 0, 0.1425]),
      'link_4': np.array([0.129, 0, 0.2575]),
      'link_5': np.array([0.129, 0, 0.2575]),
      'link_6': np.array([0.129, 0, 0.2575])}

def solve_task(t):
    pup = np.array(t['pup'], float) / 1000.0
    dup = np.array(t['dup'], float)
    Pup, Dup = t['wup'][0], norm(t['wup'][1])
    pairs = [(dup, Dup)]
    perp = np.array(t['ddn'], float)
    # 用下游方向作为第二方向(若与 dup 不平行)
    d2 = norm(perp)
    if abs(np.dot(d2, norm(dup))) < 0.999:
        R = kabsch_dirs(pairs + [(t['ddn'], norm(t['wdn']))])
    else:
        # 平行: 用主轴贴合规则回退: dup→Dup 最小旋转 + 主轴(bbox长轴=单位z?)—这里用Δ方向回退
        R = kabsch_dirs(pairs)
        R = roll_align(R, Dup, np.array(t['ddn'], float), norm(t['wdn']))
    pl = PL[t['link']]
    t_xyz = (Pup - pl) - R @ pup     # 相对 link 帧
    err = np.linalg.norm(norm(R @ norm(dup)) - norm(Dup))
    return R, t_xyz, err

out = {}
for t in TASKS:
    R, tb, err = solve_task(t)
    r, p, y = rpy_of(R)
    out[t['link']] = dict(xyz=[round(float(x), 6) for x in tb],
                          rpy=[round(r, 4), round(p, 4), round(y, 4)],
                          dir_err=round(float(err), 4))
print(json.dumps(out, ensure_ascii=False, indent=1))
json.dump(out, open('/tmp/vis_precise.json', 'w'))
