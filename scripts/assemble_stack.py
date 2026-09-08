#!/usr/bin/env python3
"""刚性叠装求解器: 基座→J1件(J2)→J23(J3)→J4→J5A→J5B→J6
每件: 用她的'上行孔'(与上一件同轴, 点+方向世界已知)定位, 滚转=第二孔方向约束/主轴规则。
输出每件: world R,t (stack 坐标系) → 再转各 link 帧相对视觉 origin。
纯离线。
"""
import json, math, struct, os
import numpy as np

MESHDIR = '/home/arduino/PickOrangeDummy/src/dummyarm_description/meshes'
TOOL = '/home/arduino/PickOrangeDummy/scripts/extract_step_joints.py'
STEP_DIR = ('/home/arduino/Dummy_work/ren20250527/1.Dummy V2任同学整理/4.Model/'
            'Dummy V2 版本 视频款/CNC加工STEP文件')
import subprocess

def axes_of(part):
    out = subprocess.run(['python3', TOOL, f'{STEP_DIR}/{part}.step', '--json'],
                         capture_output=True, text=True)
    return json.loads(out.stdout)

def norm(v):
    v = np.asarray(v, float); n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v

def R_align_dir(u, U):
    u = norm(u); U = norm(U)
    v = np.cross(u, U); s = np.linalg.norm(v); c = np.dot(u, U)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    K = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3) + K + (K @ K) * ((1 - c) / (s * s))

def roll(R, U, ref_local, ref_world):
    U = norm(U); ref_local = norm(ref_local); ref_world = norm(ref_world)
    best = None
    for ang in np.linspace(-math.pi, math.pi, 721):
        K = np.array([[0,-U[2],U[1]],[U[2],0,-U[0]],[-U[1],U[0],0]])
        Rr = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)
        Rt = Rr @ R
        score = np.dot(norm(Rt @ ref_local), ref_world)
        if best is None or score > best[0]:
            best = (score, Rt)
    return best[1]

def pickup(part, radius_find, prefer_dir=None):
    """在件里按半径选集, 取'最大/唯一合适'轴(半径最接近 radius_find), 返回 (point mm, dir)"""
    c = axes_of(part)
    cand = [x for x in c if abs(x['r'] - radius_find) < 0.5]
    if not cand:
        return None, None
    # 若 prefer_dir 指定, 从候选中选方向接近的
    if prefer_dir is not None:
        cand.sort(key=lambda x: abs(np.dot(norm(x['dir']), norm(prefer_dir))), reverse=True)
    cand.sort(key=lambda x: x['r'], reverse=True)
    x = cand[0]
    return np.array(x['p'], float), norm(x['dir'])

# ---------- 求解 ----------
# (件名, 上行孔半径(找), 与上行孔连接的轴世界方向(由上行孔已定, SAME), 滚转约束(ref_local, ref_world), 上行孔世界点方向提示)
# 基座: 世界 J1 轴竖直过原点, 顶面 Φ40.6 中心=原点平面
WORLD = {}
WORLD['base'] = (np.eye(3), np.zeros(3))

# 1. 基座件自身: 内部 Φ40.6 顶面孔中心 = 局部? 基座顶孔 (20.3) —
# 用基座大孔 Φ40.6 锚: 找到其竖直轴
p_base, d_base = pickup('基座', 20.3)
# 基座定位: 自身竖直轴→世界Z@(0,0,0)
R0 = R_align_dir(d_base, [0, 0, 1])
# 滚转无约束→保持默认
t0 = np.zeros(3) - R0 @ (p_base / 1000.0)
WORLD['base2'] = (R0, t0)  # 视觉世界姿(相对 world 原点)

# 后续装配世界的"基座顶孔轴"已就位: J1 世界轴线方向 = R0·(z)? = world Z; 顶孔中心世界 = R0·(0,0,?)+t0…

# 2. J1件: 上行孔 = 底部竖直孔(Φ40.6 槽)接基座: 找 J1 竖直大孔 r≈20.3
#    J1 件有竖直孔? step里J1竖直轴少; 用其J2轴Φ11(r5.525)直接作为'世界J2轴'锚(原点在(0.035,0,0.0925), 方向-Y)
#    但栈叠应在J1件竖直槽台上 —— 从装配讲: J1件坐着基座顶, 其J2孔轴线过 (0.035,0,0.0925)(由图纸, 已定)
J2w = np.array([0.035, 0, 0.0925]); D2_w = np.array([0, -1, 0])
pJ1j2, dJ1j2 = pickup('J1', 5.525)
R1 = R_align_dir(dJ1j2, D2_w)
# 滚转: J1件 竖直方向(基座对应对齐世界Z): ref_local=up? 用其底部孔竖直 dir? J1 无竖直大孔, 以其'向上块'Z为主轴: 用bbox长轴
lo, hi = [0,0,0], [0,0,0]  # 跳过, 直接: 用第二孔方向约束: J1件还有减速器盘孔(Φ54?r27/31/32) dir与J2轴同X? → 用盘面法向=J2轴⊥平面=Y_local? 取1:55m多孔排(Φ18簇 dir X)与J2同向
# 简化: 用J1件 bbox 长轴方向(D-Boss盘向?) — 取局部最高vertext? 我们用: 件'上级'方向=J2在件里的副向(r44?dir?)…
# 实际: J1件在 J2轴上有盘(Φ54)其法向+/-X=J2轴(X), 竖直=件底面法向=世界Z? 件底面法向locally? — 用近似: roll使局部z→世界z
R1 = roll(R1, D2_w, [0, 0, 1], [0, 0, 1])
t1 = J2w - R1 @ (pJ1j2 / 1000.0)
WORLD['1'] = (R1, t1)

# 3. J23: J2孔(Φ13) 对接 J1件的 J2世界轴(点J2w方向-Y)
p23a, d23a = pickup('J23', 6.5)
R2 = R_align_dir(d23a, D2_w)
# 滚转: J23 第二孔 J3(Φ13)方向 也要→ J3世界轴−Y? 同向(平行)→主轴规则: J23 bbox 长轴→世界(J3-J2方向矢量)
# 用 bbox 长轴(L_ARM方向 local: J2J3件长轴=local Y(205mm)); 世界目标=(J3w-J2w)=(0.146,0,0.050)方向
J3w = J2w + np.array([0.146, 0, 0.050])
segv = norm(J3w - J2w)
# 局部长轴方向: 取 J23 两个孔差: Δ23 = (p23b-p23a)/|..| = 世界方向 j3w-j2w
p23b, d23b = pickup('J23', 6.5, prefer_dir=None)
# pickup 可能返回同一孔, 需要另一个: 直接二选一: 用聚类
c = axes_of('J23'); holes = [x for x in c if abs(x['r']-6.5) < 0.4]
A = np.array(holes[0]['p']); B = np.array(holes[1]['p'])
if np.linalg.norm(A - p23a) < 1e-9: p23b = B
else: p23b = A; p23a = A if A is p23a else B
delv = norm(p23b - p23a)
R2 = roll(R2, D2_w, delv, segv)
t2 = J2w - R2 @ (p23a / 1000.0)
WORLD['2'] = (R2, t2)

# 4. J4件: J3 法兰盘(Φ40.1) 对接 J23 的 J3孔世界轴(点=p23b变换后, 方向=R2·d23b(≈? 应为世界 −(J3轴?)) )
J3_point_w = R2 @ (p23b / 1000.0) + t2
J3_dir_w = norm(R2 @ norm(d23b))
p4a, d4a = pickup('J4', 20.05)   # Φ40.1
R3 = R_align_dir(d4a, J3_dir_w)
# 滚转: J4件的 J4轴(其大副孔r?取(r在25~30 cluster, 方向与J3轴不同)) 世界→J4轴=+X
# J4件第二大孔: r 约? (29/30): 
c4 = axes_of('J4'); sec = [x for x in c4 if 24 < x['r'] < 31]
if sec:
    sec.sort(key=lambda x: -x['r']); s4 = sec[0]
    R3 = roll(R3, J3_dir_w, s4['dir'], [1, 0, 0])
t3 = J3_point_w - R3 @ (p4a / 1000.0)
WORLD['3'] = (R3, t3)

# 5. J5A: 轴颈 Φ11.95 (dir -Y local) 插 J4件减速器输出(对应world dir= R3·(s4 dir?)) = J4轴世界方向
J4_point_w = R3 @ (np.array(s4['p']) / 1000.0) + t3
J4_dirw = norm(R3 @ norm(s4['dir']))
p5a, d5a = pickup('J5A', 5.975)
R4 = R_align_dir(d5a, J4_dirw)
# 滚转: J5A 端面D盘法向/它的上级轴颈另一孔(Φ? 13.5→)方向→ 世界 J5轴 -Y
c5a = axes_of('J5A'); sec5 = [x for x in c5a if abs(x['r'] - 13.5) < 0.5]
if sec5:
    R4 = roll(R4, J4_dirw, sec5[0]['dir'], [0, -1, 0])
t4 = J4_point_w - R4 @ (p5a / 1000.0)
WORLD['4'] = (R4, t4)

# 6. J5B: 小孔(Φ? r5.0?) 插 J5A轴颈: J5B 上行孔 = 其r≈5.0孔(与J5A轴颈同轴)
p6a, d6a = pickup('J5B', 5.0)
J5A_p_w = R4 @ (p5a / 1000.0) + t4
J5A_dw = norm(R4 @ norm(d5a))
R5 = R_align_dir(d6a, J5A_dw)
# 滚转: J5B 大孔 Φ31.05 (其 dir) → 世界J6轴 +X
c6 = axes_of('J5B'); big = [x for x in c6 if abs(x['r'] - 15.525) < 0.6]
if big:
    R5 = roll(R5, J5A_dw, big[0]['dir'], [1, 0, 0])
t5 = J5A_p_w - R5 @ (p6a / 1000.0)
WORLD['5'] = (R5, t5)

# 7. J6: Φ26.1 孔插入 J5B 大孔
p7a, d7a = pickup('J6', 13.05)
bigw = R5 @ (np.array(big[0]['p']) / 1000.0) + t5 if big else J5A_p_w
bigd = norm(R5 @ norm(big[0]['dir'])) if big else J5A_dw
R6 = R_align_dir(d7a, bigd)
# 滚转: J6 主轴 Φ8.95 Δ垂直分量→世界 +Z(执行器朝上/外):利用其Δ
c7 = axes_of('J6'); sp = [x for x in c7 if abs(x['r'] - 4.475) < 0.3]
spd = norm(sp[0]['dir']) if sp else None
if spd is not None:
    # Δ = sp_p - p26_p 的垂直分量 目标=世界(0,0,±1)? 用'轴外'方向: 垂直于 bigd
    D = norm(bigd)
    dvec = norm(np.array(sp[0]['p'], float) - np.array(p7a, float))
    perp = dvec - np.dot(dvec, spd) * spd
    R6 = roll(R6, D, perp, [0, 0, 1])
t6 = bigw - R6 @ (p7a / 1000.0)
WORLD['6'] = (R6, t6)

print('stack WORLD poses:')
for k, (R, t) in sorted(WORLD.items()):
    print(f'  {k}: t={(t*1000).round(1)} R0={(R[0]).round(2)}')
json.dump({k: [v[0].tolist(), v[1].tolist()] for k, v in WORLD.items()}, open('/tmp/stack_world.json', 'w'))
