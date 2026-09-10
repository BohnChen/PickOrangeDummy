#!/usr/bin/env python3
"""从固件 DH 真值重建 dummyarm URDF 运动学链(离线、无机械风险)。

设计(对应固件 6dof_kinematic.cpp, 已真机数值验证):
  每个关节 k: R_k = Rz(θh_k + q) * Rx(α_k)
  URDF 表达: revolute_k(轴z, rpy=(0,0,θh_k)) + fixed_k(rpy=(α_k,0,0), origin=t_k)
  t_k = Rx(α_k) * v_k   (v_k 为固件子坐标系连杆矢量)
  v1=(D_BASE,-L_BASE,0) v2=(L_ARM,0,0) v3=(-D_ELBOW,0,L_FOREARM) v6=(0,0,L_WRIST)
  网格(全局装配坐标 STL)锚定: 参考姿态=真机 REST, 视觉原点 O = -R_k(REST)^-1 · P_k(REST)
  使 REST 下网格叠加回装配像(RViz 中拼装完整)。精确位姿待 Fusion URDF 导出后精修。
"""
import math, os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PARAM = dict(L_BASE=0.109, D_BASE=0.035, L_ARM=0.146, D_ELBOW=0.052,
             L_FOREARM=0.115, L_WRIST=0.072)
D2R = math.pi / 180.0

def rx(m):
    a = m * D2R
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]

def rz(m):
    a = m * D2R
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]

def vadd(a, b):
    return [a[i] + b[i] for i in range(3)]

P = PARAM
THETA_HOME = [0, -90, 90, 0, 0, 0]
ALPHA = [-90, 0, 90, -90, 90, 0]
V = [(P['D_BASE'], -P['L_BASE'], 0), (P['L_ARM'], 0, 0),
     (-P['D_ELBOW'], 0, P['L_FOREARM']), (0, 0, 0), (0, 0, 0),
     (0, 0, P['L_WRIST'])]

def chain(q_deg):
    """返回 {frame: (R3x3, P3)}; frames: base, g1..g6(pre), f1..f6(post)"""
    R = {'base': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    P = {'base': [0.0, 0.0, 0.0]}
    prev = 'base'
    for k in range(6):
        g = f'g{k+1}'; f = f'f{k+1}'
        # revolute: 轴 z, rpy=(0,0,θh) → Rz(θh+q)
        Rg = mul(R[prev], mul(rz(THETA_HOME[k]), rz(q_deg[k])))
        R[g] = Rg
        P[g] = P[prev][:]
        # fixed: rpy=(α,0,0) → Rx(α); origin t = Rx(α)·v_k
        t = mv(rx(ALPHA[k]), V[k])
        R[f] = mul(Rg, rx(ALPHA[k]))
        P[f] = vadd(P[g], mv(Rg, t))
        prev = f
    return R, P

def inv_rot(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]

def fmt(x):
    return f"{x:.9f}"

def main():
    REST = [0, -73, 180, 0, 0, 0]
    R, P = chain(REST)

    out = []
    out.append('<?xml version="1.0" ?>')
    out.append('<!-- dummyarm: 运动学链由固件 auk DH 真值重建(离线生成); '
               '网格锚定参考位=真机 REST。精确几何待 Fusion 导出精修 -->')
    out.append('<robot name="dummyarm" xmlns:xacro="http://www.ros.org/wiki/xacro">')
    out.append('<xacro:include filename="$(find dummyarm_description)/urdf/materials.xacro" />')
    out.append('<link name="base_link"/>')

    mesh_of = ['base_link', 'link1_1_1', 'link2_1_1', 'link3_1_1',
               'link4_1_1', 'link5_1_1', 'link6_1_1']
    # part k (1..6) 附着到 g_k (随 joint k 转动), 视觉原点 = -R_gk(REST)^-1 · P_gk(REST)
    anchor = {}
    for k in range(1, 7):
        g = f'g{k}'
        Rg, Pg = R[g], P[g]
        O = mv(inv_rot(Rg), [-x for x in Pg])
        anchor[k] = O
    # base 部件直接贴 base_link(原点 0,0,0; 全局坐标假设 CAD 原点=底座)
    anchor[0] = [0.0, 0.0, 0.0]

    for k in range(6):
        g = f'g{k+1}'; f = f'f{k+1}'
        parent = 'base_link' if k == 0 else f'f{k}'
        th = THETA_HOME[k]
        # link 声明(带视觉)
        mesh = mesh_of[k + 1]
        Ox, Oy, Oz = anchor[k + 1]
        out.append(f'<link name="{f}"/>')
        out.append(f'<link name="{g}">')
        out.append('  <visual>')
        out.append(f'    <origin xyz="{fmt(Ox)} {fmt(Oy)} {fmt(Oz)}" rpy="0 0 0"/>')
        out.append('    <geometry><mesh filename="package://dummyarm_description/meshes/'
                   f'{mesh}.stl" scale="0.001 0.001 0.001"/></geometry>')
        out.append('    <material name="silver"/>')
        out.append('  </visual>')
        out.append(f'  <collision><geometry><mesh filename="package://dummyarm_description/meshes/'
                   f'{mesh}.stl" scale="0.001 0.001 0.001"/></geometry></collision>')
        out.append('</link>')
        out.append(f'<joint name="Joint{k+1}" type="revolute">')
        out.append(f'  <parent link="{parent}"/>')
        out.append(f'  <child link="{g}"/>')
        out.append(f'  <origin xyz="0 0 0" rpy="0 0 {th}"/>')
        out.append('  <axis xyz="0 0 1"/>')
        limits = [(-2.9671, 2.9671), (-1.2741, 1.5708), (0.6109, 3.1416),
                  (-3.1416, 3.1416), (-2.0944, 2.0944), (-12.566, 12.566)]
        lo, hi = limits[k]
        out.append(f'  <limit lower="{lo}" upper="{hi}" effort="150" velocity="3.15"/>')
        out.append('</joint>')
        t = mv(rx(ALPHA[k]), V[k])
        a = ALPHA[k]
        out.append(f'<joint name="twist_{k+1}" type="fixed">')
        out.append(f'  <parent link="{g}"/>')
        out.append(f'  <child link="{f}"/>')
        out.append(f'  <origin xyz="{fmt(t[0])} {fmt(t[1])} {fmt(t[2])}" rpy="{a} 0 0"/>')
        out.append('</joint>')

    out.append('<link name="tool0"/>')
    out.append('<joint name="tool0_joint" type="fixed">')
    out.append('  <parent link="f6"/>')
    out.append('  <child link="tool0"/>')
    out.append('  <origin xyz="0 0 0" rpy="0 0 0"/>')
    out.append('</joint>')
    out.append('</robot>')
    open(os.path.join(REPO, 'src/dummyarm_description/urdf/dummyarm.xacro'), 'w').write('\n'.join(out) + '\n')
    print('已生成 dummyarm.xacro')
    # 打印 REST 锚点核对
    for k in range(1, 7):
        print(f"  part{k} anchor=({anchor[k][0]*1000:8.2f},{anchor[k][1]*1000:8.2f},{anchor[k][2]*1000:8.2f}) mm")

if __name__ == '__main__':
    main()
