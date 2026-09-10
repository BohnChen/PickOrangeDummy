#!/usr/bin/env python3
"""生成 dummyarm 运动学链 URDF —— 采用图纸真值(改版 CNV 件)。

世界系(零位姿态假设): 基座底面中心 = (0,0,0), X=机械臂前向, Z=上, J1 轴=Z。
真值来源: 改版工程图 + auk DH/固件 DOFKinematic 交叉核对 (2026-09-10 修正):
          L_BASE=92.5(基座55 + J1件37.5); D_BASE=35; L_ARM=146(J2-J3 轴距);
          D_EW=52(J3-J4 轴距); L_FA=117(肘-腕沿 J4 轴); L_WRIST=72(固件 75.5 近似)。
          注: 2026-09-10 修正此前把 D_ELBOW/L_FA 角色放反的错误
          (Joint3 曾带 0.05 面内偏距; Joint4 曾为 (-0.052,0,0.115))。

注: 与固件 DH(0.109/…)的差异已确认: 固件 L_BASE 用原版值, 图纸物理值 92.5。
    本模型为物理真值优先, KDL 验证只做自洽性, 与真机 REST/HOME 对照会有 ~16mm
    级固有偏差(固件模型近似), 属预期。
"""
import math
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "src/dummyarm_description/urdf/dummyarm.xacro")

# ---- 真值参数 (mm) ----
L_BASE = 92.5       # 桌面→J2 轴心 (55 基座 + 37.5 J1 主孔中心)   [待实测确认]
D_BASE = 35.0       # J1 轴→J2 轴 前向偏距 (J1图 35±0.05 == auk D_BASE)
L_ARM = 146.0       # J2→J3 轴距 (J23 图 146; auk L_AM=0.146)
D_EW = 52.0         # J3→J4 轴距 (auk D_EW=0.052; 2026-09-10 修正角色)
L_FA = 117.0        # 肘→腕沿 J4 轴 (auk L_FOREARM=0.117)
L_WRIST = 72.0      # J5→J6 (auk L_WRIST=75.5; 用户 J5B 款件差异 → 参数化)

REST = (0.0, -73.0, 180.0, 0.0, 0.0, 0.0)   # 物理(固件)休憩位; 模型值=(0,163,-180,0,0,0)

# 限位为模型约定 (q_model = SIGNS·(q_fw+TH_HOME), TH=[0,-90,0,0,0,0]):
# 物理 J2[-73,90] -> 模型 [0,163]; J3[35,180] -> [-180,-35]; 其余同物理
limits_deg = [(-170, 170), (0, 163), (-180, -35), (-180, 180), (-95, 95), (-720, 720)]  # J5 ±95(105°碰结构)
reduction = [30, 30, 30, 24, 30, 50]
# 关节轴: J1=Z, J2=Y, J3=Y, J4=X(腕滚), J5=Y, J6=X(末端旋转轴)
# 关节轴 (定案, KDL 自洽验证): J1=Z, J2/J3/J5=-Y(俯仰), J4/J6=+X(腕滚/末端轴)
axes = [(0, 0, 1), (0, -1, 0), (0, -1, 0), (1, 0, 0), (0, -1, 0), (1, 0, 0)]
# 各关节 origin (父 frame, m)
# 腕部结构照搬固件(公共腕点): 腕心=肘+R03·(L_FA,0,D_EW) → J4 origin 在 link3 系 (0.117,0,0.052)
# J5/J6 与腕心重合; tool0 = 腕点 + 0.072 沿 J6 轴(+X)
origins = [
    (0.0, 0.0, 0.0),                                  # J1 @桌面
    (D_BASE / 1000, 0.0, L_BASE / 1000),              # J2
    (L_ARM / 1000, 0.0, 0.0),                         # J3 (J2→J3 轴距)
    (L_FA / 1000, 0.0, D_EW / 1000),                  # J4 (腕心, link3 系, 固件 DH)
    (0.0, 0.0, 0.0),                                  # J5 @腕心
    (0.0, 0.0, 0.0),                                  # J6 @腕心 (末端驱动轴)
]
TOOL0 = (L_WRIST / 1000, 0.0, 0.0)

def fmt(x):
    return f"{x:.6f}"

def main():
    import struct, os
    # ---- 每件: 视觉姿态(首版启发式: PCA主轴对齐链段方向 + 段中点, 基座贴地) ----
    MESHDIR = os.path.join(REPO, "src/dummyarm_description/meshes")
    def stl_bbox(name):
        vs = []
        with open(os.path.join(MESHDIR, name), 'rb') as f:
            data = f.read()
        off = 84
        while off + 50 <= len(data):
            d = struct.unpack_from('<12fH', data, off)
            vs += [d[3:6], d[6:9], d[9:12]]
            off += 50
        import numpy as np
        V = np.array(vs)
        return V.min(0), V.max(0)
    def pca_axes(name):
        import numpy as np
        lo, hi = stl_bbox(name)
        dims = hi - lo
        order = np.argsort(dims)[::-1]     # 长→短
        return lo, hi, dims, order
    import math as _m
    def rot_rpy_from_axes(axes_w, axes_part):
        # axes_w: 世界目标(列), axes_part: 部件主方向(列); 求R使 R·e_i → axes_w[i] (标签按排序对齐)
        import numpy as np
        M = np.column_stack(axes_w); P = np.column_stack(axes_part)
        R = M @ P.T
        def rpy_of(R):
            r = _m.atan2(R[2, 1], R[2, 2]); p = _m.atan2(-R[2, 0], _m.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)); y = _m.atan2(R[1, 0], R[0, 0])
            return r, p, y
        return rpy_of(R)
    # 段: (中点, 方向) 世界(零位) — 来自链
    import numpy as _np
    J2 = _np.array([0.035, 0, 0.0925]); J3 = _np.array([0.181, 0, 0.0925])
    J4 = _np.array([0.298, 0, 0.1445]); Wrist = _np.array([0.298, 0, 0.1445])
    Tool = _np.array([0.370, 0, 0.1445])
    segs = {
        1: (( _np.array([0,0,0]) + J2)/2,  (J2-_np.array([0,0,0]))/0.1),
        2: ((J2 + J3)/2,                  (J3-J2)),
        3: ((J3 + J4)/2,                  (J4-J3)),
        4: (J4,                           (_np.array([1,0,0]))),
        5: (Wrist,                        (_np.array([1,0,0]))),
        6: ((Wrist + Tool)/2,             (Tool-Wrist)),
    }
    VIS_POSE = {
      1: (0.04525, -0.035, 0.0925, 3.1416, -0.0, -1.5708),
      2: (0.0, -0.007, 0.0, 3.1416, -0.0, 1.5708),
      3: (-0.017, -0.15625, 0.165, 0.0, -0.0, 0.0),
      4: (-0.1621, -0.151321, 0.21553, -0.7854, -0.0, 0.0),
      5: (-0.1494, -0.198, -0.0186, 0.0, -0.0, 0.0),
      6: (-0.2045, -0.198, 0.22, 3.1416, -0.0, 3.1416),
    }
    out = []
    out.append('<?xml version="1.0" ?>')
    out.append('<!-- dummyarm 运动学链(图纸真值版, 2026-09-08 generate_true_urdf.py; 2026-09-10 按 DH 修正 J3/J4) -->')
    out.append('<!-- 说明: L_BASE=92.5(基座55+J1件37.5); D_BASE=35; L_ARM=146(J2-J3 轴距); D_EW=52(J3-J4 轴距); '
               'L_FA=117(肘-腕沿轴); 与 auk DH/固件 DOFKinematic 一致; 限位=模型约定(TH=[0,-90,0,0,0,0]); 固件 L_BASE 用原版值(近似)差异属预期 -->')
    out.append('<robot name="dummyarm" xmlns:xacro="http://www.ros.org/wiki/xacro">')
    out.append('<xacro:include filename="$(find dummyarm_description)/urdf/materials.xacro" />')
    out.append('<link name="base_link"><visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="package://dummyarm_description/meshes/arm_base.stl" scale="0.001 0.001 0.001"/></geometry><material name="silver"/></visual></link>')

    prev = 'base_link'
    for i in range(6):
        ln = f'link_{i+1}'
        stl = {1:'arm_J1.stl',2:'arm_J23.stl',3:'arm_J4.stl',4:'arm_J5A.stl',5:'arm_J5B.stl',6:'arm_J6.stl'}[i+1]
        ox, oy, oz, rx, py, yw = VIS_POSE[i+1]
        out.append(f'<link name="{ln}">')
        out.append('  <visual>')
        out.append('    <origin xyz="%s %s %s" rpy="%s %s %s"/>' % (fmt(ox), fmt(oy), fmt(oz), rx, py, yw))
        out.append('    <geometry><mesh filename="package://dummyarm_description/meshes/%s" scale="0.001 0.001 0.001"/></geometry>' % stl)
        out.append('    <material name="silver"/>')
        out.append('  </visual>')
        out.append(f'  <collision><geometry><mesh filename="package://dummyarm_description/meshes/%s" scale="0.001 0.001 0.001"/></geometry></collision>' % stl)
        out.append('</link>')
        lo, hi = limits_deg[i]
        x, y, z = origins[i]
        ax = ' '.join(fmt(a) for a in axes[i])
        out.append(f'<joint name="Joint{i+1}" type="revolute">')
        out.append(f'  <parent link="{prev}"/>')
        out.append(f'  <child link="{ln}"/>')
        out.append(f'  <origin xyz="{fmt(x)} {fmt(y)} {fmt(z)}" rpy="0 0 0"/>')
        out.append(f'  <axis xyz="{ax}"/>')
        out.append(f'  <limit lower="{fmt(lo*math.pi/180)}" upper="{fmt(hi*math.pi/180)}" '
                   f'effort="150" velocity="3.15"/>')
        out.append('</joint>')
        prev = ln

    out.append('<link name="tool0"/>')
    out.append('<joint name="tool0_joint" type="fixed">')
    out.append(f'  <parent link="{prev}"/>')
    out.append('  <child link="tool0"/>')
    out.append('  <origin xyz="%s %s %s" rpy="0 0 0"/>' % (fmt(TOOL0[0]), fmt(TOOL0[1]), fmt(TOOL0[2])))
    out.append('</joint>')
    out.append('  <xacro:include filename="$(find dummyarm_description)/urdf/dummyarm.ros2_control.xacro" />')
    out.append('  <xacro:dummyarm_ros2_control name="dummyarm" '
               'initial_positions_file="$(find dummyarm_description)/config/initial_positions.yaml"/>')
    out.append('</robot>')
    open(OUT, 'w').write('\n'.join(out) + '\n')
    print(f'已写入 {OUT}')
    print('参数: L_BASE=%.1f D_BASE=%.1f L_ARM=%.1f D_EW=%.1f L_FA=%.1f L_WRIST=%.1f'
          % (L_BASE, D_BASE, L_ARM, D_EW, L_FA, L_WRIST))

if __name__ == '__main__':
    main()
