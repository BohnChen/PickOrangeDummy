#!/usr/bin/env python3
"""生成 dummyarm 运动学链 URDF —— 采用图纸真值(改版 CNV 件)。

世界系(零位姿态假设): 基座底面中心 = (0,0,0), X=机械臂前向, Z=上, J1 轴=Z。
真值来源: 改版工程图 (基座55 + J1件37.5 → J2 高 92.5; D_BASE 35; L_ARM 146;
          D_ELBOW 52(图50, 保留52并注释); L_FA 115 (auk); J4/J5/J6 段按 auk 结构+尺寸,
          其中 J5B→J6 差 2~4mm 待用户款件核实 → 参数化)。

注: 与固件 DH(0.109/…)的差异已确认: 固件 L_BASE 用原版值, 图纸物理值 92.5。
    本模型为物理真值优先, KDL 验证只做自洽性, 与真机 REST/HOME 对照会有 ~16mm
    级固有偏差(固件模型近似), 属预期。
"""
import math
import os

OUT = os.path.expanduser("~/PickOrangeDummy/src/dummyarm_description/urdf/dummyarm.xacro")

# ---- 真值参数 (mm) ----
L_BASE = 92.5       # 桌面→J2 轴心 (55 基座 + 37.5 J1 主孔中心)   [待实测确认]
D_BASE = 35.0       # J1 轴→J2 轴 前向偏距 (J1图 35±0.05 == auk D_BASE)
L_ARM = 146.0       # J2→J3 轴距 (J23 图 146)
D_ELBOW = 52.0      # J3 轴沿 J2 轴方向偏距 (J23 图 50±0.05; auk=52; 二选一待精调)
L_FA = 115.0        # J3→J4 轴距 (auk L_FOREARM; 图纸 J4 52/17/8 相关, 待精调)
L_EW = 92.0         # J4→J5 (由 J5B 92 端面尺寸近似)   [待精调]
L_WRIST = 72.0      # J5→J6 (auk L_WRIST; 用户 J5B 款件差异 → 参数化)

REST = (0.0, -73.0, 180.0, 0.0, 0.0, 0.0)

limits_deg = [(-170, 170), (-73, 90), (35, 180), (-180, 180), (-120, 120), (-720, 720)]
reduction = [30, 30, 30, 24, 30, 50]
# 关节轴: J1=Z, J2=Y, J3=Y, J4=X(腕滚), J5=Y, J6=X(末端旋转轴)
# 关节轴 (定案, KDL 自洽验证): J1=Z, J2/J3/J5=-Y(俯仰), J4/J6=+X(腕滚/末端轴)
axes = [(0, 0, 1), (0, -1, 0), (0, -1, 0), (1, 0, 0), (0, -1, 0), (1, 0, 0)]
# 各关节 origin (父 frame, m)
# 腕部结构照搬固件(公共腕点): 腕心=肘+R03·(-D_ELBOW,0,L_FA) → J4 origin 在 link3 系 (-0.052,0,0.115)
# J5/J6 与腕心重合; tool0 = 腕点 + 0.072 沿 J6 轴(+X)
origins = [
    (0.0, 0.0, 0.0),                                  # J1 @桌面
    (D_BASE / 1000, 0.0, L_BASE / 1000),              # J2
    (L_ARM / 1000, 0.0, 0.050),                       # J3 (面内偏 D_ELBOW 见下)
    (-D_ELBOW / 1000, 0.0, L_FA / 1000),              # J4 (腕心, link3 系, 固件 v3 结构)
    (0.0, 0.0, 0.0),                                  # J5 @腕心
    (0.0, 0.0, 0.0),                                  # J6 @腕心 (末端驱动轴)
]
TOOL0 = (L_WRIST / 1000, 0.0, 0.0)

def fmt(x):
    return f"{x:.6f}"

def main():
    out = []
    out.append('<?xml version="1.0" ?>')
    out.append('<!-- dummyarm 运动学链(图纸真值版, 2026-09-08 %s) -->' % os.path.basename(__file__))
    out.append('<!-- 说明: L_BASE=92.5(基座55+J1件37.5); D_BASE=35; L_ARM=146; D_ELBOW=52(图50 二选一); '
               'L_FA=115; J4/J5/J6 段为近似+待调; 固件DH与其差异属预期 -->')
    out.append('<robot name="dummyarm" xmlns:xacro="http://www.ros.org/wiki/xacro">')
    out.append('<xacro:include filename="$(find dummyarm_description)/urdf/materials.xacro" />')
    out.append('<link name="base_link"/>')

    prev = 'base_link'
    for i in range(6):
        ln = f'link_{i+1}'
        out.append(f'<link name="{ln}"/>')
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
    out.append('</robot>')
    open(OUT, 'w').write('\n'.join(out) + '\n')
    print(f'已写入 {OUT}')
    print('参数: L_BASE=%.1f D_BASE=%.1f L_ARM=%.1f D_ELBOW=%.1f L_FA=%.1f L_EW=%.1f L_WRIST=%.1f'
          % (L_BASE, D_BASE, L_ARM, D_ELBOW, L_FA, L_EW, L_WRIST))

if __name__ == '__main__':
    main()
