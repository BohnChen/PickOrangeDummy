#!/usr/bin/env python3
"""把 fusion2urdf 导出(Dummy_ForURDF_description)转换并入 dummyarm_description 单一 xacro。

转换规则:
  joints: _1.._6 → Joint1..Joint6; _6 continuous→revolute
  links : base_link→base_link; J1_2→link_1; J2J3_2→link_2; J4_2→link_3;
          35_20Motor_Assembly_2→link_4; 35_20Motor_Assembly_1_2→link_5; '3'→tool0(末端)
  限位 : 模型约定覆盖 (J1±170 J2 0..163 J3 -180..-35 J4±180 J5±95! J6±720;
         物理值 J2-73..90/J3 35..180 经 TH=[0,-90,0,0,0,0] 映射)
  网格 : 拷贝导出 meshes → dummyarm_description/meshes/export/<link>_visual.dae(+collision stl)
         路径改为 package://dummyarm_description/meshes/export/...
  保留 : 各 link inertial/visual origin/rpy (CAD 真装配)
  加   : 我们自己的 ros2_control 宏调用 (Joint1..6)
纯离线; 生成后仍需 KDL+映射复核(SIGNS/TH 可能变化)。
"""
import math, os, shutil
import xml.etree.ElementTree as ET

EXPT = os.path.expanduser('~/dummyurdf/Dummy_Description')
OUTD = os.path.expanduser('~/PickOrangeDummy/src/dummyarm_description')
MESH_DST = os.path.join(OUTD, 'meshes', 'export')
os.makedirs(MESH_DST, exist_ok=True)

# ---- 拷贝网格 ----
mesh_map = {
  'base_link': ('base_link/base_link.dae', 'base_link/base_link_collision.stl'),
  'J1_2': ('J1/J1_2.dae', 'J1/J1_2_collision.stl'),
  'J2J3_2': ('J2J3/J2J3_2.dae', 'J2J3/J2J3_2_collision.stl'),
  'J4_2': ('J4/J4_2.dae', 'J4/J4_2_collision.stl'),
  '35_20Motor_Assembly_2': ('35_20Motor_Assembly/35_20Motor_Assembly_2.dae', '35_20Motor_Assembly/35_20Motor_Assembly_2_collision.stl'),
  '35_20Motor_Assembly_1_2': ('35_20Motor_Assembly_1/35_20Motor_Assembly_1_2.dae', '35_20Motor_Assembly_1/35_20Motor_Assembly_1_2_collision.stl'),
  '3': ('Dummy_ForURDF/3.dae', 'Dummy_ForURDF/3_collision.stl'),
}
names_out = {
  'base_link': 'base_link', 'J1_2': 'link_1', 'J2J3_2': 'link_2', 'J4_2': 'link_3',
  '35_20Motor_Assembly_2': 'link_4', '35_20Motor_Assembly_1_2': 'link_5', '3': 'tool0',
}
for old, (vis, col) in mesh_map.items():
    new = names_out[old]
    for src, dstname in ((vis, f'{new}_visual.dae'), (col, f'{new}_collision.stl')):
        s = os.path.join(EXPT, 'meshes', src)
        if not os.path.exists(s):
            print('! 缺网格', s); continue
        shutil.copy(s, os.path.join(MESH_DST, dstname))
print('网格拷贝完成:', len(os.listdir(MESH_DST)), '个文件')

# ---- 读取各组件宏的 link 定义 ----
def get_link(asm_file, linkname):
    T = ET.parse(os.path.join(EXPT, 'urdf/assemblies', asm_file)).getroot()
    for l in T.iter('link'):
        if l.get('name') == '${prefix}' + linkname:
            return l
    return None

limits = {   # (lower, upper) rad, 模型约定 (TH=[0,-90,0,0,0,0], 2026-09-10)
  'Joint1': (-2.967059728, 2.967059728),
  'Joint2': (0.0, 2.844887),               # 物理 -73°..90° -> 模型 0°..163°
  'Joint3': (-3.141592654, -0.610865238),  # 物理 35°..180° -> 模型 -180°..-35°
  'Joint4': (-3.141592654, 3.141592654),
  'Joint5': (-1.658062789, 1.658062789),   # ±95° (105°碰结构!)
  'Joint6': (-12.566370614, 12.566370614), # ±720°
}

def fmt(x): return f'{x:.9f}'

# ---- 生成 ----
out = []
out.append('<?xml version="1.0"?>')
out.append('<robot name="dummyarm" xmlns:xacro="http://www.ros.org/wiki/xacro">')
out.append('  <!-- 由 fusion2urdf 导出 Dummy_ForURDF_description 转换并入 (2026-09-09) -->')
out.append('  <!-- 限位=模型约定: J1±170 J2 0..163 J3 -180..-35 J4±180 J5±95(105碰) J6±720 -->')
out.append('  <xacro:include filename="$(find dummyarm_description)/urdf/materials.xacro" />')

# 主文件 mount joints
T = ET.parse(os.path.join(EXPT, 'urdf', 'Dummy_ForURDF.urdf.xacro')).getroot()
joints = []
for j in T.findall('joint'):
    if j.get('type') == 'revolute' or j.get('type') == 'continuous':
        joints.append(j)
assert len(joints) == 6, f'expect 6 joints, got {len(joints)}'
print('mount joints:', [j.get('name') for j in joints])

def xmlstr(e):
    return ET.tostring(e, encoding='unicode')

# link: base_link (来自 base_link 宏)
def add_link(oldname, jname_used_later):
    newname = names_out[oldname]
    l = get_link({'base_link': 'base_link.urdf.xacro', 'J1_2': 'J1.urdf.xacro',
                  'J2J3_2': 'J2J3.urdf.xacro', 'J4_2': 'J4.urdf.xacro',
                  '35_20Motor_Assembly_2': '',
                  '35_20Motor_Assembly_1_2': '',
                  '3': 'Dummy_ForURDF.urdf.xacro'}[oldname], oldname)
    if l is None:
        return oldname, None
    return oldname, l

# 简化：直接重写 —— 先列出次序 base,link1..5,tool0 与它们原始宏文件
macro_link = {
  'base_link': 'base_link.urdf.xacro', 'J1_2': 'J1.urdf.xacro', 'J2J3_2': 'J2J3.urdf.xacro',
  'J4_2': 'J4.urdf.xacro', '3': 'Dummy_ForURDF.urdf.xacro',
  '35_20Motor_Assembly_2': '35_20Motor_Assembly.urdf.xacro',
  '35_20Motor_Assembly_1_2': '35_20Motor_Assembly_1.urdf.xacro',
}
order = ['base_link', 'J1_2', 'J2J3_2', 'J4_2', '35_20Motor_Assembly_2',
         '35_20Motor_Assembly_1_2', '3']
for old in order:
    new = names_out[old]
    l = get_link(macro_link[old], old)
    if l is None:
        print('!! 未找到 link', old); continue
    # 重写名字 & mesh 路径
    for el in l.iter():
        if el.tag == 'link':
            el.set('name', new)
        if el.tag == 'origin':
            # 提取数值并格式化
            pass
    for el in l.iter('visual'):
        m = el.find('geometry/mesh')
        if m is not None:
            old_f = m.get('filename')
            new_f = f'package://dummyarm_description/meshes/export/{new}_visual.dae'
            m.set('filename', new_f)
    for el in l.iter('collision'):
        m = el.find('geometry/mesh')
        if m is not None:
            m.set('filename', f'package://dummyarm_description/meshes/export/{new}_collision.stl')
    out.append(xmlstr(l))

# mount joints (重命名+限位+J6 revolute)
for i, j in enumerate(joints, start=1):
    parent = j.find('parent').get('link')
    child = j.find('child').get('link')
    if parent in names_out: parent = names_out[parent]
    if child in names_out: child = names_out[child]
    o = j.find('origin')
    xyz = o.get('xyz'); rpy = o.get('rpy')
    ax = j.find('axis')
    axis = ax.get('xyz') if ax is not None else '0 0 1'
    lo, hi = limits[f'Joint{i}']
    out.append(f'  <joint name="Joint{i}" type="revolute">')
    out.append(f'    <parent link="{parent}"/>')
    out.append(f'    <child link="{child}"/>')
    out.append(f'    <origin xyz="{xyz}" rpy="{rpy}"/>')
    out.append(f'    <axis xyz="{axis}"/>')
    out.append(f'    <limit lower="{fmt(lo)}" upper="{fmt(hi)}" effort="100" velocity="1.0"/>')
    out.append(f'  </joint>')

out.append('  <xacro:include filename="$(find dummyarm_description)/urdf/dummyarm.ros2_control.xacro" />')
out.append('  <xacro:dummyarm_ros2_control name="dummyarm" '
           'initial_positions_file="$(find dummyarm_description)/config/initial_positions.yaml"/>')
out.append('</robot>')
text = '\n'.join(out) + '\n'
open(os.path.join(OUTD, 'urdf', 'dummyarm.xacro'), 'w').write(text)
print('生成 dummyarm.xacro, 字节:', len(text))
