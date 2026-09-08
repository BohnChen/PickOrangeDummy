#!/usr/bin/env python3
"""从 SolidWorks 导出的 STEP(AP203) 中提取圆柱面轴线/半径清单。

目的: Dummy 机械臂 CNC 结构件中, 关节轴 = 大直径圆柱孔(轴承座/减速器安装面)的轴线。
本脚本解析每个零件 STEP 的 CYLINDRICAL_SURFACE 实体:
  - 轴线的位置(一点)与方向(单位向量)
  - 半径(mm)
输出按半径聚类的大孔清单, 用于推导相邻关节轴的相对几何(连杆长度/偏距)。

纯离线文件解析, 不涉机械/硬件。
"""
import re
import sys
from collections import defaultdict

ENTITY = re.compile(r'#(\d+)\s*=\s*(.*?)\s*;', re.S)
KIND = re.compile(r'^\s*([A-Z][A-Z0-9_]+)\s*\(')

def load(path):
    text = open(path, encoding='utf-8', errors='ignore').read()
    m = re.search(r'DATA;\n(.*?)\nENDSEC;', text, re.S)
    data = m.group(1)
    entities = {}
    for em in ENTITY.finditer(data):
        eid = int(em.group(1))
        body = em.group(2).strip()
        km = KIND.match(body)
        if km:
            entities[eid] = (km.group(1), body)
    return entities

def parse_vec3(body):
    pts = re.findall(r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*([^)]*)\)", body)
    return pts

def direction_tuple(body):
    ds = re.findall(r"DIRECTION\s*\(\s*'[^']*'\s*\(\s*([^)]*)\)", body)
    return ds

def cartesian_point(body):
    pts = re.findall(r"CARTESIAN_POINT\s*\(\s*'?[^']*'?\s*,\s*\(\s*([^)]*)\)", body)
    return pts

def main(path):
    ents = load(path)
    # 预取: AXIS2_PLACEMENT_3D → (axis_dir #, ref_dir #, origin #)
    axis2 = {}
    for eid, (kind, body) in ents.items():
        if kind == 'AXIS2_PLACEMENT_3D':
            args = re.findall(r'#\d+', body)
            axis2[eid] = args
    cart = {}
    import re as re2
    for eid, (kind, body) in ents.items():
        if kind == 'AXIS2_PLACEMENT_3D':
            args = [int(a[1:]) for a in re2.findall(r'#\d+', body)]
            if len(args) >= 2:
                origin = ents.get(args[-1])   # last is usually origin point
                # 原点可能是最后一个 # 引用 (AXIS2_PLACEMENT_3D(#dir,#ref,#origin) 顺序无保证, 按名字找)
                # 安全: 三个引用依次为 axis, ref, origin
                cart[eid] = args

    cyls = []
    for eid, (kind, body) in ents.items():
        if kind != 'CYLINDRICAL_SURFACE':
            continue
        args = [a for a in re2.findall(r'#\d+', body)]
        rm = re.search(r',\s*([\d.eE+-]+)\s*\)', body)
        if not args or len(args) < 1 or not rm:
            continue
        radius = float(rm.group(1))
        ap = args[-1]
        ap_id = int(ap[1:])
        ap_dat = axis2.get(ap_id)
        if not ap_dat:
            continue
        # axis2 placement: 第1个引用=axis方向, 第2=ref方向, 第3=原点
        # SW/SolidWorks: AXIS2_PLACEMENT_3D(origin, axis_dir, ref_dir)
        orig_eid = int(ap_dat[0][1:]); axis_eid = int(ap_dat[1][1:]) if len(ap_dat) > 1 else 0
        # 原点坐标
        orig_pt = None
        okind, obody = ents.get(orig_eid, (None, ''))
        if okind == 'CARTESIAN_POINT':
            pts = re2.findall(r'\(\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*\)', obody)
            if pts:
                orig_pt = [float(x) for x in pts[0]]
        axis_vec = None
        akind, abody = ents.get(axis_eid, (None, ''))
        if akind == 'DIRECTION':
            d = re2.findall(r'\(\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*\)', abody)
            if d:
                axis_vec = [float(x) for x in d[0]]
        cyls.append({'id': eid, 'r': radius, 'origin': orig_pt, 'axis': axis_vec})

    # 聚类: 半径近似分组(±0.5mm)
    groups = defaultdict(list)
    for c in cyls:
        key = round(c['r'])
        groups[key].append(c)

    print(f"文件: {path.split('/')[-1]}  圆柱面总数: {len(cyls)}  半径聚类: {sorted(groups)}")
    for key in sorted(groups):
        cs = groups[key]
        print(f"\n  r≈{key:.0f}mm ({len(cs)} 个):")
        uniq = set()
        for c in cs:
            if c['origin'] and c['axis']:
                o = tuple(round(v, 3) for v in c['origin'])
                d = tuple(round(v, 3) for v in c['axis'])
                if (o, d) in uniq:
                    continue
                uniq.add((o, d))
                print(f"    轴点({o[0]:9.3f},{o[1]:9.3f},{o[2]:9.3f})mm 方向({d[0]:7.3f},{d[1]:7.3f},{d[2]:7.3f})")
        if not any(c['origin'] for c in cs):
            print("    (无原点信息)")


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[2] == '--json':
        import json
        ents = load(sys.argv[1])
        axis2 = {}
        for eid, (kind, body) in ents.items():
            if kind == 'AXIS2_PLACEMENT_3D':
                axis2[eid] = [int(a[1:]) for a in re.findall(r'#\d+', body)]
        out = []
        for eid, (kind, body) in ents.items():
            if kind != 'CYLINDRICAL_SURFACE':
                continue
            args = [int(a[1:]) for a in re.findall(r'#\d+', body)]
            rm = re.search(r',\s*([\d.eE+-]+)\s*\)', body)
            if not args or len(args) < 1 or not rm:
                continue
            radius = float(rm.group(1))
            ap = args[-1]
            dat = axis2.get(ap)
            if not dat or len(dat) < 2:
                continue
            oid, aid = dat[0], dat[1]
            dep = ents.get(oid, (None, ''))[1]
            m = re.findall(r'\(\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*\)', dep)
            if not m:
                continue
            orig = [float(x) for x in m[0]]
            dd = ents.get(aid, (None, ''))[1]
            m2 = re.findall(r'\(\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*,\s*([\d.eE+-]+)\s*\)', dd)
            if not m2:
                continue
            out.append({'r': radius, 'p': orig, 'dir': [float(x) for x in m2[0]]})
        print(json.dumps(out))
    else:
        main(sys.argv[1])

