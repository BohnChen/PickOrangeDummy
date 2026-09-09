#!/usr/bin/env python3
"""解析 Fusion STEP 装配实例(ITEM_DEFINED_TRANSFORMATION) → 各产品实例的
世界位姿(4x4) 与 shape 引用; 输出按关键件名的实例清单(设计世界, mm/cm? 单位检测)。
纯离线。
"""
import re, json, math

SRC='/home/arduino/Dummy.step'
s=open(SRC,encoding='utf-8',errors='ignore').read()
KEEP_TOKENS = r'[A-Za-z0-9_$\\X2-]+'

def entities():
    ents={}
    for m in re.finditer(r'#(\d+)\s*=\s*([A-Z][A-Z0-9_]*)\s*\((.*?)\)\s*;', s, re.S):
        ents[int(m.group(1))]=(m.group(2), m.group(3))
    return ents

ENTS=entities()
print('实体数', len(ENTS))

# 单位(计算长度转换): 查 LENGTH_MEASURE_WITH_UNIT / unit context
def find_unit():
    for eid,(k,b) in ENTS.items():
        if k=='NAMED_UNIT' and 'MILLI' in b.upper():
            return 1e-3
        if k=='NAMED_UNIT':
            return 1.0
    return 1.0

sc = find_unit()
print('长度单位换算式:', sc*(1000 if sc!=1 else 1),'? (1=mm if sc==1e-3?) 检测沿用 default mm')

def parse_numlist(b):
    vals=[]
    mm=re.findall(r'\(([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\)', b)
    if mm:
        vals=[[float(x) for x in mm[0]]]
    return mm

def cart(eid):
    k,b=ENTS.get(eid,(None,''))
    if k=='CARTESIAN_POINT':
        m=re.findall(r'\(\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*\)', b)
        if m: return [float(x) for x in m[0]]
    return None

def direction(eid):
    k,b=ENTS.get(eid,(None,''))
    if k=='DIRECTION':
        m=re.findall(r'\(\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\)', b)
        if m: return [float(x) for x in m[0]]
    return None

def placements():
    out=[]
    for eid,(k,b) in ENTS.items():
        if k=='ITEM_DEFINED_TRANSFORMATION':
            # ITEM_DEFINED_TRANSFORMATION('',#name,#axis2placement_ref)
            refs=re.findall(r'#\d+', b)
            if len(refs)>=2:
                ap=int(refs[-1][1:])
                out.append((eid,ap))
    return out

def axis2(ap_id):
    k,b=ENTS.get(ap_id,(None,''))
    if k!='AXIS2_PLACEMENT_3D' and not b.startswith('AXIS2_PLACEMENT_3D'):
        if 'AXIS2_PLACEMENT_3D' not in b: return None
    refs=re.findall(r'#\d+', b)
    origin=cart(int(refs[-1][1:])) if refs else None
    # 轴/参考方向: 按 order [origin? 可能是(axis,ref,origin)] 用 DIRECTIONS
    dirs=[]
    for ref in refs:
        d=direction(int(ref[1:]))
        if d: dirs.append(d)
    return origin, dirs

# KEY成对: product 名→(shape_rep, product def) 粗糙法: 找 'PRODUCT_DEFINITION'/'PRODUCT' 与 instance 的关联需完整解析;
# 简化: 解析 occurrence chain 结构: 通常出现 ( ID ( 'name' #prod_def...))
# 直接方案: 找每个 ITEM_DEFINED_TRANSFORMATION 的父子 shape 关系:
#   REPRESENTATION_RELATIONSHIP( 'NONE','NONE', #child, #parent ) 及带 transformation 的变体
# 采集: rel(rep_rel) 带 ITEM_DEFINED_TRANSFORMATION 引用
print('\n扫描 REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION 结构…')
relx=[]
for eid,(k,b) in ENTS.items():
    if 'WITH_TRANSFORMATION' in k or (k=='REPRESENTATION_RELATIONSHIP' and 'ITEM_DEFINED_TRANSFORMATION' in b):
        relx.append((eid,k,b[:120]))
print('带变换的关系数:', len(relx))
for eid,k,b in relx[:6]:
    print(eid,k,'|',b[:150])
json.dump({'ents':len(ENTS),'transform_items':len([1 for k,b in ENTS.values() if k=='ITEM_DEFINED_TRANSFORMATION'])}, open('/tmp/step_diag.json','w'))
