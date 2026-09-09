#!/usr/bin/env python3
"""STEP compound+Location 遍历: 每个带Location的复合=一个装配实例组。
输出: 分组(名字可由后续尺寸匹配) 世界bbox(经Location). 用 ~/occvenv/bin/python 运行。
"""
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_ShapeEnum
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound, TopoDS_Iterator
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopLoc import TopLoc_Location
import json

def bbox_of(shape):
    bb = Bnd_Box()
    BRepBndLib.Add_s(shape, bb)
    return [bb.CornerMin().X(), bb.CornerMin().Y(), bb.CornerMin().Z(),
            bb.CornerMax().X(), bb.CornerMax().Y(), bb.CornerMax().Z()]

r = STEPControl_Reader()
st = r.ReadFile('/home/arduino/Dummy.step')
print('read', st)
r.TransferRoots()
print('roots', r.NbShapes())
root = r.Shape(1)

groups = []  # each: {bbox, solid_count, solids:[bbox...]}
def is_solid(shape):
    ex = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_SOLID)
    return ex.More()

def walk_instances(shape, loc=TopLoc_Location(), group=None):
    """如果 shape 是带Location的compound(且非根) → 新组; 否则并入当前组."""
    if shape.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPOUND and not loc.IsIdentity():
        # 新实例组 (根除外由调用处理)
        ng = {'bbox': None, 'n': 0, 'loc': loc.Transformation()}
        groups.append(ng)
        group = ng
    # 递归找 solid
    ex = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_SOLID)
    has = False
    it = TopoDS_Iterator(shape)
    while it.More():
        child = it.Value()
        child_loc = loc * child.Location()
        if child.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPOUND and not child.Location().IsIdentity():
            walk_instances(child, loc)   # 子组
        else:
            walk_instances(child, child_loc, group)
        it.Next()
    if has:
        pass
    # 若当前层有位于本组(g 即 group)的固体, 收集
    if group is not None and not has:
        pass

def collect(shape, loc, groups):
    ex = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_SOLID)
    if ex.More():
        moved = shape.Moved(loc)
        if not loc.IsIdentity():
            b = bbox_of(moved)
            groups.append({'bbox': b, 'solid_count': 1})
        return
    it = TopoDS_Iterator(shape)
    while it.More():
        child = it.Value()
        cl = loc * child.Location()
        if child.ShapeType() == TopAbs_ShapeEnum.TopAbs_COMPOUND and (child.Location().IsIdentity() or child.ShapeType()!=0):
            collect(child, cl, groups)
        else:
            collect(child, cl, groups)
        it.Next()

collect(root, TopLoc_Location(), groups)
print('顶层solid组数:', len(groups))
# 只显示有实际bbox且非零
nz=[g for g in groups if g['bbox']]
print('展示 (x0..x1,y0..y1,z0..z1) mm:')
for i,g in enumerate(nz):
    b=g['bbox']
    print(f'[{i:3d}] x[{b[0]*1000:8.0f},{b[1]*1000:8.0f}] y[{b[2]*1000:8.0f},{b[3]*1000:8.0f}] z[{b[4]*1000:8.0f},{b[5]*1000:8.0f}]  size=({(b[1]-b[0])*1000:.0f},{(b[3]-b[2])*1000:.0f},{(b[5]-b[4])*1000:.0f})')
json.dump([{'bbox':g['bbox']} for g in nz], open('/tmp/step_instances.json','w'))
