#!/usr/bin/env python3
"""STEP compound+Location: 全部 solid 世界bbox/体积, 只保留大件。"""
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_ShapeEnum
from OCP.TopoDS import TopoDS_Iterator
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopLoc import TopLoc_Location
import json

r = STEPControl_Reader(); r.ReadFile('/home/arduino/Dummy.step'); r.TransferRoots()
root = r.Shape(1)

def props(sh):
    bb = Bnd_Box(); BRepBndLib.Add_s(sh, bb)
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(sh, g)
    return ([bb.CornerMin().X(), bb.CornerMin().Y(), bb.CornerMin().Z(),
             bb.CornerMax().X(), bb.CornerMax().Y(), bb.CornerMax().Z()],
            g.Mass())

out = []
def rec(sh, loc):
    it = TopoDS_Iterator(sh)
    while it.More():
        c = it.Value()
        cl = loc * c.Location()
        t = c.ShapeType()
        if t == TopAbs_ShapeEnum.TopAbs_SOLID:
            m = c.Moved(cl)
            b, vol = props(m)
            if vol > 5e-6:      # >5cm^3
                out.append({'bbox': b, 'vol_cm3': vol*1e6})
        else:
            rec(c, cl)
        it.Next()

rec(root, TopLoc_Location())
out.sort(key=lambda g: -g['vol_cm3'])
print('大实体数:', len(out))
print('体积(cm³) | 世界bbox(mm, 推测单位需核)')
for i,g in enumerate(out[:40]):
    b=g['bbox']
    print(f"[{i:3d}] {g['vol_cm3']:9.1f}  x[{b[0]:9.1f},{b[1]:9.1f}] y[{b[2]:9.1f},{b[3]:9.1f}] z[{b[4]:9.1f},{b[5]:9.1f}]  s=({(b[1]-b[0]):.1f},{(b[3]-b[2]):.1f},{(b[5]-b[4]):.1f})")
json.dump(out, open('/tmp/step_solids.json','w'))
