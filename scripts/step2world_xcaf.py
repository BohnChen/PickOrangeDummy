#!/usr/bin/env python3
"""OCC(XCAF) 装配提取: 读 Dummy.step → 列出每个实体的 名字+世界位姿(4x4)+bbox。

用 OCP 标准 API; 配合 step2world 输出 JSON。 要求: python3 -c "import OCP" 通过。
"""
import json, math, sys

def main():
    from OCP.OCP.collections import Sequence_TDF_Label
    from OCP.XCAFDoc import XCAFDoc_ShapeTool as XST
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.Interface import Interface_Static
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDocStd import TDocStd_Document
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.BRep import BRep_Tool
    from OCP.TDataStd import TDataStd_Name
    from OCP.TCollection import TCollection_ExtendedString

    path = '/home/arduino/Dummy.step'
    doc = TDocStd_Document(TCollection_ExtendedString('dummy'))
    # 强制映射装配(XCAF)
    rdr = STEPCAFControl_Reader()
    if not rdr.ReadFile(path):
        print('读取失败'); sys.exit(1)
    if not rdr.Transfer(doc):
        print('Transfer 失败'); sys.exit(1)

    main = doc.Main()
    shapeTool = XCAFDoc_DocumentTool.ShapeTool_s(main)
    XCAFDoc_ShapeTool = XST
    def name_of(label):
        if label.HasAttribute():
            h = TDataStd_Name()
            if label.FindAttribute(TDataStd_Name.GetID(), h):
                return str(h.Get())
        return ''
    def mat4(loc):
        # gp_Trsf → 4x4
        t = loc.Transformation()
        g = t.Value(1,1); m = [[t.Value(r,c) for c in range(1,5)] for r in range(1,5)]
        return m

    out = []
    def subseq(label):
        seq = Sequence_TDF_Label()
        XCAFDoc_ShapeTool.GetSubShapes_s(label, seq)
        return [seq.Value(i+1) for i in range(seq.Length())]
    def walk(label, parent_loc=TopLoc_Location()):
        shp = XCAFDoc_ShapeTool.GetShape_s(label)
        loc = parent_loc * shp.Location()
        if XCAFDoc_ShapeTool.IsAssembly_s(label):
            for sub in subseq(label):
                walk(sub, loc)
        else:
            # 叶: solid
            nm = name_of(label)
            if not nm:
                nm = name_of(shapeTool.GetReferredShape(label) if False else label)
            b = Bnd_Box()
            if not shp.IsNull():
                BRepBndLib.Add_s(shp, b)
                x1,y1,z1,x2,y2,z2 = b.CornerMin().X(),b.CornerMin().Y(),b.CornerMin().Z(), \
                                    b.CornerMax().X(),b.CornerMax().Y(),b.CornerMax().Z()
            else:
                x1=y1=z1=x2=y2=z2=0.0
            out.append({'name': nm, 'xyz_min': [x1,y1,z1], 'xyz_max': [x2,y2,z2],
                        'M': mat4(loc)})

    fs = Sequence_TDF_Label()
    shapeTool.GetFreeShapes(fs)
    for i in range(fs.Length()):
        walk(fs.Value(i+1), TopLoc_Location())
    print('实例数:', len(out))
    for o in out:
        s=o['name']
        print(f"{s:40s} min=({[round(v,1) for v in o['xyz_min']]}) max=({[round(v,1) for v in o['xyz_max']]})")
    json.dump(out, open('/tmp/step_xcaf.json','w'), ensure_ascii=False, indent=1)

if __name__ == '__main__':
    main()
