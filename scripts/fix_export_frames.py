#!/usr/bin/env python3
"""方案B: 用 frame_model 的 origin_global + 父link世界位姿 修正导出关节 origin 平移。

原理: 导出器的 joint.origin_xyz 是 Fusion 空间原始平移(含设计世界米级偏移);
      origin_global(全局坐标, 真实!) 且 origin_method=joint_minus_parent
      → 正确 parent 系平移 = R_parent⁻¹ · (origin_global − P_parent)。
只改 origin xyz, rpy/axis/mesh 全部保持不变 → 帧贴回网格。
输出修正后的 6 个 origin, 供 patch 生成器。零风险(离线)。
"""
import json, math
import numpy as np

fm = json.load(open('/home/arduino/dummyurdf/debug/frame_model.json'))
model = fm['model']
J = model['joints']; L = model['links']

def rpy_mat(r,p,y):
    ca,sa=math.cos(r),math.sin(r); cb,sb=math.cos(p),math.sin(p); cg,sy=math.cos(y),math.sin(y)
    Rx=np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]]); Ry=np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]]); Rz=np.array([[cg,-sy,0],[sy,cg,0],[0,0,1]])
    return Rz@Ry@Rx

def world_rot(link):
    v = L[link]['source_world_rotation']
    if len(v) != 9:  # 可能是16?
        return np.eye(3)
    R = np.array(v,float).reshape(3,3)
    return R

def global_pos(link):
    return np.array(L[link]['global_position'],float)

out = {}
for i in range(1, 7):
    j = J[f'_{i}']
    parent = j['parent_link']; child = j['child_link']
    if parent in L and child in L:
        R_p = world_rot(parent); P_p = global_pos(parent)
        O = np.array(j['origin_global'], float)
        if R_p.shape == (3,3):
            xyz = R_p.T @ (O - P_p)          # 父系坐标(转置=逆)
        else:
            xyz = O - P_p
        out[i] = {'parent': parent, 'child': child,
                  'xyz_orig': j['origin_xyz'], 'xyz_corr': [round(float(v),6) for v in xyz],
                  'rpy': j['origin_rpy'], 'axis': j['axis'],
                  'O': O.tolist(), 'P_p': P_p.tolist()}
        print(f'_ {i} ({parent}→{child}):')
        print(f'   原 xyz = {[round(x,4) for x in out[i]["xyz_orig"][:3]]}')
        print(f'   修正 xyz = {[round(v,5) for v in xyz]}   |O-Pp|={np.linalg.norm(O-P_p):.4f} m')
json.dump(out, open('/tmp/frame_fix.json','w'), ensure_ascii=False, indent=1)
print('saved /tmp/frame_fix.json')
