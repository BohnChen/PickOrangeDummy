#!/usr/bin/env python3
"""自检渲染: 把 7 件网格按(链零位×各件rpy/t) 变换后 3D 渲染 PNG, 供 opcode 目检。"""
import struct, math, json, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

MESHDIR = '/home/arduino/PickOrangeDummy/src/dummyarm_description/meshes'
VIS = json.load(open('/tmp/vis_precise.json'))

def load_stl(name, step=7):
    data = open(os.path.join(MESHDIR, name), 'rb').read()
    off = 84; vs = []
    while off + 50 <= len(data):
        d = struct.unpack_from('<12fH', data, off)
        vs += [d[3:6], d[6:9], d[9:12]]
        off += 50
    V = np.array(vs, float) / 1000.0
    return V[::step]

def rpy_mat(r, p, y):
    ca = math.cos(r); sa = math.sin(r); cb = math.cos(p); sb = math.sin(p); cg = math.cos(y); sg = math.sin(y)
    Rx = np.array([[1,0,0],[0,ca,-sa],[0,sa,ca]]); Ry = np.array([[cb,0,sb],[0,1,0],[-sb,0,cb]]); Rz = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
    return Rz @ Ry @ Rx

# 链零位: 各 link 世界姿态 (原点+旋转) — 由 generate_true_urdf 链
AX = [(0,0,1),(0,-1,0),(0,-1,0),(1,0,0),(0,-1,0),(1,0,0)]
ORG= [[0,0,0],[0.035,0,0.0925],[0.146,0,0.050],[-0.052,0,0.115],[0,0,0],[0,0,0]]
def chain_links(q_deg=(0,)*6):
    R = np.eye(3); pos = np.zeros(3); frames = {}
    frames['base_link'] = (R.copy(), pos.copy())
    for k in range(6):
        a = np.array(AX[k], float); qr = q_deg[k]*math.pi/180
        c = math.cos(qr); s = math.sin(qr); t = 1-c
        Ra = np.array([[t*a[0]*a[0]+c, t*a[0]*a[1]-s*a[2], t*a[0]*a[2]+s*a[1]],
                       [t*a[0]*a[1]+s*a[2], t*a[1]*a[1]+c, t*a[1]*a[2]-s*a[0]],
                       [t*a[0]*a[2]-s*a[1], t*a[1]*a[2]+s*a[0], t*a[2]*a[2]+c]], float)
        pos = pos + R @ np.array(ORG[k], float)
        R = R @ Ra
        frames[f'link_{k+1}'] = (R.copy(), pos.copy())
    return frames

frames = chain_links()
parts = {  # link → (stl, vis)
 'link_1': ('arm_J1.stl','J1'), 'link_2': ('arm_J23.stl','J23'), 'link_3': ('arm_J4.stl','J4'),
 'link_4': ('arm_J5A.stl','J5A'), 'link_5': ('arm_J5B.stl','J5B'), 'link_6': ('arm_J6.stl','J6')}

fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')
mesh_list = [('arm_base.stl', np.eye(3), np.zeros(3))]
for ln, (stl, key) in parts.items():
    Rl, pl = frames[ln]
    delta = VIS.get(ln, {})
    xyz = np.array(delta.get('xyz', [0,0,0])); rpy = delta.get('rpy', [0,0,0])
    Rv = rpy_mat(*rpy)
    V = load_stl(stl)
    # 世界 = link(R)@(Rv@v+xyz) + link_pos
    W = (Rv @ V.T).T + xyz
    W = (Rl @ W.T).T + pl
    mesh_list.append((stl, W))
    print(f'{ln}: 世界云中心={W.mean(0).round(3)} 范围={ (W.max(0)-W.min(0)).round(3)}')

Vb = load_stl('arm_base.stl')
mesh_list[0] = ('arm_base.stl', Vb)

for name, W in mesh_list:
    ax.scatter(W[:,0], W[:,1], W[:,2], s=0.2, label=name)
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_box_aspect((1,1,1)); ax.legend(fontsize=6)
plt.savefig('/tmp/assembly_preview.png', dpi=110)
print('saved /tmp/assembly_preview.png')
