#!/usr/bin/env python3
"""Dummy.step (Fusion/SolidWorks 装配) 语句级解析 → 主件世界位姿提取。
阶段1: 可靠实体库 + 关键结构发现 (transform/rep/product/context 关联)。
"""
import re, json

SRC='/home/arduino/Dummy.step'
s=open(SRC,encoding='utf-8',errors='ignore').read()
# 去掉注释 /* */; 取 DATA 段
s=re.sub(r'/\*.*?\*/','',s,flags=re.S)
m=re.search(r'DATA;\s*\n(.*?)\nENDSEC;', s, re.S)
data=m.group(1)
# 语句级切分 (顶层分号)
stmts=[]
cur=[]
depth=0
instr=False
for ch in data:
    if ch=="'":
        instr=not instr
    if instr:
        cur.append(ch); continue
    if ch=='(':
        depth+=1
    elif ch==')':
        depth-=1
    if ch==';' and depth==0:
        stmts.append(''.join(cur)); cur=[]
    else:
        cur.append(ch)
if cur:
    stmts.append(''.join(cur))
print('语句数', len(stmts))

E={}
for st in stmts:
    mm=re.match(r'\s*#(\d+)\s*=\s*([A-Z_][A-Z0-9_]*)\s*\(', st)
    if mm:
        eid=int(mm.group(1)); kind=mm.group(2)
        body=st[mm.end()-1:]  # 含 '('
        # 截到最后一个 ')'
        if body.endswith(')'): body=body[:-1]
        E[eid]=(kind, body)
print('实体数', len(E))
json.dump({k:[v[0], v[1][:400]] for k,v in E.items()}, open('/tmp/step_ents.json','w'))
# 概览
from collections import Counter
print('类型分布(top):', Counter(k for k,b in E.values()).most_common(18))
print()
# 关键关联样例: ITEM_DEFINED_TRANSFORMATION
n=0
for eid,(k,b) in E.items():
    if k=='ITEM_DEFINED_TRANSFORMATION':
        print('TRAN e#%d: %s'%(eid,b[:120])); n+=1
        if n>=3: break
print()
# PRODUCT 名字与 id
pr=[]
for eid,(k,b) in E.items():
    if k=='PRODUCT':
        name=re.search(r"PRODUCT\s*\(\s*'([^']*)'",b)
        if name:
            pr.append((eid,name.group(1)))
print('PRODUCT ids:', pr[:20])
print()
# 关系类型数 & 带transform的
relcnt=Counter(k for k,b in E.values() if 'RELATIONSHIP' in k)
print('关系类型:', relcnt)
