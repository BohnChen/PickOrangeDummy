#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""布局体检：检查越界、文本溢出、卡片重叠"""
import sys
from pptx import Presentation
from pptx.util import Emu

PATH = "/Users/bohn/04_Project/PickOrangeDummy_QualcommEdge_答辩汇报.pptx"
prs = Presentation(PATH)
SW = prs.slide_width
SH = prs.slide_height
EMU = 914400.0

def inch(v):
    return round(v / EMU, 2)

issues = []
for idx, slide in enumerate(prs.slides, 1):
    boxes = []
    for sh in slide.shapes:
        try:
            l, t, w, h = sh.left, sh.top, sh.width, sh.height
        except Exception:
            continue
        if l is None:
            continue
        r, b = l + w, t + h
        name = sh.shape_type
        txt = ""
        if sh.has_text_frame:
            txt = sh.text_frame.text.replace("\n", " / ")[:40]
        # 越界
        tol = 12700  # 0.014in
        if l < -tol or t < -tol or r > SW + tol or b > SH + tol:
            issues.append(f"[P{idx}] OUT-OF-BOUNDS {sh.shape_id} "
                          f"({inch(l)},{inch(t)},{inch(w)}x{inch(h)}) '{txt}'")
        boxes.append((sh, l, t, w, h, txt))

    # 文本框相互重叠（仅文本形状之间，忽略背景/图片）
    texts = [b for b in boxes if b[0].has_text_frame and b[0].text_frame.text.strip()
             and b[0].shape_type is not None]
    for i in range(len(texts)):
        for j in range(i + 1, len(texts)):
            s1, l1, t1, w1, h1, x1 = texts[i]
            s2, l2, t2, w2, h2, x2 = texts[j]
            ox = min(l1 + w1, l2 + w2) - max(l1, l2)
            oy = min(t1 + h1, t2 + h2) - max(t1, t2)
            if ox > 0 and oy > 0:
                area = ox * oy
                min_area = min(w1 * h1, w2 * h2)
                if min_area > 0 and area / min_area > 0.30:
                    issues.append(
                        f"[P{idx}] TEXT-OVERLAP {100*area/min_area:.0f}% "
                        f"'{x1}' <-> '{x2}'")

print(f"slides: {len(prs.slides.__iter__.__self__._sldIdLst)}")
print(f"size: {inch(SW)} x {inch(SH)} in")
if issues:
    print("\n".join(issues))
else:
    print("no issues")
