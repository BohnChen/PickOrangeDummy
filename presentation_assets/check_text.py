#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用真实字体度量检查文本是否会溢出其容器。"""
import os
from pptx import Presentation
from pptx.util import Pt
from PIL import ImageFont

PATH = "/Users/bohn/04_Project/PickOrangeDummy_QualcommEdge_答辩汇报.pptx"
FONT_REG = "/System/Library/Fonts/STHeiti Medium.ttc"
FONT_BOLD = "/System/Library/Fonts/STHeiti Medium.ttc"

EMU = 914400.0
_cache = {}

def font(size_pt, bold):
    key = (round(size_pt, 1), bold)
    if key not in _cache:
        path = FONT_BOLD if bold else FONT_REG
        # PIL 用像素大小；1pt @96dpi ≈ 1.333px，这里直接按 px 近似，保证度量偏保守
        _cache[key] = ImageFont.truetype(path, max(6, int(round(size_pt * 1.333))))
    return _cache[key]

def text_width_pt(s, size_pt, bold):
    f = font(size_pt, bold)
    return f.getlength(s) * 0.75  # px -> pt

def wrap_lines(runs, box_w_pt):
    """runs: [(text, size, bold)]，按字符贪心折行，返回行数"""
    if not runs:
        return 1
    lines = 1
    cur = 0.0
    for text, size, bold in runs:
        for ch in text:
            w = text_width_pt(ch, size, bold)
            if cur + w > box_w_pt and cur > 0:
                lines += 1
                cur = w
            else:
                cur += w
    return lines

prs = Presentation(PATH)
issues = []
for idx, slide in enumerate(prs.slides, 1):
    for sh in slide.shapes:
        if not sh.has_text_frame:
            continue
        tf = sh.text_frame
        if not tf.text.strip():
            continue
        box_w_in = sh.width / EMU - (tf.margin_left + tf.margin_right) / EMU
        box_h_in = sh.height / EMU - (tf.margin_top + tf.margin_bottom) / EMU
        box_w_pt = box_w_in * 72
        need_pt = 0.0
        for p in tf.paragraphs:
            runs = []
            for r in p.runs:
                size = r.font.size.pt if r.font.size else 12
                runs.append((r.text, size, bool(r.font.bold)))
            if not runs:
                continue
            size0 = runs[0][1] if runs else 12
            n = wrap_lines(runs, box_w_pt) if tf.word_wrap else 1
            ls = p.line_spacing if isinstance(p.line_spacing, float) else 1.2
            need_pt += n * size0 * 1.22 * ls
            if p.space_before:
                need_pt += p.space_before.pt
            if p.space_after:
                need_pt += p.space_after.pt
        need_in = need_pt / 72
        if need_in > box_h_in + 0.02:
            issues.append(
                f"[P{idx}] NEED {need_in:.2f}in > HAVE {box_h_in:.2f}in | "
                f"{tf.text.replace(chr(10),' / ')[:52]}")

print(f"checked slides: {len(prs.slides.__iter__.__self__._sldIdLst)}")
if issues:
    print(f"overflow candidates: {len(issues)}")
    print("\n".join(issues))
else:
    print("no overflow")
