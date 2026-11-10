#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PickOrangeDummy · 2026 高通智能边缘大赛 答辩 PPT 生成器（v2：通俗版）
- 16:9 / 13.333 x 7.5 in
- 原生形状，可编辑
- 每页含演讲备注
"""

import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn
from PIL import Image

ROOT = "/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy"
A = lambda *p: os.path.join(ROOT, *p)

PHOTO_TOP = A("presentation_assets", "photo_jpg", "IMG_5588.jpg")
PHOTO_DETAIL = A("presentation_assets", "photo_jpg", "IMG_5589.jpg")
QUAD = A("presentation_assets", "video_frames", "v5_05s.jpg")
MOVEIT = A("presentation_assets", "video_frames", "v3_10s.jpg")

OUT = "/Users/bohn/04_Project/PickOrangeDummy_QualcommEdge_答辩汇报.pptx"

NAVY   = RGBColor(0x0B, 0x12, 0x20)
INK    = RGBColor(0x0F, 0x17, 0x2A)
SLATE  = RGBColor(0x47, 0x55, 0x69)
MUTED  = RGBColor(0x94, 0xA3, 0xB8)
LINE   = RGBColor(0xE2, 0xE8, 0xF0)
CARD   = RGBColor(0xF6, 0xF8, 0xFA)
CARD2  = RGBColor(0xEE, 0xF4, 0xFB)
BLUE   = RGBColor(0x1D, 0x4E, 0xD8)
CYAN   = RGBColor(0x0E, 0xA5, 0xE9)
ORANGE = RGBColor(0xF9, 0x73, 0x16)
AMBER  = RGBColor(0xF5, 0x9E, 0x0B)
GREEN  = RGBColor(0x05, 0x96, 0x69)
WHITE  = RGBColor(0xFF, 0xFF, 0xFF)

EA_FONT = "PingFang SC"
LAT_FONT = "PingFang SC"

SW, SH = 13.333, 7.5
MARGIN = 0.62
CONTENT_W = SW - 2 * MARGIN

prs = Presentation()
prs.slide_width = Inches(SW)
prs.slide_height = Inches(SH)
BLANK = prs.slide_layouts[6]
_page = {"n": 0}


def set_run(run, size=14, bold=False, color=INK, ea=EA_FONT, lat=LAT_FONT, italic=False):
    f = run.font
    f.size = Pt(size)
    f.bold = bold
    f.italic = italic
    f.color.rgb = color
    f.name = lat
    rPr = run._r.get_or_add_rPr()
    for tag in ("a:ea", "a:cs"):
        el = rPr.find(qn(tag))
        if el is None:
            el = rPr.makeelement(qn(tag), {})
            rPr.append(el)
        el.set("typeface", ea)


def textbox(slide, x, y, w, h, anchor=MSO_ANCHOR.TOP, wrap=True):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = wrap
    tf.margin_left = tf.margin_right = 0
    tf.margin_top = tf.margin_bottom = 0
    tf.vertical_anchor = anchor
    return tf


def para(tf, text, size=14, bold=False, color=INK, space_before=0, space_after=0,
         align=PP_ALIGN.LEFT, line_spacing=1.25, first=False, italic=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    p.line_spacing = line_spacing
    r = p.add_run()
    r.text = text
    set_run(r, size, bold, color, italic=italic)
    return p


def rich(tf, parts, size=14, space_before=0, space_after=0, align=PP_ALIGN.LEFT,
         line_spacing=1.3, first=False):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    p.line_spacing = line_spacing
    for text, bold, color, sz in parts:
        r = p.add_run()
        r.text = text
        set_run(r, sz or size, bold, color)
    return p


def rect(slide, x, y, w, h, fill=CARD, line=None, radius=None, line_w=1.0):
    shape_type = MSO_SHAPE.ROUNDED_RECTANGLE if radius is not None else MSO_SHAPE.RECTANGLE
    sh = slide.shapes.add_shape(shape_type, Inches(x), Inches(y), Inches(w), Inches(h))
    if radius is not None:
        sh.adjustments[0] = radius
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid()
        sh.fill.fore_color.rgb = fill
    if line is None:
        sh.line.fill.background()
    else:
        sh.line.color.rgb = line
        sh.line.width = Pt(line_w)
    sh.shadow.inherit = False
    tf = sh.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = Inches(0.14)
    tf.margin_top = tf.margin_bottom = Inches(0.10)
    return sh


def circle(slide, x, y, d, fill=BLUE):
    sh = slide.shapes.add_shape(MSO_SHAPE.OVAL, Inches(x), Inches(y), Inches(d), Inches(d))
    sh.fill.solid()
    sh.fill.fore_color.rgb = fill
    sh.line.fill.background()
    sh.shadow.inherit = False
    return sh


def picture_fit(slide, path, x, y, w, h, border=None):
    iw, ih = Image.open(path).size
    ar = iw / ih
    if w / h > ar:
        ph, pw = h, h * ar
    else:
        pw, ph = w, w / ar
    px, py = x + (w - pw) / 2, y + (h - ph) / 2
    pic = slide.shapes.add_picture(path, Inches(px), Inches(py), Inches(pw), Inches(ph))
    if border is not None:
        pic.line.color.rgb = border
        pic.line.width = Pt(1.25)
    return pic


def new_slide(bg=WHITE, footer=True):
    s = prs.slides.add_slide(BLANK)
    _page["n"] += 1
    bgshape = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, prs.slide_height)
    bgshape.fill.solid()
    bgshape.fill.fore_color.rgb = bg
    bgshape.line.fill.background()
    bgshape.shadow.inherit = False
    if footer:
        tf = textbox(s, MARGIN, SH - 0.44, 8.0, 0.25)
        para(tf, "PickOrangeDummy  ·  高通智能边缘大赛  ·  低位柑橘采摘机器人",
             size=9, color=MUTED, first=True)
        tf2 = textbox(s, SW - MARGIN - 0.9, SH - 0.44, 0.9, 0.25)
        para(tf2, f"{_page['n']:02d}", size=9, color=MUTED, align=PP_ALIGN.RIGHT, first=True)
    return s


def header(slide, tag, title, subtitle=None, tag_color=BLUE, title_color=INK,
           sub_color=SLATE, y=0.52):
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(y + 0.07),
                                 Inches(0.055), Inches(0.30))
    bar.fill.solid()
    bar.fill.fore_color.rgb = tag_color
    bar.line.fill.background()
    bar.shadow.inherit = False

    tf = textbox(slide, MARGIN + 0.20, y, CONTENT_W - 0.2, 0.26)
    para(tf, tag, size=10.5, bold=True, color=tag_color, first=True)

    tf = textbox(slide, MARGIN, y + 0.30, CONTENT_W, 0.52)
    para(tf, title, size=25, bold=True, color=title_color, first=True, line_spacing=1.0)

    if subtitle:
        tf = textbox(slide, MARGIN, y + 0.92, CONTENT_W, 0.40)
        para(tf, subtitle, size=12, color=sub_color, first=True, line_spacing=1.3)


def notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


def metric(slide, x, y, w, h, value, unit, label, color=BLUE):
    rect(slide, x, y, w, h, fill=CARD, line=LINE, radius=0.06)
    tf = textbox(slide, x + 0.16, y + 0.14, w - 0.32, 0.55)
    parts = [(value, True, color, 25)]
    if unit:
        parts.append((unit, True, color, 11))
    rich(tf, parts, first=True, line_spacing=1.0)
    tf = textbox(slide, x + 0.16, y + 0.70, w - 0.32, h - 0.78)
    para(tf, label, size=9, color=SLATE, first=True, line_spacing=1.2)


# ================================================================ 01 封面
s = new_slide(bg=NAVY, footer=False)
ln = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, Inches(0.06))
ln.fill.solid(); ln.fill.fore_color.rgb = BLUE; ln.line.fill.background(); ln.shadow.inherit = False

picture_fit(s, PHOTO_DETAIL, 7.55, 0.62, 5.16, 6.26, border=RGBColor(0x2A, 0x3A, 0x55))

tf = textbox(s, MARGIN, 0.72, 6.6, 0.3)
para(tf, "2026 高通智能边缘大赛  ·  参赛答辩", size=11, bold=True,
     color=RGBColor(0x60, 0xA5, 0xFA), first=True)

tf = textbox(s, MARGIN, 1.28, 6.7, 1.9)
para(tf, "基于高通智能边缘平台的", size=27, bold=True, color=WHITE, first=True, line_spacing=1.25)
para(tf, "低位柑橘采摘机器人", size=40, bold=True, color=WHITE, line_spacing=1.15)

bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(3.32),
                         Inches(1.05), Inches(0.05))
bar.fill.solid(); bar.fill.fore_color.rgb = ORANGE; bar.line.fill.background(); bar.shadow.inherit = False

tf = textbox(s, MARGIN, 3.62, 6.6, 0.9)
para(tf, "PickOrangeDummy", size=16, bold=True, color=RGBColor(0xFD, 0xBA, 0x74), first=True)
para(tf, "一块板子管三件事：看得见果子、走得进果园、摘得下果实。",
     size=14, color=RGBColor(0xCB, 0xD5, 0xE1), line_spacing=1.45, space_before=6)

items = [("40 TOPS", "车载 AI 算力"), ("8 核", "处理器"), ("1 块", "计算板"), ("全程", "本地运行")]
bx = MARGIN
for v, l in items:
    rect(s, bx, 5.05, 1.52, 0.92, fill=RGBColor(0x16, 0x21, 0x38),
         line=RGBColor(0x2A, 0x3A, 0x55), radius=0.08)
    tf = textbox(s, bx + 0.12, 5.17, 1.3, 0.32)
    para(tf, v, size=15, bold=True, color=WHITE, first=True, line_spacing=1.0)
    tf = textbox(s, bx + 0.12, 5.53, 1.3, 0.26)
    para(tf, l, size=8, color=MUTED, first=True, line_spacing=1.0)
    bx += 1.64

tf = textbox(s, MARGIN, 6.42, 6.6, 0.6)
para(tf, "Ubuntu 24.04  ·  ROS 2 Jazzy  ·  MoveIt 2  ·  YOLO11n-seg", size=9.5,
     color=RGBColor(0x7D, 0x8C, 0xA3), first=True)
para(tf, "汇报人：__________        指导教师：__________        2026.11", size=9.5,
     color=RGBColor(0x7D, 0x8C, 0xA3), space_before=4)

notes(s, """【开场 30 秒】
各位评委老师好，我汇报的项目是「基于高通智能边缘平台的低位柑橘采摘机器人」。

一句话说清楚：这块高通边缘计算板，把「看得见果子、走得进果园、摘得下果实」三件事，全部扛在了一块板子上。车上没有工控机，没有外接电脑，也不联网跑云。

接下来我讲四件事：为什么要做、系统是怎么搭的以及为什么这么搭、做成了什么、以及过程中真实遇到的难题和我做的取舍。""")

# ================================================================ 02 痛点
s = new_slide()
header(s, "01 / 项目背景", "柑橘采摘，卡在「最后一米」",
       "大机器进不了窄果园，人又越来越难请 —— 缺的不是力气，是「走到树前、把一颗果摘下来」的能力。")

cards = [
    ("时间窗口太短", "柑橘成熟很集中，最佳采摘期只有 60 到 90 天。错过了就落果、掉价，一年的收成受影响。",
     "季节性用工缺口", ORANGE),
    ("请人越来越贵", "采摘人工普遍占到生产总成本的三成以上，而愿意下地干活的人越来越少、年龄越来越大。",
     "劳动力老龄化", ORANGE),
    ("大机器进不去", "丘陵果园行距窄、地面不平，大型采收机开不进来；背着工控机下地，又贵、又重、还怕震。",
     "体积 / 成本 / 可靠性", ORANGE),
]
x = MARGIN
for title, body, tag, col in cards:
    rect(s, x, 2.05, 3.86, 2.32, fill=WHITE, line=LINE, radius=0.05)
    top = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(2.05), Inches(3.86), Inches(0.055))
    top.fill.solid(); top.fill.fore_color.rgb = col; top.line.fill.background(); top.shadow.inherit = False

    tf = textbox(s, x + 0.26, 2.26, 3.34, 0.34)
    para(tf, title, size=15, bold=True, color=INK, first=True, line_spacing=1.1)
    tf = textbox(s, x + 0.26, 2.72, 3.34, 1.20)
    para(tf, body, size=11, color=SLATE, first=True, line_spacing=1.5)
    tf = textbox(s, x + 0.26, 3.96, 3.34, 0.28)
    para(tf, "· " + tag, size=9.5, bold=True, color=ORANGE, first=True)
    x += 4.04

rect(s, MARGIN, 4.72, CONTENT_W, 1.28, fill=NAVY, line=None, radius=0.035)
tf = textbox(s, MARGIN + 0.42, 4.92, 1.75, 0.9, anchor=MSO_ANCHOR.MIDDLE)
para(tf, "我们的定位", size=10, bold=True, color=RGBColor(0xFD, 0xBA, 0x74), first=True)

tf = textbox(s, MARGIN + 2.20, 4.92, CONTENT_W - 2.6, 0.9, anchor=MSO_ANCHOR.MIDDLE)
para(tf, "不造更大的机器，造一台「钻得进窄行、对得准单颗果、能一直干下去」的小车 ——",
     size=14, bold=True, color=WHITE, first=True, line_spacing=1.35)
para(tf, "让一块边缘计算板，直接把「看见 → 对准 → 剪下」这一整套动作在自己身上跑完。",
     size=14, bold=True, color=RGBColor(0x60, 0xA5, 0xFA), line_spacing=1.35, space_before=2)

notes(s, """【背景 45 秒】
先说为什么值得做。

柑橘是典型的劳动密集型作物：成熟期集中，采摘窗口只有两三个月；人工支出占生产成本三成以上；而愿意下地摘果的人越来越少、越来越老。

已有的自动化方案有两个方向，但都不合适。一是大型采收机械 —— 丘陵密植果园行距窄、地面不平，根本开不进去。二是通用工控机加外挂算力 —— 车上一堆盒子，贵、重、怕震，成本压不下来。

所以我们把问题重新定义：不是缺力气，是缺「最后一米」的能力 —— 要能钻窄行、要能对准单颗果、还要能便宜。这三件事，正好是边缘 AI 加轻量机械臂能解决的。""")

# ================================================================ 03 架构
s = new_slide()
header(s, "02 / 系统架构", "车上只有一块板，软件分成五层",
       "上车一看只有一台计算板；软件分成五层，上层负责「想」，下层负责「动」，谁也不越级指挥。")

layers = [
    ("L4", "交互与安全", "人怎么指挥它 —— 人工确认、紧急停止", RGBColor(0x7C, 0x3A, 0xED)),
    ("L3", "任务编排", "现在该干什么 —— 一张流程表，按节拍往下走", BLUE),
    ("L2", "动作能力", "它会做的每个动作 —— 动手臂、剪切、开车、找果", CYAN),
    ("L1", "设备接入", "怎么跟硬件说话 —— 把相机、底盘、机械臂变成标准设备", RGBColor(0x0D, 0x94, 0x88)),
    ("L0", "底层固件", "真正让电机转、也真正能踩刹车的地方", SLATE),
]
ly = 2.06
for tag, name, desc, col in layers:
    rect(s, MARGIN, ly, 6.28, 0.80, fill=CARD, line=LINE, radius=0.06)
    badge = rect(s, MARGIN + 0.14, ly + 0.17, 0.62, 0.46, fill=col, line=None, radius=0.12)
    tf = badge.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf, tag, size=12, bold=True, color=WHITE, align=PP_ALIGN.CENTER, first=True, line_spacing=1.0)

    tf = textbox(s, MARGIN + 0.90, ly + 0.13, 1.72, 0.30)
    para(tf, name, size=11.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 0.90, ly + 0.44, 5.20, 0.28)
    para(tf, desc, size=9.5, color=SLATE, first=True, line_spacing=1.05)
    ly += 0.88

tf = textbox(s, MARGIN, ly + 0.02, 6.28, 0.3)
para(tf, "↑ 只允许上层调用下层；下层不需要知道上层在干什么", size=9.5, italic=True,
     color=MUTED, first=True)

hx = 7.24
rect(s, hx, 2.06, 5.48, 4.02, fill=CARD2, line=LINE, radius=0.04)
tf = textbox(s, hx + 0.26, 2.20, 5.0, 0.28)
para(tf, "车上的硬件，全部实测过", size=10.5, bold=True, color=BLUE, first=True)

core = rect(s, hx + 1.62, 2.62, 2.24, 0.86, fill=NAVY, line=None, radius=0.10)
tf = core.text_frame
tf.vertical_anchor = MSO_ANCHOR.MIDDLE
para(tf, "车载计算板", size=11.5, bold=True, color=WHITE, align=PP_ALIGN.CENTER,
     first=True, line_spacing=1.0)
para(tf, "8 核 CPU + 40 TOPS AI 算力", size=8, color=RGBColor(0x93, 0xC5, 0xFD),
     align=PP_ALIGN.CENTER, line_spacing=1.0)

nodes = [
    ("底盘控制芯片", "管油门、转向、刹车", hx + 0.20, 3.72, 2.30),
    ("机械臂控制芯片", "管六个关节", hx + 2.98, 3.72, 2.30),
    ("前向相机", "看路况、看远处障碍", hx + 0.20, 4.70, 2.30),
    ("侧向相机", "找橘子、量距离", hx + 2.98, 4.70, 2.30),
    ("超声波", "贴脸防撞", hx + 0.20, 5.68, 2.30),
    ("三路信号线", "油门 / 转向 / 档位", hx + 2.98, 5.68, 2.30),
]
for title, sub, nx, ny, nw in nodes:
    c = rect(s, nx, ny, nw, 0.82, fill=WHITE, line=LINE, radius=0.08)
    tf = c.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf, title, size=9.5, bold=True, color=INK, align=PP_ALIGN.CENTER, first=True, line_spacing=1.0)
    para(tf, sub, size=7.5, color=SLATE, align=PP_ALIGN.CENTER, line_spacing=1.0, space_before=2)

tf = textbox(s, hx + 0.20, 3.52, 5.1, 0.2)
para(tf, "── 一根线连下去，中间没有第二台电脑 ──", size=7.5, color=MUTED,
     align=PP_ALIGN.CENTER, first=True)

notes(s, """【架构 60 秒】
先看整体。

硬件上，这台车上只有一个大脑：一块高通的边缘计算板，8 核 CPU 加 40 TOPS 的 AI 算力，跑 Ubuntu 和 ROS 2。底盘、机械臂、两个相机、超声波，全部挂在它上面。

软件分成五层，规矩只有一条：只能上层调用下层。
最上面是人怎么指挥它，以及怎么保证安全；往下是「现在该干什么」的流程表；再往下是它会做的每一个动作；然后是「怎么跟硬件说话」这一层；最底下是控制芯片里的固件 —— 真正让电机转、也真正能踩刹车的地方。

这样分的好处很实在：以后加一个新功能，只要在对应那一层加东西，不用把五个地方都改一遍。""")

# ================================================================ 04 三条规矩
s = new_slide()
header(s, "03 / 设计原因", "三条死规矩，都是被故障教出来的",
       "每一条背后都有一次真实的翻车，不是抄来的最佳实践。")

rules = [
    ("一个部件只有一个司机", "同一条指令线上，只允许一个程序发指令。两个程序同时发，机器就会打架。",
     "起因：剪切脚本和手臂驱动抢同一个接口，采摘必定失败。", BLUE),
    ("指挥层不许被卡住", "做决策的那一层不能原地等。它一等，整台车就停在那儿。",
     "起因：一处「等三秒」，让整条流程停摆，而且逻辑写死在代码里，只有作者能改。", CYAN),
    ("最后一道保险装在芯片里", "所有能让机器动的保护，都放在底层控制芯片，不依赖电脑还活着。",
     "起因：全车只有一个总开关，一断电手臂就往下掉 —— 所以断电不能当刹车用。", GREEN),
]
x = MARGIN
for i, (title, desc, why, col) in enumerate(rules):
    rect(s, x, 2.02, 3.86, 2.44, fill=WHITE, line=LINE, radius=0.05)
    num = circle(s, x + 0.26, 2.22, 0.42, fill=col)
    tf = num.text_frame
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf, str(i + 1), size=13, bold=True, color=WHITE, align=PP_ALIGN.CENTER,
         first=True, line_spacing=1.0)

    tf = textbox(s, x + 0.80, 2.22, 2.90, 0.62)
    para(tf, title, size=12.5, bold=True, color=INK, first=True, line_spacing=1.15)

    tf = textbox(s, x + 0.26, 2.92, 3.34, 0.92)
    para(tf, desc, size=10, color=SLATE, first=True, line_spacing=1.5)

    tf = textbox(s, x + 0.26, 3.84, 3.34, 0.56)
    para(tf, why, size=8.5, color=col, first=True, line_spacing=1.35)
    x += 4.04

tf = textbox(s, MARGIN, 4.72, CONTENT_W, 0.30)
para(tf, "还有两次「主动不做」—— 忍住不加，比堆功能更难", size=13, bold=True,
     color=INK, first=True)

rest = [
    ("不做地图和定位", "轮子没有测速装置，靠算出来的位置只会越算越偏。改成「盯着目标一点点挪过去」，反而更稳。"),
    ("不做复杂的流程框架", "采摘本来就是一条直线流程。一张流程表足够清楚，简单的东西才改得动、交得出去。"),
]
x = MARGIN
for t, d in rest:
    rect(s, x, 5.14, 5.96, 1.42, fill=CARD, line=LINE, radius=0.05)
    tf = textbox(s, x + 0.24, 5.28, 5.5, 0.30)
    para(tf, "✕  " + t, size=11, bold=True, color=ORANGE, first=True, line_spacing=1.1)
    tf = textbox(s, x + 0.24, 5.64, 5.5, 0.84)
    para(tf, d, size=9.5, color=SLATE, first=True, line_spacing=1.45)
    x += 6.16

notes(s, """【设计原因 60 秒 —— 这页评委最爱追问】

这三条不是我一开始就想到的，都是被故障教出来的。

第一，一个部件只能有一个司机。我们真遇到过：剪切脚本和手臂驱动同时打开同一个接口，指令互相踩，采摘百分百失败。所以定死一条 —— 一个执行器只允许一个程序发指令。

第二，指挥层不许被卡住。做决策的那一层只要有一处「等三秒」，整台车就停在那儿；而且流程写死在代码细节里，只有我自己能改。所以上层只允许发任务、收结果。

第三，最后一道保险装在芯片里。这台车只有一个总开关，手臂和电脑同一路供电，一断电手臂直接往下掉 —— 所以「断电当刹车」这个想法是错的，刹车必须靠芯片里的保护逻辑。

另外还有两次我主动不做的事：轮子没有测速装置，做地图定位只是自我安慰，不如看着目标一点点挪过去；采摘流程本来就是直线，上复杂框架是杀鸡用牛刀。对边缘设备来说，能省掉的复杂度就是最好的性能。""")

# ================================================================ 05 感知
s = new_slide()
header(s, "04 / 感知系统", "三个传感器，互相补对方的盲区",
       "不是谁高级就用谁，而是让每个传感器「看不见的地方」，都有别的传感器接住。")

sensors = [
    ("前向相机", "看路况", "巡航时看路、看远处障碍。视野宽，能提前发现前面有东西。",
     "短板：贴身 0.6 米以内测不出距离", RGBColor(0x7C, 0x2D, 0x12)),
    ("侧向相机", "找橘子", "装在车左侧，正对采摘区。彩色加深度，能直接算出一颗果的三维位置。",
     "短板：看不到车头正前方", BLUE),
    ("超声波", "贴脸防撞", "装在车头，直接接在控制芯片上，不占相机带宽。静止后自动停止发射。",
     "短板：只知道多远，不知道在哪边", GREEN),
]
x = MARGIN
for name, role, desc, limit, col in sensors:
    rect(s, x, 2.02, 2.62, 2.06, fill=WHITE, line=LINE, radius=0.05)
    tf = textbox(s, x + 0.20, 2.16, 2.26, 0.30)
    para(tf, name, size=12, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.20, 2.46, 2.26, 0.24)
    para(tf, role, size=8.5, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.20, 2.76, 2.26, 0.74)
    para(tf, desc, size=9, color=SLATE, first=True, line_spacing=1.4)
    tf = textbox(s, x + 0.20, 3.58, 2.26, 0.44)
    para(tf, limit, size=8, color=ORANGE, first=True, line_spacing=1.2)
    x += 2.72

rect(s, 8.62, 2.02, 4.10, 2.42, fill=CARD, line=LINE, radius=0.04)
picture_fit(s, QUAD, 8.70, 2.08, 3.94, 1.86)
tf = textbox(s, 8.72, 3.98, 3.90, 0.42)
para(tf, "真机画面：上排前向相机的彩色和距离图，下排侧向相机的彩色和距离图，四路同时在线",
     size=7.2, color=SLATE, first=True, line_spacing=1.2)

tf = textbox(s, MARGIN, 4.58, CONTENT_W, 0.30)
para(tf, "从 3 米到 0 厘米，一层接一层 —— 任何一段失效，下一段还在", size=13, bold=True,
     color=INK, first=True)

chain = [
    ("3 – 0.8 米", "相机看路", "画面里「测不到距离」的区域突然变大，就判断前方有障碍", BLUE),
    ("0.8 – 0.6 米", "提前停车", "把停车线往前挪，宁可早停，也不赌它看得见", CYAN),
    ("0.6 米以内", "超声波硬拦", "直接由控制芯片拦下，不经过电脑，绕不过去", GREEN),
    ("最后一手", "0.5 秒兜底", "半秒收不到新指令，自动回正、自动停车", ORANGE),
]
x = MARGIN
for rng, who, how, col in chain:
    rect(s, x, 5.00, 2.94, 1.30, fill=CARD, line=LINE, radius=0.05)
    tf = textbox(s, x + 0.20, 5.12, 2.6, 0.28)
    para(tf, rng, size=12.5, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.20, 5.44, 2.6, 0.24)
    para(tf, who, size=9.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.20, 5.70, 2.6, 0.50)
    para(tf, how, size=8.3, color=SLATE, first=True, line_spacing=1.35)
    x += 3.08
    if x < MARGIN + 3.0 * 3:
        ar = s.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x - 0.16), Inches(5.54),
                                Inches(0.16), Inches(0.20))
        ar.fill.solid(); ar.fill.fore_color.rgb = MUTED
        ar.line.fill.background(); ar.shadow.inherit = False

notes(s, """【感知 60 秒】
感知这块，我们不比谁的传感器多，比谁的盲区补得住。

前向相机负责看路和远处障碍，视野宽。但它有个硬毛病：贴身 0.6 米以内测不出距离 —— 也就是说，障碍物在最该停的那一刻「消失」了。

侧向相机装在左侧，专门找橘子，彩色加深度，能直接给出一颗果的三维位置。但它看不到车头正前方。

所以车头加了一个超声波，直接接在控制芯片上，不占相机的带宽，专门补最后那段盲区。

下面这条链是关键：三米到 0.8 米用相机；0.8 到 0.6 米把停车线提前，宁可早停；0.6 米以内超声波硬拦，不经过电脑；最后还有半秒收不到指令就自动停车。

四段里任何一段坏了，下一段都还在。这是我们在这台车上的安全思路：不指望一个完美的传感器，而是搭一条不会同时断掉的链。""")

# ================================================================ 06 视觉
s = new_slide()
header(s, "05 / 视觉识别", "教机器认橘子，也教它别撞树",
       "不追求什么都认识，只要求分清四件事：能摘的、不能摘的、要绕开的、和果柄在哪。")

metric(s, MARGIN, 2.02, 2.28, 1.42, "86.8", "%", "能摘的果子，分割准确率", BLUE)
metric(s, MARGIN + 2.40, 2.02, 2.28, 1.42, "93.2", "%", "认出来的果子里，有多少是对的", CYAN)
metric(s, MARGIN + 4.80, 2.02, 2.28, 1.42, "17.1", "毫秒", "认一张图要多久", GREEN)
metric(s, MARGIN + 7.20, 2.02, 2.28, 1.42, "2.84", "M", "模型大小（参数）", ORANGE)
metric(s, MARGIN + 9.60, 2.02, 2.28, 1.42, "4", "类", "认得的东西有四种", SLATE)

tf = textbox(s, MARGIN, 3.66, 6.10, 0.30)
para(tf, "四个类别，对应四个决定", size=13, bold=True, color=INK, first=True)

classes = [
    ("能摘的果", "完整露在外面", "主要采摘目标，给出位置和距离，直接送去摘", GREEN, "86.8%"),
    ("被挡住的果", "被枝叶遮住一半", "判为「不好下刀」，先跳过，免得罩口硬怼枝叶", AMBER, "57.7%"),
    ("果柄", "果子连着树枝那一小段", "最终下刀的位置参考，目前样本太少，正在补", ORANGE, "补齐中"),
    ("树干树枝", "粗枝、主干", "规划前先排除，别让手臂一头撞上去", BLUE, "已加入"),
]
cy = 4.04
for name, cn, desc, col, val in classes:
    rect(s, MARGIN, cy, 6.10, 0.58, fill=CARD, line=LINE, radius=0.06)
    circle(s, MARGIN + 0.18, cy + 0.23, 0.10, fill=col)
    tf = textbox(s, MARGIN + 0.40, cy + 0.07, 1.72, 0.26)
    para(tf, name, size=9.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 0.40, cy + 0.31, 1.72, 0.22)
    para(tf, cn, size=7.5, color=MUTED, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 2.16, cy + 0.15, 3.10, 0.34)
    para(tf, desc, size=8.5, color=SLATE, first=True, line_spacing=1.25)
    tf = textbox(s, MARGIN + 5.34, cy + 0.15, 0.66, 0.30)
    para(tf, val, size=8.5, bold=True, color=col, align=PP_ALIGN.RIGHT, first=True, line_spacing=1.0)
    cy += 0.62

rect(s, 7.10, 3.66, 5.62, 2.42, fill=CARD2, line=LINE, radius=0.04)
tf = textbox(s, 7.34, 3.80, 5.14, 0.28)
para(tf, "两轮迭代，一步一个脚印", size=11, bold=True, color=BLUE, first=True)

rows = [
    ("第一轮", "269 张图 / 3 类", "先跑通流程，验证数据能训、真机能用", "准确率 48.2%"),
    ("第二轮", "518 张图 / 4 类", "样本翻倍，专门加入「树干树枝」这一类", "准确率 33.5%"),
]
ry = 4.16
for tag, data, note, res in rows:
    tf = textbox(s, 7.34, ry, 1.02, 0.26)
    para(tf, tag, size=8.5, bold=True, color=BLUE, first=True, line_spacing=1.0)
    tf = textbox(s, 8.36, ry, 2.50, 0.50)
    para(tf, data, size=8.5, bold=True, color=INK, first=True, line_spacing=1.0)
    para(tf, note, size=7.8, color=SLATE, line_spacing=1.2, space_before=1)
    tf = textbox(s, 10.94, ry, 1.54, 0.28)
    para(tf, res, size=8.5, bold=True, color=INK, align=PP_ALIGN.RIGHT, first=True, line_spacing=1.0)
    ry += 0.60

tf = textbox(s, 7.34, 5.40, 5.14, 0.62)
para(tf, "总分变低了，是因为加了难题。但能摘的果子始终保持在 80% 以上 —— 我们宁可总分难看，也要让「别撞树」这一类进来。",
     size=8, color=SLATE, first=True, line_spacing=1.35)

tf = textbox(s, MARGIN, 6.62, CONTENT_W, 0.26)
para(tf, "真机上线时优先用模型判断；万一模型没命中，再用颜色加距离兜底，保证不空转。",
     size=8.5, italic=True, color=MUTED, first=True)

notes(s, """【视觉 60 秒】
视觉这块我们踩过一个典型的坑：一开始只想做「认果子」，指标很好看。

第一轮 269 张图、3 个类别，7 轮训练，能摘的果子准确率就到 86.8%，认出来的果子里 93% 是对的，认一张图只要 17 毫秒 —— 在车载算力上完全够实时。

但真机上出了问题：模型能看见果，看不见树。手臂伸进去会撞到粗枝。所以第二轮把样本翻倍到 518 张，硬加了一个「树干树枝」类。结果总分从 48% 掉到 33% —— 分数变难看了，但这是我们主动的取舍：宁可总分下降，也要让「别撞树」这一类进来，因为撞树的代价是硬件损坏。

另外我们单独分了「被挡住的果」，遮住一半以上的先跳过，免得罩口硬怼枝叶。

也要诚实说一句：果柄目前是短板，验证集里只有几个样本，模型还没学会。这是下一轮要重点补的。""")

# ================================================================ 07 流程
s = new_slide()
header(s, "06 / 作业流程", "从「看见一颗果」到「摘下它」，再去找下一颗",
       "这是项目真正跑通的东西 —— 不是某个单点功能，而是一条能一圈一圈转下去的完整流程。")

row1 = [("巡航找果", "低速往前开\n相机看着前方"),
        ("发现目标", "侧向相机抓到果子\n确认这是能摘的"),
        ("停车", "立刻切断动力\n锁住，不许滑动"),
        ("算位置", "算出果子相对车身\n在哪、有多远"),
        ("走到预备位", "罩口停在果子前\n5 厘米处待命")]
row2 = [("慢慢伸进去", "20 度仰角\n每次进 1.5 厘米"),
        ("等人确认", "★ 停住，等指令\n不确认就不下刀"),
        ("剪断", "刀片合拢 60 度\n果子落进收集管"),
        ("退回原位", "退 8 厘米、抬起\n回到待命姿势")]


def chevron_row(items, y, start_x, w, h, colors):
    x = start_x
    for i, (title, desc) in enumerate(items):
        col = colors[i % len(colors)]
        sh = s.shapes.add_shape(MSO_SHAPE.CHEVRON, Inches(x), Inches(y), Inches(w), Inches(h))
        sh.fill.solid(); sh.fill.fore_color.rgb = col
        sh.line.fill.background(); sh.shadow.inherit = False
        tf = sh.text_frame
        tf.word_wrap = True
        tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        tf.margin_left = Inches(0.30)
        tf.margin_right = Inches(0.05)
        para(tf, title, size=10.5, bold=True, color=WHITE, align=PP_ALIGN.CENTER,
             first=True, line_spacing=1.0)
        tf2 = textbox(s, x + 0.34, y + h + 0.12, w - 0.56, 0.72)
        para(tf2, desc, size=7.8, color=SLATE, align=PP_ALIGN.CENTER, first=True, line_spacing=1.3)
        x += w - 0.30


chevron_row(row1, 1.98, MARGIN, 2.62, 0.82,
            [BLUE, RGBColor(0x25, 0x63, 0xEB), CYAN, RGBColor(0x0D, 0x94, 0x88), GREEN])
chevron_row(row2, 3.62, MARGIN + 0.62, 2.62, 0.82,
            [GREEN, ORANGE, RGBColor(0xD9, 0x77, 0x06), SLATE])

tf = textbox(s, 10.98, 3.72, 1.72, 0.72)
para(tf, "↻", size=20, bold=True, color=BLUE, first=True, line_spacing=0.9)
para(tf, "回到巡航", size=8.5, bold=True, color=BLUE, line_spacing=1.1)

facts = [
    ("停一次，多摘几个", "开到一处就尽量把够得着的都摘完，少让车反复启停"),
    ("不用地图也能对准", "不建地图、不算里程，靠「看着目标一点点挪」来对齐"),
    ("剪一次要 17 秒", "刀片走完一趟 60 度约 17 秒，合刀加张刀约 34 秒"),
]
x = MARGIN
for t, d in facts:
    rect(s, x, 5.42, 3.86, 1.02, fill=CARD, line=LINE, radius=0.05)
    tf = textbox(s, x + 0.22, 5.54, 3.42, 0.26)
    para(tf, t, size=10, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.22, 5.84, 3.42, 0.48)
    para(tf, d, size=8.3, color=SLATE, first=True, line_spacing=1.35)
    x += 4.04

notes(s, """【流程 60 秒 —— 全场核心】
这一页是最重要的：我们跑通的不是某一个单点功能，而是一条能一圈一圈转下去的完整流程。

上半行：车低速往前开，相机盯着前方；侧向相机抓到果子、确认是能摘的；立刻切断动力停稳，并锁住防止滑动；然后算出果子相对车身在哪、多远；手臂先移到预备位 —— 罩口停在果子前方 5 厘米待命。

下半行：以 20 度仰角、每次 1.5 厘米慢慢伸进去，这样果柄在上、果子在下，更容易套住；套到位后停住，等人确认 —— 这是我们刻意保留的唯一人工介入点，防止误剪、也防伤人；确认后刀片合拢 60 度剪断，果子顺着管子落进收集筐；最后退 8 厘米、抬起，回到待命姿势，接着去找下一颗。

这里有个必须坦诚的数字：剪一次大约 17 秒，合刀加张刀约 34 秒，所以「停车—摘一颗—继续走」一圈大约 60 到 90 秒。这是下一步要重点压缩的。""")

# ================================================================ 08 机械臂
s = new_slide()
header(s, "07 / 末梢执行", "最难的不是看见果，是伸进去套住它",
       "在枝叶中间把罩口对准一颗果，然后干净地剪断 —— 这一段的精度决定整台机器有没有用。")

items = [
    ("六轴小手臂", "结构照开源方案复刻，驱动和轨迹规划自己写，真机能按规划走", BLUE),
    ("相机和手臂对齐", "用 6 个已知点反推两者的相对位置；换一组点验证，误差 7.6 毫米", CYAN),
    ("伸进去的手法", "罩口内径 120 毫米，先停在果前 5 厘米，再以 20 度仰角每次进 1.5 厘米；对准的是罩口不是刀尖", GREEN),
    ("三道关卡", "每次动作前查：关节角度是否越界、算出的姿态偏差是否过大。超了就一动不动", ORANGE),
]
iy = 2.02
for name, desc, col in items:
    rect(s, MARGIN, iy, 7.02, 1.06, fill=WHITE, line=LINE, radius=0.05)
    strip = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(iy), Inches(0.05), Inches(1.06))
    strip.fill.solid(); strip.fill.fore_color.rgb = col
    strip.line.fill.background(); strip.shadow.inherit = False

    tf = textbox(s, MARGIN + 0.24, iy + 0.14, 2.30, 0.72, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, name, size=11.5, bold=True, color=INK, first=True, line_spacing=1.2)
    tf = textbox(s, MARGIN + 2.66, iy + 0.16, 4.14, 0.78, anchor=MSO_ANCHOR.MIDDLE)
    para(tf, desc, size=9, color=SLATE, first=True, line_spacing=1.4)
    iy += 1.14

rect(s, 7.94, 2.02, 4.78, 2.94, fill=CARD, line=LINE, radius=0.04)
picture_fit(s, MOVEIT, 8.04, 2.10, 4.58, 2.24)
tf = textbox(s, 8.06, 4.40, 4.54, 0.50)
para(tf, "电脑里先算一遍：手臂从待命位走到目标位，路径先仿真确认，再放到真机上走",
     size=7.5, color=SLATE, first=True, line_spacing=1.3)

tf = textbox(s, 7.94, 5.04, 4.78, 0.28)
para(tf, "误差是怎么一步步收上来的", size=10.5, bold=True, color=INK, first=True)

chain = [("相机看到的位置", "1 – 1.5 厘米"), ("相机和手臂对齐", "7.6 毫米"), ("手臂自己走点", "接近 0")]
cx = 7.94
for t, v in chain:
    rect(s, cx, 5.40, 1.50, 0.90, fill=CARD2, line=None, radius=0.06)
    tf = textbox(s, cx + 0.10, 5.50, 1.30, 0.28)
    para(tf, v, size=12, bold=True, color=BLUE, align=PP_ALIGN.CENTER, first=True, line_spacing=1.0)
    tf = textbox(s, cx + 0.10, 5.82, 1.30, 0.40)
    para(tf, t, size=7.5, color=SLATE, align=PP_ALIGN.CENTER, first=True, line_spacing=1.2)
    cx += 1.58

tf = textbox(s, MARGIN, 6.86, CONTENT_W, 0.26)
para(tf, "还有一层补偿：真实套果实验发现视觉估算有固定偏差（来自安装误差和手臂自重下垂），反算成偏移量写进配置参与瞄准。",
     size=8.3, italic=True, color=MUTED, first=True)

notes(s, """【机械臂 60 秒】
末梢执行是最容易翻车的地方 —— 看见果只是开始，能不能在枝叶中间套住它才是关键。

手臂是照开源方案复刻的六轴小手臂，驱动和轨迹规划是我们自己写的，能在真机上按规划走。

精度分三步收：相机直接看到的位置，误差 1 到 1.5 厘米；相机和手臂之间的对齐，用 6 个已知点反推、再换一组点验证，误差 7.6 毫米；手臂自己走点，误差接近 0。所以瓶颈不在手臂，在相机那一段。

伸进去的手法我们改过一次：一开始把刀尖送到果子中心，结果罩口还没到。改成以「罩口对准果心」为目标、20 度仰角从斜下方往上套，实测误差 3 毫米。

安全上每次动作前要过三道关卡：关节角度不能越界（大臂历史上断过一次，这条最严），算出的姿态偏差不能太大，超了就一动不动。

最后一个细节：真实套果和视觉估算之间有固定偏差，来自安装误差和手臂自重下垂，我们通过实验反算成偏移量写进配置里补偿。""")

# ================================================================ 09 工程挑战
s = new_slide(bg=RGBColor(0x0B, 0x12, 0x20))
header(s, "08 / 工程实战", "一个真实的坎：接口不够用",
       "这一页讲我们没做成什么、为什么没做成、以及因此换来了什么。",
       tag_color=RGBColor(0xFD, 0xBA, 0x74), title_color=WHITE, sub_color=RGBColor(0xCB, 0xD5, 0xE1))

steps = [
    ("现象", "车上接口太少", "两个相机、底盘、手臂，同时都要插上，板子上的口不够用", RGBColor(0xEF, 0x44, 0x44)),
    ("恶化", "一转接就掉线", "加了转接扩展坞后，相机一开图，整条接口链重新识别，设备全掉", RGBColor(0xF9, 0x73, 0x16)),
    ("查因", "带宽和供电都顶不住", "两路相机同时出图就把带宽吃满，扩展坞的供电也扛不住 —— 软件绕不过去", RGBColor(0xF5, 0x9E, 0x0B)),
    ("取舍", "保住核心，砍掉次要", "侧向相机直插板子；先砍掉语音模块，把带宽和算力全留给感知和运动", RGBColor(0x22, 0xC5, 0x5E)),
]
x = MARGIN
for tag, title, desc, col in steps:
    rect(s, x, 2.10, 2.92, 2.06, fill=RGBColor(0x13, 0x1C, 0x2E),
         line=RGBColor(0x2A, 0x3A, 0x55), radius=0.05)
    bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(2.10), Inches(2.92), Inches(0.05))
    bar.fill.solid(); bar.fill.fore_color.rgb = col
    bar.line.fill.background(); bar.shadow.inherit = False

    tf = textbox(s, x + 0.22, 2.26, 2.46, 0.26)
    para(tf, tag, size=9, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.22, 2.54, 2.46, 0.56)
    para(tf, title, size=12.5, bold=True, color=WHITE, first=True, line_spacing=1.2)
    tf = textbox(s, x + 0.22, 3.18, 2.46, 0.90)
    para(tf, desc, size=8.8, color=RGBColor(0xA8, 0xB6, 0xC8), first=True, line_spacing=1.45)
    x += 3.06

rect(s, MARGIN, 4.44, 5.96, 1.68, fill=RGBColor(0x0F, 0x2A, 0x22),
     line=RGBColor(0x1E, 0x5B, 0x46), radius=0.04)
tf = textbox(s, MARGIN + 0.26, 4.56, 5.4, 0.28)
para(tf, "换来了什么", size=10, bold=True, color=RGBColor(0x4A, 0xDE, 0x80), first=True)
tf = textbox(s, MARGIN + 0.26, 4.88, 5.42, 1.14)
para(tf, "• 两个相机能同时稳定出图，巡航和找果不用二选一", size=9.5,
     color=RGBColor(0xD1, 0xFA, 0xE5), first=True, line_spacing=1.4)
para(tf, "• 接口不再反复重连，底盘和手臂的指令一直在线", size=9.5,
     color=RGBColor(0xD1, 0xFA, 0xE5), line_spacing=1.4, space_before=3)
para(tf, "• 算力全留给识别和规划，认一张图稳定在 17 毫秒", size=9.5,
     color=RGBColor(0xD1, 0xFA, 0xE5), line_spacing=1.4, space_before=3)

rect(s, 6.88, 4.44, 5.84, 1.68, fill=RGBColor(0x2A, 0x14, 0x0C),
     line=RGBColor(0x5B, 0x2A, 0x18), radius=0.04)
tf = textbox(s, 7.14, 4.56, 5.3, 0.28)
para(tf, "代价和遗留", size=10, bold=True, color=RGBColor(0xFB, 0x92, 0x3C), first=True)
tf = textbox(s, 7.14, 4.88, 5.30, 1.14)
para(tf, "• 语音交互这一版没上，改由人工确认下刀来兜底", size=9.5,
     color=RGBColor(0xFE, 0xE7, 0xD5), first=True, line_spacing=1.4)
para(tf, "• 靠外接扩展坞不是长久办法，得做一块专门的底板把接口收拢", size=9.5,
     color=RGBColor(0xFE, 0xE7, 0xD5), line_spacing=1.4, space_before=3)
para(tf, "• 问题定位在硬件连接上，不是软件架构的毛病", size=9.5,
     color=RGBColor(0xFE, 0xE7, 0xD5), line_spacing=1.4, space_before=3)

tf = textbox(s, MARGIN, 6.34, CONTENT_W, 0.55)
para(tf, "我的判断：在算力和接口都有限的设备上，「知道自己该放弃什么」，比「什么都想加」更能决定项目能不能跑通。",
     size=11, bold=True, color=RGBColor(0xFD, 0xBA, 0x74), first=True)

notes(s, """【工程挑战 60 秒 —— 建议重点讲】

这一页我想说实话：我们有个东西没做成。

板子上的接口不够。我们要同时插两个相机、底盘、手臂，物理口子就不够。于是用了转接扩展坞，结果更糟 —— 相机一开图，整条接口链重新识别，相机和串口一起掉线。

排查之后确认，这是带宽加供电的双重硬约束，不是软件能绕过去的。所以做了取舍：侧向相机直插板子，然后砍掉语音模块 —— 语音要占一个接口，而它是所有功能里最不刚需的那个。

换来的是什么：两个相机能同时稳定出图，接口不再反复重连，算力全给识别，认一张图稳定 17 毫秒，整条采摘流程能连续跑起来。

代价是什么：语音这一版没有，改由人工确认下刀来兜底 —— 这其实也符合我们「下刀必须人工确认」的安全设计。

我的判断是：在边缘设备上，知道自己该放弃什么，比什么都想加更能决定项目能不能跑通。""")

# ================================================================ 10 实测
s = new_slide()
header(s, "09 / 实机验证", "这些都是在真车上跑出来的",
       "底盘能走、两个相机能同时出图、手臂能对准、果子能套住、整条流程能连着转。")

pics = [
    (PHOTO_TOP, "整车：窄体底盘 + 六轴手臂 + 3D 打印的漏斗罩口", 0.62, 2.02, 4.06, 2.10),
    (PHOTO_DETAIL, "末端：罩口内径 120 毫米，后面接收集软管，相机架在同一块板上", 4.86, 2.02, 4.06, 2.10),
    (QUAD, "两个相机同时出图：上面是前向，下面是侧向，彩色和距离都在", 9.10, 2.02, 3.62, 2.10),
]
for path, cap, px, py, pw, ph in pics:
    rect(s, px, py, pw, ph, fill=CARD, line=LINE, radius=0.03)
    picture_fit(s, path, px + 0.06, py + 0.06, pw - 0.12, ph - 0.12)
    tf = textbox(s, px, py + ph + 0.08, pw, 0.44)
    para(tf, cap, size=7.5, color=SLATE, first=True, line_spacing=1.3)

tf = textbox(s, MARGIN, 4.66, CONTENT_W, 0.30)
para(tf, "实测清单", size=13, bold=True, color=INK, first=True)

checks = [
    ("车能走", "三路信号真机验证；半秒收不到指令自动停；加速有上限，起步不窜", GREEN),
    ("两个相机", "前向和侧向同时出图，侧向直插板子，彩色加距离约每秒 15 帧", GREEN),
    ("手能对准", "相机和手臂对齐后换点验证，误差 7.6 毫米；手臂自己走点接近 0", GREEN),
    ("能套住果", "真果子套入实验，罩口对准果心差 3 毫米，角度几乎没偏", GREEN),
    ("流程能转", "巡航 → 发现 → 停车 → 定位 → 预备位，能一圈一圈连着执行", GREEN),
    ("语音没上", "受接口限制这一版没接，已列进下一阶段的硬件整改", ORANGE),
]
cy = 5.02
col_x = [MARGIN, MARGIN + 6.16]
for i, (name, desc, col) in enumerate(checks):
    cx = col_x[i % 2]
    yy = cy + (i // 2) * 0.62
    rect(s, cx, yy, 5.96, 0.54, fill=CARD, line=None, radius=0.06)
    circle(s, cx + 0.16, yy + 0.21, 0.11, fill=col)
    tf = textbox(s, cx + 0.38, yy + 0.15, 1.16, 0.26)
    para(tf, name, size=9.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, cx + 1.56, yy + 0.16, 4.28, 0.28)
    para(tf, desc, size=8.3, color=SLATE, first=True, line_spacing=1.2)

notes(s, """【实机验证 45 秒】
这一页放的都是真车。

左边整车：窄体底盘、四个大轮，中间是六轴手臂，末端是我们自己设计、3D 打印的漏斗罩口，后面接收集软管。
中间是末端特写：罩口内径 120 毫米，两个相机固定在同一块板子上。
右边是两个相机同时出图的画面：上面是前向的彩色和距离，下面是侧向的彩色和距离，四路同时在线 —— 这是接口问题解决之后的状态。

下面这张清单都是实测的：车能走，半秒没指令自动停；两个相机同时出图；相机和手臂对齐误差 7.6 毫米；真果子套入差 3 毫米；从巡航到预备位这条流程能连着执行。

唯一没做成的是语音，原因上一页已经说了。""")

# ================================================================ 11 价值
s = new_slide()
header(s, "10 / 价值", "从「能摘一颗」到「摘得起一片」",
       "单颗采摘是技术验证；真正决定它有没有意义的，是成本和能不能复制。")

vals = [
    ("省钱", "一块板顶一台工控机", "所有计算收在一块板子上，省掉工控机和外挂算力的硬件钱，也省电。", BLUE),
    ("好复制", "换个果园不用重新测绘", "不建地图、不算里程，换一片园子不用工程师重新采点，落地门槛低。", CYAN),
    ("有人用得上", "直面劳动力缺口", "把弯腰、重复、高强度的活交给机器，缓解采摘期招工难和人手老化。", GREEN),
]
x = MARGIN
for title, sub, desc, col in vals:
    rect(s, x, 2.02, 3.86, 2.10, fill=WHITE, line=LINE, radius=0.05)
    tf = textbox(s, x + 0.26, 2.20, 3.34, 0.28)
    para(tf, title, size=10, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.26, 2.52, 3.34, 0.42)
    para(tf, sub, size=13, bold=True, color=INK, first=True, line_spacing=1.2)
    tf = textbox(s, x + 0.26, 3.06, 3.34, 0.90)
    para(tf, desc, size=9.5, color=SLATE, first=True, line_spacing=1.5)
    x += 4.04

rect(s, MARGIN, 4.44, 6.00, 2.34, fill=CARD2, line=LINE, radius=0.04)
tf = textbox(s, MARGIN + 0.28, 4.60, 5.44, 0.30)
para(tf, "为什么选这块板", size=11, bold=True, color=BLUE, first=True)

points = [
    ("算得多", "一颗芯片上并行跑识别和规划，不用再外挂加速卡。"),
    ("吃得少", "功耗低，直接用车上的电池供，续航不牺牲。"),
    ("接得上", "操作系统和机器人框架都是现成的，现成工具直接能用，迁移成本低。"),
]
py = 4.96
for t, d in points:
    tf = textbox(s, MARGIN + 0.28, py, 1.00, 0.26)
    para(tf, t, size=9, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 1.34, py, 4.38, 0.54)
    para(tf, d, size=8.5, color=SLATE, first=True, line_spacing=1.4)
    py += 0.60

rect(s, 6.92, 4.44, 5.80, 2.34, fill=WHITE, line=LINE, radius=0.04)
tf = textbox(s, 7.18, 4.60, 5.28, 0.30)
para(tf, "从样机到产品，还差三步", size=11, bold=True, color=INK, first=True)

road = [
    ("01", "把接口收拢", "做一块专用底板，替掉扩展坞，把语音接回来"),
    ("02", "把速度提上去", "压缩剪切时间，一次停车多摘几个，缩短单果周期"),
    ("03", "把难活啃下来", "补果柄样本、加接触感知，提高复杂光照下的成功率"),
]
ry = 4.94
for num, t, d in road:
    tf = textbox(s, 7.18, ry, 0.46, 0.28)
    para(tf, num, size=10, bold=True, color=ORANGE, first=True, line_spacing=1.0)
    tf = textbox(s, 7.64, ry, 1.34, 0.28)
    para(tf, t, size=9, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, 9.00, ry, 3.46, 0.54)
    para(tf, d, size=8.3, color=SLATE, first=True, line_spacing=1.4)
    ry += 0.58

notes(s, """【价值 45 秒】
技术跑通之后，真正决定有没有意义的，是成本和能不能复制。

第一是省钱：所有计算收在一块板子上，省掉了工控机和外挂算力的硬件钱，也省电。对农业装备来说，成本压不压得下来，直接决定农户买不买。

第二是好复制：不建地图、不算里程，换一片果园不用工程师重新采点 —— 这点对推广特别关键，否则每到一个新园子都要出差一次。

第三是有人用得上：把弯腰、重复、高强度的活交给机器，缓解采摘期招工难和从业者老龄化。

为什么选这块高通板：算得多，一颗芯片并行跑识别和规划；吃得少，直接用车上的电池；接得上，操作系统和机器人框架都是现成的，迁移成本低。

从样机到产品还差三步：把接口收拢、把速度提上去、把难活啃下来。""")

# ================================================================ 12 总结
s = new_slide(bg=NAVY, footer=False)
ln = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, Inches(0.06))
ln.fill.solid(); ln.fill.fore_color.rgb = BLUE; ln.line.fill.background(); ln.shadow.inherit = False

tf = textbox(s, MARGIN, 0.82, CONTENT_W, 0.30)
para(tf, "11 / 总结", size=10.5, bold=True, color=RGBColor(0x60, 0xA5, 0xFA), first=True)

tf = textbox(s, MARGIN, 1.18, 8.6, 0.62)
para(tf, "一块芯片，一条闭环，一次真实的取舍", size=30, bold=True, color=WHITE,
     first=True, line_spacing=1.1)

summary = [
    ("做成了", "在一块车载边缘计算板上跑通「看见果 → 开车过去 → 手臂套住 → 剪下来 → 找下一颗」的完整流程。",
     RGBColor(0x4A, 0xDE, 0x80)),
    ("讲清楚了", "五层分工加三条死规矩，每一条都对应一次真实故障；两次主动不做，换来的是改得动、交得出去。",
     RGBColor(0x60, 0xA5, 0xFA)),
    ("认清了边界", "接口不够是本版真正的瓶颈，语音因此让位；问题定位在硬件连接，不是软件架构。",
     RGBColor(0xFD, 0xBA, 0x74)),
]
ry = 2.16
for tag, body, col in summary:
    rect(s, MARGIN, ry, 8.30, 1.16, fill=RGBColor(0x13, 0x1C, 0x2E),
         line=RGBColor(0x2A, 0x3A, 0x55), radius=0.05)
    tf = textbox(s, MARGIN + 0.26, ry + 0.16, 1.30, 0.28)
    para(tf, tag, size=10, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 1.62, ry + 0.14, 6.44, 0.90)
    para(tf, body, size=10, color=RGBColor(0xCB, 0xD5, 0xE1), first=True, line_spacing=1.5)
    ry += 1.26

tf = textbox(s, MARGIN, 6.02, 8.30, 0.30)
para(tf, "下一步：收拢接口 → 接回语音 → 补果柄和接触感知 → 多台协同", size=10,
     color=RGBColor(0xFD, 0xBA, 0x74), first=True)

picture_fit(s, PHOTO_TOP, 9.30, 2.16, 3.42, 3.90, border=RGBColor(0x2A, 0x3A, 0x55))

tf = textbox(s, 9.30, 6.18, 3.42, 0.60)
para(tf, "谢谢各位评委老师", size=15, bold=True, color=WHITE, first=True, line_spacing=1.1)
para(tf, "Q & A", size=10, color=RGBColor(0x7D, 0x8C, 0xA3), space_before=2)

notes(s, """【收尾 30 秒】
最后三句话。

第一，做成了：在一块车载边缘计算板上，跑通了「看见果、开过去、手臂套住、剪下来、再找下一颗」的完整流程。
第二，讲清楚了：五层分工加三条死规矩，每一条都来自真实故障；还有两次主动不做，换来的是改得动、交得出去。
第三，认清了边界：接口不够是本版真正的瓶颈，语音因此让位，但问题定位在硬件连接上，不是软件架构的毛病。

下一步四件事：收拢接口、接回语音、补果柄和接触感知、多台协同。

我的汇报到这里，谢谢各位评委老师，请提问。""")

prs.save(OUT)
print("OK ->", OUT)
print("slides:", len(prs.slides._sldIdLst))
