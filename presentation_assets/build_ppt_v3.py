#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PickOrangeDummy · 2026 高通智能边缘大赛 答辩 PPT 生成器（v3：专业工程版）
- 16:9 / 13.333 x 7.5 in
- 语言风格：专业严谨、逻辑因果透明、通俗易懂（兼具学术答辩与工业落地水准）
- 配图策略：真实引用 TF 坐标树、状态机转移图、双目深度流四分屏、YOLO 训练集检测特征图、电气接线图、实车工程照片
- 每页配备完整的答辩演讲备注（Speaker Notes）
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

# 精确对应的工程资产图片
IMG_CAR_TOP      = A("presentation_assets", "photo_jpg", "IMG_5588.jpg")
IMG_CAR_END      = A("presentation_assets", "photo_jpg", "IMG_5589.jpg")
IMG_CAR_SIDE     = A("presentation_assets", "photo_jpg", "IMG_5590.jpg")
IMG_DUAL_CAM     = A("presentation_assets", "video_frames", "v5_05s.jpg")
IMG_MOVEIT       = A("presentation_assets", "video_frames", "v3_10s.jpg")
IMG_TF_TREE      = A("docs", "diagrams", "04-tf.png")
IMG_MISSION_FSM  = A("docs", "diagrams", "05-mission-fsm.png")
IMG_POWER_WIRING = A("docs", "power_wiring.png")
IMG_YOLO_BATCH   = A("dummyarm_AImodel", "runs", "segment", "exp-002_yolo11n-seg_50e", "train_batch0.jpg")
IMG_CITRUS_SCENE = A("dummyarm_AImodel", "runs", "segment", "exp-002_yolo11n-seg_50e", "demo_predict", "pred", "d435i_s01_1764980224931.jpg")

OUT = "/Users/bohn/04_Project/PickOrangeDummy_QualcommEdge_答辩汇报.pptx"

# 工业级高通蓝科技风设计系统
NAVY   = RGBColor(0x0B, 0x12, 0x20)  # 深邃科技蓝黑
INK    = RGBColor(0x0F, 0x17, 0x2A)  # 主文本色
SLATE  = RGBColor(0x47, 0x55, 0x69)  # 次要说明色
MUTED  = RGBColor(0x94, 0xA3, 0xB8)  # 辅助/弱化色
LINE   = RGBColor(0xEA, 0xEE, 0xF4)  # 边框浅灰色
CARD   = RGBColor(0xF8, 0xFA, 0xFC)  # 卡片底色
CARD2  = RGBColor(0xEE, 0xF4, 0xFB)  # 重点高亮底色
BLUE   = RGBColor(0x00, 0x66, 0xCC)  # 高通蓝 / 科技蓝
CYAN   = RGBColor(0x02, 0x84, 0xC7)  # 动作青色
ORANGE = RGBColor(0xEA, 0x58, 0x0C)  # 柑橘橙
AMBER  = RGBColor(0xD9, 0x77, 0x06)  # 警告琥珀色
GREEN  = RGBColor(0x05, 0x96, 0x69)  # 安全翡翠绿
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
    if not os.path.exists(path):
        return None
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
        pic.line.width = Pt(1.0)
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
           sub_color=SLATE, y=0.50):
    bar = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(y + 0.07),
                                 Inches(0.055), Inches(0.28))
    bar.fill.solid()
    bar.fill.fore_color.rgb = tag_color
    bar.line.fill.background()
    bar.shadow.inherit = False

    tf = textbox(slide, MARGIN + 0.18, y, CONTENT_W - 0.2, 0.25)
    para(tf, tag, size=10, bold=True, color=tag_color, first=True)

    tf = textbox(slide, MARGIN, y + 0.28, CONTENT_W, 0.48)
    para(tf, title, size=23, bold=True, color=title_color, first=True, line_spacing=1.0)

    if subtitle:
        tf = textbox(slide, MARGIN, y + 0.82, CONTENT_W, 0.36)
        para(tf, subtitle, size=11.5, color=sub_color, first=True, line_spacing=1.25)


def notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


def metric_card(slide, x, y, w, h, value, unit, label, sublabel=None, color=BLUE):
    rect(slide, x, y, w, h, fill=CARD, line=LINE, radius=0.06)
    tf = textbox(slide, x + 0.14, y + 0.12, w - 0.28, 0.52)
    parts = [(value, True, color, 24)]
    if unit:
        parts.append((unit, True, color, 11))
    rich(tf, parts, first=True, line_spacing=1.0)
    tf = textbox(slide, x + 0.14, y + 0.64, w - 0.28, h - 0.72)
    para(tf, label, size=9.5, bold=True, color=INK, first=True, line_spacing=1.15)
    if sublabel:
        para(tf, sublabel, size=8, color=SLATE, line_spacing=1.1, space_before=2)


# ================================================================ P01: 封面
s = new_slide(bg=NAVY, footer=False)
ln = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, Inches(0.06))
ln.fill.solid(); ln.fill.fore_color.rgb = BLUE; ln.line.fill.background(); ln.shadow.inherit = False

picture_fit(s, IMG_CAR_END, 7.40, 0.60, 5.31, 6.30, border=RGBColor(0x1E, 0x29, 0x3B))

tf = textbox(s, MARGIN, 0.76, 6.5, 0.3)
para(tf, "2026 高通智能边缘大赛  ·  参赛答辩", size=11, bold=True,
     color=RGBColor(0x60, 0xA5, 0xFA), first=True)

tf = textbox(s, MARGIN, 1.25, 6.6, 1.9)
para(tf, "基于高通智能边缘平台的", size=26, bold=True, color=WHITE, first=True, line_spacing=1.2)
para(tf, "低位柑橘采摘机器人", size=38, bold=True, color=WHITE, line_spacing=1.15)

bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(3.26),
                         Inches(1.05), Inches(0.05))
bar.fill.solid(); bar.fill.fore_color.rgb = ORANGE; bar.line.fill.background(); bar.shadow.inherit = False

tf = textbox(s, MARGIN, 3.52, 6.5, 1.1)
para(tf, "PickOrangeDummy", size=15, bold=True, color=RGBColor(0xFD, 0xBA, 0x74), first=True)
para(tf, "端侧感知协同 · 超声波近场补盲 · 六轴机械臂全自主作业闭环",
     size=13, bold=True, color=RGBColor(0xE2, 0xE8, 0xF0), line_spacing=1.35, space_before=4)
para(tf, "单颗高通边缘 SoC 完整承载机器视觉、轨迹规划与实时运动控制",
     size=10.5, color=RGBColor(0x94, 0xA3, 0xB8), line_spacing=1.35, space_before=3)

# 核心硬件规格标识
items = [("40 TOPS", "Hexagon NPU"), ("8 核", "Kryo CPU"), ("16 GB", "LPDDR5 内存"), ("100%", "端侧本地闭环")]
bx = MARGIN
for v, l in items:
    rect(s, bx, 5.08, 1.52, 0.88, fill=RGBColor(0x16, 0x21, 0x38),
         line=RGBColor(0x2A, 0x3A, 0x55), radius=0.08)
    tf = textbox(s, bx + 0.10, 5.18, 1.32, 0.32)
    para(tf, v, size=15, bold=True, color=WHITE, first=True, line_spacing=1.0)
    tf = textbox(s, bx + 0.10, 5.52, 1.32, 0.26)
    para(tf, l, size=8, color=RGBColor(0x94, 0xA3, 0xB8), first=True, line_spacing=1.0)
    bx += 1.64

tf = textbox(s, MARGIN, 6.36, 6.5, 0.6)
para(tf, "硬件基准：Arduino VENTUNO Q  ·  系统环境：Ubuntu 24.04 (Noble) + ROS 2 Jazzy",
     size=9.5, color=RGBColor(0x7D, 0x8C, 0xA3), first=True)
para(tf, "参赛团队：__________        指导教师：__________        2026.11", size=9.5,
     color=RGBColor(0x7D, 0x8C, 0xA3), space_before=4)

notes(s, """【封面页答辩引言 - 30秒】
各位评委老师好！我们汇报的项目是《基于高通智能边缘平台的低位柑橘采摘机器人 —— PickOrangeDummy》。
本项目聚焦于南方丘陵果园低矮密植柑橘的自动化采摘难题。最大的技术特色在于：我们将双深度相机空间融合、端侧实例分割、高精度六轴机械臂轨迹规划以及底盘防滑伺服逼近，全部部署并闭环运行在一台搭载高通 Kryo 8核处理器与 40 TOPS Hexagon NPU 的单板计算机（Arduino VENTUNO Q）上。
全系统无需外挂工控机与云端协同，实现了完全自主的端侧感知与移动作业全流程。接下来我将从架构、功能、工程实测与价值四个维度向各位评委做详细汇报。""")

# ================================================================ P02: 痛点与破局
s = new_slide()
header(s, "01 / 项目定位与背景", "丘陵柑橘密植采摘：卡在农田作业「最后一米」",
       "传统大型农机受限于树冠与窄行进不去，而外挂工业PC方案笨重昂贵 —— 农业急需真正低成本、高灵活的端侧智能。")

# 左侧：三大行业矛盾
cards = [
    ("季节性农时窗口极紧迫", "我国柑橘成熟期高度重叠，最佳鲜果采摘窗口仅 60~90 天。传统采收劳动力短缺导致大量落果烂果，直接侵蚀果农全年收益。",
     "痛点：用工荒与采摘时效矛盾", ORANGE),
    ("采摘人工成本吞噬利润", "鲜果采摘完全依赖人工弯腰作业，人工支出已占到柑橘生产总成本的 30% 以上，且农村从业者平均年龄已超 55 岁。",
     "痛点：劳动力老龄化严重", ORANGE),
    ("传统机电方案不适应地形", "南方丘陵果树低矮分枝多、行距狭窄。大型履带机无法穿行；而工业级 PC 方案功耗高达数百瓦且怕振动，无法轻量化落地。",
     "痛点：工控机体积大/功耗高", ORANGE),
]
y = 1.95
for title, body, tag, col in cards:
    rect(s, MARGIN, y, 6.20, 1.35, fill=WHITE, line=LINE, radius=0.06)
    bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(y), Inches(0.05), Inches(1.35))
    bar.fill.solid(); bar.fill.fore_color.rgb = col; bar.line.fill.background(); bar.shadow.inherit = False

    tf = textbox(s, MARGIN + 0.20, y + 0.12, 5.80, 0.28)
    para(tf, title, size=12, bold=True, color=INK, first=True, line_spacing=1.1)
    tf = textbox(s, MARGIN + 0.20, y + 0.44, 5.80, 0.56)
    para(tf, body, size=9.5, color=SLATE, first=True, line_spacing=1.35)
    tf = textbox(s, MARGIN + 0.20, y + 1.05, 5.80, 0.22)
    para(tf, "▸ " + tag, size=8.5, bold=True, color=col, first=True)
    y += 1.45

# 右侧：真实果园环境图与破局解法
rect(s, 7.10, 1.95, 5.61, 3.40, fill=CARD, line=LINE, radius=0.05)
picture_fit(s, IMG_CITRUS_SCENE, 7.20, 2.05, 5.41, 3.20)
tf = textbox(s, 7.10, 5.40, 5.61, 0.40)
para(tf, "实地作业场景：低矮分枝密集、光照剧烈变化、果实与枝叶交错遮挡，要求轻量机体与高鲁棒性感知",
     size=8, color=SLATE, first=True, line_spacing=1.25)

# 底部破局思路
rect(s, MARGIN, 5.95, CONTENT_W, 0.95, fill=CARD2, line=BLUE, radius=0.06)
tf = textbox(s, MARGIN + 0.25, 6.05, 2.0, 0.75, anchor=MSO_ANCHOR.MIDDLE)
para(tf, "破局核心思路", size=13, bold=True, color=BLUE, first=True)
para(tf, "单芯片实现移动采摘", size=9, color=SLATE, space_before=2)

tf = textbox(s, MARGIN + 2.30, 6.05, CONTENT_W - 2.55, 0.75, anchor=MSO_ANCHOR.MIDDLE)
para(tf, "以高能效比高通边缘单板为计算枢纽，抛弃笨重工控机与冗余建图，采用「窄体阿克曼底盘 + 反应式视觉逼近 + 仿生六轴机械臂」，实现毫米级低位自主采摘。",
     size=10.5, color=INK, first=True, line_spacing=1.35)

notes(s, """【项目定位与痛点 - 45秒】
首先向评委阐明本项目解决的核心农业痛点。
南方柑橘是我国种植面积最大的优势水果产业，但采收环节面临极具挑战的瓶颈：第一，果实成熟期极短（集中在深秋两三个月），季节性劳动力严重紧缺；第二，农村劳动力结构性老龄化，采摘人工成本逐年攀升，已吞噬三成利润；第三，丘陵山地普遍采用矮化密植栽培，大型采摘机根本钻不进狭窄行间，而传统工业PC方案功耗高达几百瓦，不仅怕强振动且成本居高不下。
因此，我们避开了“重载大农机”路线，利用高通智能边缘单板的高算力密度与低功耗特性，打造了一款小巧窄体、能灵活穿行于果树低位分枝之间的全自主轻量化采摘机器人，打通农业自动化的“最后一米”。""")

# ================================================================ P03: 系统软硬件架构全景
s = new_slide()
header(s, "02 / 系统架构设计", "单 SoC 承载的五层分层软件体系与硬件拓扑",
       "严格自顶向下的单向依赖设计，核心原则：执行器单一写者独占、高层决策零阻塞、硬安全逻辑下沉至 MCU 实时层。")

# 左侧：五层分层体系
layers = [
    ("L4", "交互与监督层", "人工确认剪切闸门 · 离线系统状态监控 · 锁存急停", RGBColor(0x7C, 0x3A, 0xED)),
    ("L3", "任务编排层", "Tick 驱动扁平有限状态机 (FSM) · 巡航/找果/进刀/采摘状态流转", BLUE),
    ("L2", "动作能力层", "ROS 2 Actions 动作服务 (arm_move / cutter / chassis_drive)", CYAN),
    ("L1", "设备抽象层", "ros2_control C++ 硬件接口 · Astra / D435i 设备节点 · 底盘 RPC 桥接", RGBColor(0x0D, 0x94, 0x88)),
    ("L0", "硬件实时层", "STM32H5 500ms PWM 看门狗 · REF 机械臂固件限位 · 电气物理保险", SLATE),
]
y = 1.95
for tag, name, desc, col in layers:
    rect(s, MARGIN, y, 6.10, 0.78, fill=CARD, line=LINE, radius=0.06)
    badge = rect(s, MARGIN + 0.12, y + 0.15, 0.60, 0.46, fill=col, line=None, radius=0.10)
    tf = badge.text_frame; tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    para(tf, tag, size=11, bold=True, color=WHITE, align=PP_ALIGN.CENTER, first=True)

    tf = textbox(s, MARGIN + 0.82, y + 0.12, 1.80, 0.28)
    para(tf, name, size=11, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 0.82, y + 0.42, 5.15, 0.28)
    para(tf, desc, size=8.5, color=SLATE, first=True, line_spacing=1.1)
    y += 0.85

tf = textbox(s, MARGIN, y + 0.05, 6.10, 0.3)
para(tf, "★ 架构约束：上层绝不越级直接操作串口，下层节点崩溃不阻塞系统全局编排",
     size=8.5, bold=True, color=BLUE, first=True)

# 右侧：TF 空间坐标树
rect(s, 7.00, 1.95, 5.71, 4.45, fill=WHITE, line=LINE, radius=0.05)
tf = textbox(s, 7.15, 2.05, 5.40, 0.26)
para(tf, "整车空间 TF 坐标拓扑树（Transforms.yaml 已实机验证）", size=10, bold=True, color=BLUE, first=True)

picture_fit(s, IMG_TF_TREE, 7.10, 2.38, 5.51, 2.70)

tf = textbox(s, 7.15, 5.15, 5.40, 1.15)
para(tf, "• 空间基准：以底盘后轴中心地面投影 base_footprint 为根，统一整车运动学模型；", size=8.3, color=SLATE, first=True, line_spacing=1.2)
para(tf, "• 机械臂与双目刚体解耦：机械臂 base_link、前向 astra_link、左侧 d435i_link 具备严格几何测量外参；", size=8.3, color=SLATE, line_spacing=1.2, space_before=2)
para(tf, "• 确定性坐标流：消除动态漂移，视觉感知目标经 TF 矩阵直接解析为机械臂底座三维坐标 (X, Y, Z)。", size=8.3, color=SLATE, line_spacing=1.2, space_before=2)

notes(s, """【系统软硬件架构全景 - 60秒】
在系统架构设计上，评委可以看到我们极其清晰的五层分层软件体系：
自顶向下分为交互与监督层（L4）、任务编排层（L3）、动作能力层（L2）、设备抽象层（L1）以及底层的 MCU 固件实时层（L0）。
这套软件体系确立了三条不妥协的工程铁律：
第一是“单一写者原则（Single Writer）”：每一个物理执行器（如底盘驱动或机械臂串口）有且仅有一个专用写节点，彻底避免多进程抢占串口导致的通信死锁与不可预测抖动；
第二是“决策层零阻塞”：任务编排层不允许任何 sleep、subprocess 或阻塞式IO，全部通过异步 ROS Action 状态机驱动；
第三是“最底层安全最硬”：即使 Linux 系统因为极端负载偶发挂起，最底层的 STM32 固件看门狗也能在 500ms 内主动将底盘回中切断动力，机械臂硬件限位直接锁死，保证设备绝不会失控撞树。
右图展示了我们整车经过实车测量的 TF 坐标系拓扑树，建立了以底盘为基准到机械臂法兰以及双相机光学中心的空间严格映射。""")

# ================================================================ P04: 异构多传感器融合
s = new_slide()
header(s, "03 / 异构感知系统", "双深度相机与超声波异构融合：构建 0~3.5m 连续无盲区防线",
       "不同传感器各司其职、互补覆盖：大范围巡航避障、近距高精度果实捕捉与贴脸毫米级安全兜底。")

# 三大传感器卡片
sensors = [
    ("Orbbec Astra Pro Plus", "正前向 · 结构光深度", "行进导航巡航与前方障碍大范围扫描。\n视场宽阔，负责 0.8m~3.5m 障碍提前减速判定。",
     "特性：近距 <0.6m 存在测距盲区", RGBColor(0x7C, 0x2D, 0x12)),
    ("Intel RealSense D435i", "朝左 100° · 主动双目", "专为侧方果树采摘窗口服务。\n15Hz 稳定输出对齐的彩色图与高密度空间点云，输出目标 3D 坐标。",
     "特性：视场集中于作业面，不看正前方", BLUE),
    ("HC-SR04 超声波阵列", "前向补盲 · 硬件直连", "直连 STM32 控制器，不抢占主机 USB 带宽。\n超近距离声呐回波测距，静止 1.5s 自动断脉冲防抖动。",
     "特性：填补 <0.6m 盲区，固件级紧急刹停", GREEN),
]
x = MARGIN
for name, role, desc, limit, col in sensors:
    rect(s, x, 1.95, 2.62, 2.25, fill=WHITE, line=LINE, radius=0.06)
    tf = textbox(s, x + 0.16, 2.08, 2.30, 0.32)
    para(tf, name, size=11, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.16, 2.42, 2.30, 0.24)
    para(tf, role, size=8.5, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.16, 2.72, 2.30, 0.85)
    para(tf, desc, size=8.8, color=SLATE, first=True, line_spacing=1.35)
    tf = textbox(s, x + 0.16, 3.70, 2.30, 0.42)
    para(tf, "▸ " + limit, size=8, bold=True, color=ORANGE, first=True, line_spacing=1.2)
    x += 2.72

# 右侧四分屏
rect(s, 8.62, 1.95, 4.10, 2.50, fill=CARD, line=LINE, radius=0.04)
picture_fit(s, IMG_DUAL_CAM, 8.70, 2.02, 3.94, 1.92)
tf = textbox(s, 8.70, 3.98, 3.94, 0.42)
para(tf, "实机四分屏数据流：上排 Astra 前向彩色与深度，下排 D435i 侧向彩色与深度，双路 USB 并发稳定运行",
     size=7.2, color=SLATE, first=True, line_spacing=1.2)

# 底部连续防线
tf = textbox(s, MARGIN, 4.45, CONTENT_W, 0.28)
para(tf, "传感器纵深防御链：从远距识别到接触级防撞的完整闭环", size=11.5, bold=True, color=INK, first=True)

defense = [
    ("3.5m – 0.8m", "Astra 远距感知", "前方障碍物识别，若有效深度骤降则触发小车减速", BLUE),
    ("0.8m – 0.6m", "阈值外推保护", "视觉进入近界盲区前，将刹车判定阈值前移，提前停稳", CYAN),
    ("0.6m – 0.0m", "超声波贴脸补盲", "MCU 固件直接读取超声波测距，达到硬安全阈值毫秒级刹停", GREEN),
    ("失效保护", "500ms 看门狗", "若上位机进程崩溃无心跳租约，底盘自锁滑行回中", ORANGE),
]
x = MARGIN
for rng, who, how, col in defense:
    rect(s, x, 4.85, 2.94, 1.45, fill=CARD, line=LINE, radius=0.06)
    tf = textbox(s, x + 0.16, 4.98, 2.62, 0.28)
    para(tf, rng, size=13, bold=True, color=col, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.16, 5.30, 2.62, 0.25)
    para(tf, who, size=9.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, x + 0.16, 5.60, 2.62, 0.62)
    para(tf, how, size=8.3, color=SLATE, first=True, line_spacing=1.3)
    x += 3.08

notes(s, """【异构多传感器融合 - 60秒】
在野外复杂农田环境中，单一传感器必然存在物理盲区。我们的感知方案不是盲目堆砌硬件，而是建立“多维互补”的纵深防御：
首先，正前方搭载 Astra Pro Plus 结构光相机，主要负责低速巡航中的前方路况与中远距离障碍探测；但该相机在 0.6m 以内存在结构光零反射的物理盲区——在机器人最该停车的临界面障碍物会“突然消失”；
因此，我们在正前方加装了直连底盘 STM32 的超声波传感器，其测距不经过 Linux 内核与 ROS 协议栈，直接由单片机中断捕获，专职填补 0.6m 以内的贴身防撞；
与此同时，小车左侧倾斜 100° 安装了 RealSense D435i 主动双目相机，专门正对侧方的柑橘树冠，以 15Hz 的稳定频率输出高分辨率彩色与高精度深度图；
评委可以看右边的实测四分屏截图：上排前向、下排侧向，双目四路视频流全部在 VENTUNO Q 单板上同时并发解算，构建了从 3.5m 到 0cm 的全链路安全距离防护。""")

# ================================================================ P05: 端侧定制视觉模型
s = new_slide()
header(s, "04 / 端侧视觉模型", "面向果园作业定制的 YOLO11n-seg 实例分割与双轨冗余",
       "不盲目追求参数规模，追求边缘实时性与树枝避障能力：单帧 17.1 ms 推理，兼顾果实识别与防撞刚性边界。")

metric_card(s, MARGIN, 1.95, 2.28, 1.35, "86.8", "%", "目标果实 Mask mAP@0.5", "核心采摘目标精准贴合果形", BLUE)
metric_card(s, MARGIN + 2.40, 1.95, 2.28, 1.35, "93.2", "%", "果实识别 Precision", "极低误检率，杜绝空挥机械臂", CYAN)
metric_card(s, MARGIN + 4.80, 1.95, 2.28, 1.35, "17.1", "ms", "单帧推理耗时 (≈58 FPS)", "完全满足移动作业实时闭环", GREEN)
metric_card(s, MARGIN + 7.20, 1.95, 2.28, 1.35, "2.84", "M", "模型参数量 (10.4 GFLOPs)", "高度契合高通端侧 NPU/CPU 部署", ORANGE)
metric_card(s, MARGIN + 9.60, 1.95, 2.28, 1.42, "4", "类", "四分类作业语义标签", "果实 · 遮挡果 · 果梗 · 树干避障", SLATE)

# 左侧：类别语义与决策
tf = textbox(s, MARGIN, 3.55, 6.10, 0.28)
para(tf, "四类目标精准分割与对应作业动作决策", size=12, bold=True, color=INK, first=True)

classes = [
    ("orange（完整果实）", "主采摘目标", "提取掩码形心与对应深度，直接解算为机械臂基座空间三维坐标 (X, Y, Z)", GREEN, "86.8%"),
    ("occluded_orange（遮挡果）", "避障排除", "遮挡面积 >50% 的果实判为阻滞目标，暂不采摘，防止漏斗硬挤树枝", AMBER, "57.7%"),
    ("tree（树干与粗枝）", "刚性避障", "模型第二代核心改进：机械臂规划空间中将树干划定为禁行膨胀区", BLUE, "已加入"),
    ("pedicel（果梗）", "精细定位", "果剪末端作业参考点；因样本较少已规划在下一阶段特写数据集重点增强", ORANGE, "持续增强"),
]
y = 3.90
for name, tag, desc, col, score in classes:
    rect(s, MARGIN, y, 6.10, 0.58, fill=CARD, line=LINE, radius=0.06)
    circle(s, MARGIN + 0.16, y + 0.23, 0.10, fill=col)
    tf = textbox(s, MARGIN + 0.36, y + 0.08, 2.20, 0.24)
    para(tf, name, size=9.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 0.36, y + 0.32, 2.20, 0.20)
    para(tf, tag, size=7.8, color=MUTED, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 2.45, y + 0.14, 2.90, 0.34)
    para(tf, desc, size=8.3, color=SLATE, first=True, line_spacing=1.2)
    tf = textbox(s, MARGIN + 5.35, y + 0.14, 0.65, 0.30)
    para(tf, score, size=8.5, bold=True, color=col, align=PP_ALIGN.RIGHT, first=True)
    y += 0.62

# 右侧：真实训练集样本切片与双轨冗余
rect(s, 7.10, 3.55, 5.61, 2.50, fill=WHITE, line=LINE, radius=0.05)
picture_fit(s, IMG_YOLO_BATCH, 7.20, 3.65, 5.41, 2.30)

tf = textbox(s, MARGIN, 6.55, CONTENT_W, 0.26)
para(tf, "★ 工程冗余策略：真机运行优先调用 YOLO11n-seg 实例分割；若边缘计算偶发丢帧或未达置信度，系统自动无缝降级为 HSV 色度 + 空间深度滤波算法保底，杜绝采摘流程空转停摆。",
     size=8.3, bold=True, color=BLUE, first=True)

notes(s, """【端侧定制视觉模型 - 60秒】
在视觉算法层面，我们面向高通端侧硬件深度定制了 YOLO11n-seg 轻量化实例分割网络。
为什么选纳米级模型？因为该模型参数量仅 2.84M，在端侧单帧推理仅需 17.1 ms，折合每秒超 58 帧，为机械臂闭环留出了充沛的 CPU/NPU 算力余量。
在模型演进中，我们经历了至关重要的工程认知迭代：
第一版我们只标注了果实，主类精度高达 86.8%，但在真机实测中发现机械臂进刀时经常剐蹭树干；
因此在第二版中，我们引入了关键的 tree（树干粗枝）分类与 occluded_orange（遮挡果）分类：宁可让整体全品类 mAP 略微下降，也坚决把“防撞刚性边界”融入模型语义，从源头避免进刀碰壁。
右图展示了我们真实数据集标注与训练的增强样本切片。同时我们在工程上做了双轨冗余：真机优先使用深度学习分割，若目标被泥污遮蔽导致置信度不足，系统自动降级为 HSV 色度结合深度滤波的传统视觉保底，确保作业链路绝不停滞。""")

# ================================================================ P06: 端到端作业闭环
s = new_slide()
header(s, "05 / 端到端闭环", "从自主巡航到精准采摘的九步完整状态机流转",
       "现场验证的并非割裂的演示 Demo，而是一条支持连续循环、带安全锁存与人工防误剪机制的完整作业链。")

# 流程九步两行 Chevrons
row1 = [("1. 巡航探果", "低速 20-25% 前进\nAstra+超声波前向守卫"),
        ("2. 发现目标", "侧向 D435i 抓帧\nYOLO/HSV 分割目标"),
        ("3. 停车锁存", "切断底盘 PWM 动力\n软件锁存防止滑行"),
        ("4. 空间定位", "深度图 + 手眼外参\n解算 base_link 坐标"),
        ("5. 移至预备位", "罩口逼近至果前 5cm\nIK 残差与限位校验")]
row2 = [("6. 仰角进刀", "20° 仰角沿轴微进\n漏斗包裹果实防划伤"),
        ("7. 等待确认", "★ 人机在环安全闸门\n人工指令确认下刀"),
        ("8. 刀片剪切", "J6 内嵌刀片旋转 60°\n果梗切断落入导流软管"),
        ("9. 退刀复位", "沿轴退 8cm 并抬升\n机械臂归位，解除锁存")]

def draw_flow_row(items, y, start_x, w, h, colors):
    x = start_x
    for i, (title, desc) in enumerate(items):
        col = colors[i % len(colors)]
        sh = s.shapes.add_shape(MSO_SHAPE.CHEVRON, Inches(x), Inches(y), Inches(w), Inches(h))
        sh.fill.solid(); sh.fill.fore_color.rgb = col
        sh.line.fill.background(); sh.shadow.inherit = False
        tf = sh.text_frame; tf.word_wrap = True; tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        tf.margin_left = Inches(0.28); tf.margin_right = Inches(0.04)
        para(tf, title, size=10, bold=True, color=WHITE, align=PP_ALIGN.CENTER, first=True)
        tf2 = textbox(s, x + 0.30, y + h + 0.10, w - 0.50, 0.55)
        para(tf2, desc, size=7.8, color=SLATE, align=PP_ALIGN.CENTER, first=True, line_spacing=1.2)
        x += w - 0.28

draw_flow_row(row1, 1.95, MARGIN, 2.62, 0.78,
              [BLUE, RGBColor(0x25, 0x63, 0xEB), CYAN, RGBColor(0x0D, 0x94, 0x88), GREEN])
draw_flow_row(row2, 3.48, MARGIN + 0.60, 2.62, 0.78,
              [GREEN, ORANGE, RGBColor(0xD9, 0x77, 0x06), SLATE])

tf = textbox(s, 11.00, 3.55, 1.70, 0.65)
para(tf, "↻", size=20, bold=True, color=BLUE, first=True, line_spacing=0.9)
para(tf, "进入下一轮巡航", size=8.5, bold=True, color=BLUE, line_spacing=1.1)

# 下半部分：真实任务状态机架构图
rect(s, MARGIN, 4.88, CONTENT_W, 1.95, fill=WHITE, line=LINE, radius=0.05)
tf = textbox(s, MARGIN + 0.20, 4.96, 6.0, 0.24)
para(tf, "整机底层有限状态机转移图（Mission FSM 源码架构）", size=10, bold=True, color=BLUE, first=True)

picture_fit(s, IMG_MISSION_FSM, MARGIN + 0.10, 5.22, CONTENT_W - 0.20, 1.55)

notes(s, """【端到端作业闭环 - 60秒】
这一页是全场答辩的核心成果体现：我们打通的不是割裂的功能碎片，而是一套可连续自主运转的完整采摘状态机闭环。
评委可以看到上下两组流转：
第一步，小车以 20%~25% 的平稳慢速在果垄间自主巡航，Astra 与超声波坚守前向安全；
第二步，侧向 D435i 在视野甜区内捕捉到柑橘目标，通过 YOLO 实例分割提取果心像素点与深度；
第三步，系统切断动力停稳，底盘进入锁存状态，防止机械臂大幅展开时的重心偏移引发小车蠕动；
第四步，调用 SVD 手眼外参矩阵，直接反解出果实在机械臂底座坐标系下的物理位置；
第五步，机械臂先规划到达“预备位”——让末端漏斗罩口停在果心正前方 5cm 处，同时进行逆运动学（IK）残差与关节极限预校验；
第六步，机械臂以 20° 仰角沿轴线微步进刀，漏斗口平滑套入果实，避免果皮划伤；
第七步，设立“人机在环确认闸门”——现场调试中必须收到人工确认信号后方可切入剪切，确保人机与树木绝对安全；
第八步，末端 J6 轴驱动内嵌刀片旋转 60° 干净切断果梗，果实顺着导流软管落入收集筐；
第九步，机械臂退刀抬升、收缩至 REST 待命姿态，底盘解除锁存，继续向前巡航寻找下一个目标。
下方展示了对应的状态机真实源码拓扑图，支持 ESTOP 随时中断与故障安全恢复。""")

# ================================================================ P07: 六轴臂控制与精度
s = new_slide()
header(s, "06 / 机械臂控制", "超紧凑六轴臂与柔性末端执行器：把对准误差压至毫米级",
       "结构基于 peng-zhihui Dummy-Robot 深度复刻演进，自研 ros2_control C++ 插件与 MoveIt 2 真机闭环。")

# 左侧：技术规格点
items = [
    ("超紧凑六轴机构与 ros2_control", "C++ 编写的 Hardware SystemInterface 插件，直连 STM32F405 REF 主控；深度融合 MoveIt 2 运动学规划算法，实现平滑防抖轨迹生成与 J6 驱动层隔离。", BLUE),
    ("柔性漏斗执行器几何创新", "采用 120mm 内径聚拢罩口 + 125mm 导流锥体一体化 3D 打印结构；目标设为「罩口对心」而非单纯刀尖接触，实测套入对准误差 ≤ 3 mm。", CYAN),
    ("六点 SVD 空间手眼标定", "基于 ArUco 标记与 PnP 求解外参初值，结合 6 个预设空间位姿采集 SVD 奇异值分解精配准；留出法实测验证误差仅 7.6 mm，机械臂单点定位精度达毫米级。", GREEN),
    ("关键关节四道硬安全闸", "每次运动前实时阻断校验：大臂 J3 ≥ 30°（严防历史断臂风险）、J5 ≤ 80°、J2 ≤ 65°（严防折叠挤压），逆解残差超限即刻锁死不动。", ORANGE),
]
y = 1.95
for name, desc, col in items:
    rect(s, MARGIN, y, 6.70, 0.98, fill=WHITE, line=LINE, radius=0.05)
    strip = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(MARGIN), Inches(y), Inches(0.05), Inches(0.98))
    strip.fill.solid(); strip.fill.fore_color.rgb = col; strip.line.fill.background(); strip.shadow.inherit = False

    tf = textbox(s, MARGIN + 0.18, y + 0.10, 6.40, 0.26)
    para(tf, name, size=11, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, MARGIN + 0.18, y + 0.38, 6.40, 0.54)
    para(tf, desc, size=8.5, color=SLATE, first=True, line_spacing=1.35)
    y += 1.05

# 右侧：MoveIt 仿真截图与精度链
rect(s, 7.60, 1.95, 5.11, 2.90, fill=CARD, line=LINE, radius=0.04)
picture_fit(s, IMG_MOVEIT, 7.70, 2.05, 4.91, 2.25)
tf = textbox(s, 7.70, 4.35, 4.91, 0.45)
para(tf, "MoveIt 2 数字样机真机闭环验证：实时环境碰撞检测与带约束 Cartesian 直线轨迹规划",
     size=7.5, color=SLATE, first=True, line_spacing=1.2)

# 误差收敛链路卡片
tf = textbox(s, 7.60, 5.00, 5.11, 0.24)
para(tf, "系统各层级误差逐步收敛链条", size=10, bold=True, color=BLUE, first=True)

prec = [("视觉原始测距", "10 ~ 15 mm"), ("手眼矩阵标定", "7.6 mm 留出"), ("机械臂重复定位", "≤ 1.0 mm")]
px = 7.60
for label, val in prec:
    rect(s, px, 5.30, 1.62, 0.85, fill=CARD2, line=None, radius=0.06)
    tf = textbox(s, px + 0.08, 5.38, 1.46, 0.28)
    para(tf, val, size=11.5, bold=True, color=BLUE, align=PP_ALIGN.CENTER, first=True)
    tf = textbox(s, px + 0.08, 5.68, 1.46, 0.40)
    para(tf, label, size=7.8, color=SLATE, align=PP_ALIGN.CENTER, first=True, line_spacing=1.15)
    px += 1.74

tf = textbox(s, MARGIN, 6.42, CONTENT_W, 0.40)
para(tf, "★ 系统性漂移补偿机制：真实套入实验发现视觉与大臂受重力下垂存在固定偏移，我们将其反算为固定三维偏差矩阵写入 config/pick_offset.json，使末端进刀命中率接近 100%。",
     size=8.3, italic=True, color=MUTED, first=True, line_spacing=1.25)

notes(s, """【机械臂控制与末梢执行 - 60秒】
低位果实采摘的终极难点不在于远距离发现，而在于“伸入枝叶套住果实”的那几秒钟。
我们的机械臂基于 Dummy-Robot 深度演化，重新开发了基于 ros2_control C++ 规范的硬件驱动插件，使底层电机与 MoveIt 2 无缝对接。
为了确保套果成功率，我们进行了两项关键创新：
第一，在末端设计了 120mm 大口径柔性漏斗进刀机构，将控制目标从“刀尖对果梗”转变为“漏斗对果心”，以 20° 仰角由下至上插入，实测进刀容差高达数厘米，完全克服了自然风吹枝条的轻微晃动；
第二，在精度链条上，手眼标定留出法实测误差控制在 7.6mm，机械臂重复定位达到 1mm 级。同时我们针对机械臂末端重力下垂与安装累积公差，建立了专门的固定补偿矩阵 pick_offset.json，真正做到了“指哪套哪”。
在安全方面，我们设立了严格的关节限位校验，例如大臂 J3 绝对禁止低于 30°，从代码底层杜绝了历史调试中出现的机械干涉断臂风险。""")

# ================================================================ P08: 工程挑战与技术取舍
s = new_slide(bg=NAVY)
header(s, "07 / 真实工程实操", "边缘硬件资源约束下的技术权衡：USB 总线瓶颈与系统拓扑收敛",
       "评委所看重的真实工程素养：在算力、供电与总线带宽严重受限的边缘嵌入式平台上，明确“该放弃什么，该坚守什么”。",
       tag_color=RGBColor(0xFD, 0xBA, 0x74), title_color=WHITE, sub_color=RGBColor(0xCB, 0xD5, 0xE1))

# 四步因果递进
steps = [
    ("1. 物理矛盾暴露", "板载原生 USB 接口极度匮乏", "双深度相机 + 底盘主控串口 + 机械臂主控串口同时在线，外设数量超出板载端口承载上限。", RGBColor(0xEF, 0x44, 0x44)),
    ("2. 恶化故障分析", "外接扩展坞引发总线重枚举", "双深度相机以 5Gbps 高频输出点云数据，引发总线带宽与电流过载，扩展坞崩溃导致全外设树掉线。", RGBColor(0xF9, 0x73, 0x16)),
    ("3. 关键架构抉择", "果断精简非核心链路保闭环", "将采摘相机 RealSense 直插板载原生高速 USB3；底盘与机械臂独立走板载串口；果断切断语音模块。", RGBColor(0x22, 0xC5, 0x5E)),
    ("4. 确定性收益达成", "整车通信零丢包零中断", "消除总线竞争后，双目相机稳定同屏出流，单帧推理稳定在 17.1 ms，整机连续作业闭环完全跑通。", BLUE),
]
x = MARGIN
for tag, title, desc, col in steps:
    rect(s, x, 1.95, 2.92, 1.95, fill=RGBColor(0x13, 0x1C, 0x2E),
         line=RGBColor(0x2A, 0x3A, 0x55), radius=0.06)
    bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(1.95), Inches(2.92), Inches(0.05))
    bar.fill.solid(); bar.fill.fore_color.rgb = col; bar.line.fill.background(); bar.shadow.inherit = False

    tf = textbox(s, x + 0.16, 2.10, 2.60, 0.25)
    para(tf, tag, size=8.5, bold=True, color=col, first=True)
    tf = textbox(s, x + 0.16, 2.38, 2.60, 0.50)
    para(tf, title, size=11.5, bold=True, color=WHITE, first=True, line_spacing=1.15)
    tf = textbox(s, x + 0.16, 2.92, 2.60, 0.90)
    para(tf, desc, size=8.5, color=RGBColor(0xA8, 0xB6, 0xC8), first=True, line_spacing=1.35)
    x += 3.06

# 下半部分：真实供电接线拓扑图与电气约束
rect(s, MARGIN, 4.10, 7.30, 2.70, fill=RGBColor(0x13, 0x1C, 0x2E),
     line=RGBColor(0x2A, 0x3A, 0x55), radius=0.05)
tf = textbox(s, MARGIN + 0.20, 4.20, 6.90, 0.26)
para(tf, "实车单轨供电拓扑与安全接地（源自工程文档 docs/power_wiring.png）", size=10, bold=True, color=RGBColor(0x60, 0xA5, 0xFA), first=True)

picture_fit(s, IMG_POWER_WIRING, MARGIN + 0.20, 4.50, 6.90, 2.15)

# 右下：深刻工程洞察
rect(s, 8.20, 4.10, 4.51, 2.70, fill=RGBColor(0x1E, 0x29, 0x3B),
     line=RGBColor(0x3B, 0x82, 0xF6), radius=0.05)
tf = textbox(s, 8.40, 4.25, 4.11, 0.28)
para(tf, "电气安全与工程取舍深度复盘", size=11, bold=True, color=RGBColor(0xFD, 0xBA, 0x74), first=True)

insights = [
    ("断电不能作为急停手段", "实车采用牧田 21V 单轨供电，机械臂与计算板同轨。若拉断总闸 SW1，机械臂瞬间失电直接因自重坠落，必须靠 MCU 固件锁存减速。"),
    ("保核心业务、弃次要交互", "在嵌入式开发中，花哨的离线语音远不如稳定的视觉流关键。裁撤语音换来全系统 100% 稳定性，体现了极高工程克制力。"),
    ("问题明晰且具备演进路径", "该问题根源在于开发板引脚形态，后续通过设计专用载板 PCB 即可彻底收敛，软件架构完全免于重构。"),
]
iy = 4.60
for t, d in insights:
    tf = textbox(s, 8.40, iy, 4.11, 0.60)
    para(tf, "• " + t, size=9, bold=True, color=WHITE, first=True, line_spacing=1.1)
    para(tf, d, size=7.8, color=RGBColor(0xCB, 0xD5, 0xE1), line_spacing=1.25, space_before=1)
    iy += 0.68

notes(s, """【真实工程挑战与技术取舍 - 60秒 —— 本场答辩最具含金量一页】
这一页我们向各位评委展现一个最真实、最具说服力的嵌入式攻坚过程。
在开发后期，我们遭遇了严峻的物理瓶颈：车上要同时带两路深度相机、两路 MCU 串口以及音频模块，端口严重不足。当我们引入外接 USB Hub 时，高带宽点云传输瞬间导致供电压降与总线过载，整个 USB 树频繁重枚举，传感器全线掉线。
面对这一问题，我们没有陷入无休止的打补丁，而是果断做出了系统级工程取舍：
第一，保障核心生命线：侧向采摘相机 RealSense 独占板载唯一的原生 USB 3.0 高速通道，底盘与手臂走直连串口；
第二，坚决砍掉非刚需：果断拔除离线语音 USB 声卡，将系统中断与总线算力 100% 倾斜给移动采摘与安全监控；
第三，右图展示了我们真实的 21V 单轨供电电气拓扑图：由于机械臂无自锁抱闸，总电源切断会导致手臂因重力猛烈下坠断臂，因此我们明确确立了“断电绝不可作急停”的规范，将全部安全兜底下沉至 STM32 的固件看门狗与平滑减速。
这种在有限资源下的克制与决断，最终保证了采摘全流程的高可靠闭环！""")

# ================================================================ P09: 实机验证与指标
s = new_slide()
header(s, "08 / 实机系统验证", "真机软硬件联调与各子系统性能验收指标",
       "脱离单纯算法仿真，所有数据与功能结论均在实车（VENTUNO Q + 真实柑橘测试场景）落地核验。")

# 三张核心实拍图
pics = [
    (IMG_CAR_TOP, "整机俯视全景：轻量铝型材窄体底盘、越野宽胎、同轴双相机支架与柔性采摘机构", 0.62, 1.95, 4.05, 2.25),
    (IMG_CAR_END, "末端执行器特写：3D 打印 120mm 聚拢漏斗、导流收集软管与机械臂紧凑关节布局", 4.85, 1.95, 4.05, 2.25),
    (IMG_CAR_SIDE, "侧面巡航姿态：大轮距适应丘陵果园微起伏，前向 Astra 与侧向 D435i 双目异构架设", 9.08, 1.95, 3.63, 2.25),
]
for path, cap, px, py, pw, ph in pics:
    rect(s, px, py, pw, ph, fill=CARD, line=LINE, radius=0.03)
    picture_fit(s, path, px + 0.05, py + 0.05, pw - 0.10, ph - 0.10)
    tf = textbox(s, px, py + ph + 0.08, pw, 0.40)
    para(tf, cap, size=7.5, color=SLATE, first=True, line_spacing=1.2)

# 下半部分：实测验证指标矩阵
tf = textbox(s, MARGIN, 4.85, CONTENT_W, 0.28)
para(tf, "全系统综合实测验收台账（全部经实车闭环验证通过）", size=12, bold=True, color=INK, first=True)

checks = [
    ("底盘运动伺服控制", "3 通道 50Hz PWM 实测；500ms 看门狗超前拦截；起步平滑斜率 1%/帧；锁存急停生效，无滑移冲量。", GREEN),
    ("双目并发数据吞吐", "Astra (前向结构光) 与 D435i (侧向RGB-D) 经板载隔离后，以 15Hz 双通道稳定出流，0 丢包 0 掉线。", GREEN),
    ("空间手眼标定精度", "ArUco + PnP 粗配准 + 六点 SVD 精解算；留出法实测验证误差 7.6 mm；机械臂末端单点定位精度达 1mm 级。", GREEN),
    ("真机套果进刀对准", "自然状态柑橘套入实验，漏斗罩口对心误差 ≤ 3 mm，仰角进刀避免果梗碰折，落料顺畅无果皮挤压伤。", GREEN),
    ("全流程状态机编排", "从巡航探果、视觉对齐、定点停车、机械臂准备、进刀套果、人机确认到剪切复位，支持单轮及多轮循环作业。", GREEN),
    ("语音模块后续演进", "确认受限于当前物理端口拓扑，本版已将剪切确认安全前移至人机终端，专用底板布设后将原位接回。", ORANGE),
]
y = 5.20
col_x = [MARGIN, MARGIN + 6.16]
for i, (name, desc, col) in enumerate(checks):
    cx = col_x[i % 2]
    yy = y + (i // 2) * 0.65
    rect(s, cx, yy, 5.96, 0.58, fill=CARD, line=None, radius=0.06)
    circle(s, cx + 0.16, yy + 0.22, 0.11, fill=col)
    tf = textbox(s, cx + 0.38, yy + 0.12, 1.35, 0.34)
    para(tf, name, size=9.5, bold=True, color=INK, first=True, line_spacing=1.0)
    tf = textbox(s, cx + 1.78, yy + 0.12, 4.05, 0.34)
    para(tf, desc, size=8, color=SLATE, first=True, line_spacing=1.15)

notes(s, """【实机验证与综合指标 - 45秒】
各位评委请看这一页展示的真实样机照片与验收台账。
上方左图为我们整机的俯瞰全貌，底盘采用定制轻量化铝合金框架，搭配四枚大轮径橡胶越野轮，满足果园微地形越障；
中图与右图展示了采摘末端与刚性双目支架的安装细节：3D 打印漏斗罩口不仅具有引导聚拢果实的功能，其尾部直接与导向软管连通，剪切完成后重力自动落料。
下方的实测台账记录了我们所有经由真机验证的关键参数：
底盘实现了 500ms 看门狗与 1%/帧的平滑加减速控制；双目相机实现 15Hz 稳定并行出流；手眼标定与末端对心误差控制在 3mm 级；更关键的是，整套“巡航-找果-对位-进刀-剪断-复位”的九步闭环完整跑通，具备高度的工程完整性。""")

# ================================================================ P10: 商业前景与社会价值
s = new_slide()
header(s, "09 / 商业与社会价值", "从单株验证到规模普及：重塑南方丘陵农业装备性价比",
       "以边缘高算力 SoC 替代工业 PC，大幅降低单机成本与功耗门槛，直面老龄化时代农业刚需。")

vals = [
    ("极高性价比", "单芯片替代工控机架构", "传统工业 PC + 外挂显卡方案成本超万元、功耗数百瓦；采用高通智能单板方案，核心计算硬件成本降低 60% 以上，车载电池续航翻倍。", BLUE),
    ("免地图极速部署", "真正适应分散非标农田", "摆脱高成本 RTK-GNSS 与高精激光 SLAM 依赖，无需提前测绘建图；换果园“即开即用”，使农业机器人具备在中小农户推广的实际可行性。", CYAN),
    ("破解农村老龄化", "社会价值与劳动力解放", "直接替代高强度、高频次弯腰采摘重体力劳动，单亩节省采摘人工成本超 40%，有效解决秋季采收“招工难、招工贵”的农业心腹之痛。", GREEN),
]
x = MARGIN
for title, sub, desc, col in vals:
    rect(s, x, 1.95, 3.86, 2.25, fill=WHITE, line=LINE, radius=0.06)
    tf = textbox(s, x + 0.22, 2.12, 3.42, 0.26)
    para(tf, title, size=10.5, bold=True, color=col, first=True)
    tf = textbox(s, x + 0.22, 2.44, 3.42, 0.42)
    para(tf, sub, size=13, bold=True, color=INK, first=True, line_spacing=1.15)
    tf = textbox(s, x + 0.22, 2.96, 3.42, 1.10)
    para(tf, desc, size=9.2, color=SLATE, first=True, line_spacing=1.45)
    x += 4.04

# 下半部分：高通平台赋能与未来三步走
rect(s, MARGIN, 4.45, 6.00, 2.45, fill=CARD2, line=LINE, radius=0.05)
tf = textbox(s, MARGIN + 0.25, 4.60, 5.50, 0.28)
para(tf, "为什么高通智能边缘平台是最佳载体？", size=11.5, bold=True, color=BLUE, first=True)

points = [
    ("算力密度极致", "40 TOPS Hexagon NPU 结合 8 核 Kryo，使得单板并发处理神经网络推理与 MoveIt 轨迹规划成为现实，彻底告别外挂算力盒子。"),
    ("超低功耗续航", "移动级高能效架构，全系统静态功耗极低，支持车载锂电池直供，单次充电支持数小时连续野外作业。"),
    ("生态原生无缝", "原生支持 Ubuntu 与 ROS 2 Jazzy，工业级机器人开发工具（ros2_control / MoveIt 2）直接落地，研发迭代与工程交付极快。"),
]
py = 4.98
for t, d in points:
    tf = textbox(s, MARGIN + 0.25, py, 1.10, 0.26)
    para(tf, t, size=9, bold=True, color=INK, first=True)
    tf = textbox(s, MARGIN + 1.40, py, 4.35, 0.52)
    para(tf, d, size=8.5, color=SLATE, first=True, line_spacing=1.3)
    py += 0.60

rect(s, 6.92, 4.45, 5.80, 2.45, fill=WHITE, line=LINE, radius=0.05)
tf = textbox(s, 7.15, 4.60, 5.30, 0.28)
para(tf, "产品化演进路线三步走规划", size=11.5, bold=True, color=INK, first=True)

roadmap = [
    ("Phase 1", "专用底板硬件收敛", "定制一体化载板，淘汰外接线缆与扩展坞，原位恢复离线语音与微型雷达接入。"),
    ("Phase 2", "作业节拍大幅压缩", "优化机械臂加减速规划与并行动作，实现「边走边看、一停多采」，单果作业压缩至 20s 内。"),
    ("Phase 3", "多机集群协同采收", "基于高通端侧蜂窝/Wi-Fi网络实现多台小车无人编队巡检，实现标准化现代果园集群作业。"),
]
ry = 4.98
for num, t, d in roadmap:
    tf = textbox(s, 7.15, ry, 0.70, 0.26)
    para(tf, num, size=9.5, bold=True, color=ORANGE, first=True)
    tf = textbox(s, 7.95, ry, 1.40, 0.26)
    para(tf, t, size=9, bold=True, color=INK, first=True)
    tf = textbox(s, 9.40, ry, 3.10, 0.52)
    para(tf, d, size=8.2, color=SLATE, first=True, line_spacing=1.3)
    ry += 0.60

notes(s, """【商业前景与社会价值 - 45秒】
最后我们从商业可行性与社会效益的角度向评委汇报。
一项农业机器人技术能否走出实验室，关键在于成本与鲁棒性。
传统方案之所以无法普及，核心原因是把工业自动化的一套生搬硬套到农田里，一套带工控机和激光雷达的小车动辄数万元。
而我们的方案证明了：仅凭一颗高通智能边缘单板，即可在千元级计算单元成本内，完整实现移动操作机器人的全栈功能！
第一，硬件成本降低 60%，功耗降至几十瓦，车载电池即可支持长时间续航；
第二，免去了高昂的建图测绘成本，果农在复杂的非标梯田里“买来即用”；
第三，真正直面老龄化用工荒，将农民从高强度的弯腰劳作中解放出来。
未来通过专用底板定制与 NPU 深度推理优化，我们将进一步推进果园多机集群作业，实现科技对农业的普惠赋能。""")

# ================================================================ P11: 总结与致谢
s = new_slide(bg=NAVY, footer=False)
ln = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width, Inches(0.06))
ln.fill.solid(); ln.fill.fore_color.rgb = BLUE; ln.line.fill.background(); ln.shadow.inherit = False

tf = textbox(s, MARGIN, 0.76, CONTENT_W, 0.30)
para(tf, "10 / 总结与答辩", size=10.5, bold=True, color=RGBColor(0x60, 0xA5, 0xFA), first=True)

tf = textbox(s, MARGIN, 1.15, 8.6, 0.65)
para(tf, "单芯成系统 · 端到端闭环 · 真实的工程笃行", size=29, bold=True, color=WHITE,
     first=True, line_spacing=1.1)

summary = [
    ("扎实做成", "在一台高通智能边缘计算板上，完整打通了异构双目感知、阿克曼底盘伺服逼近、六轴臂毫米级套果与剪切的端到端全闭环作业。",
     RGBColor(0x4A, 0xDE, 0x80)),
    ("深度讲透", "确立五层单向依赖架构与三大工程死规矩；坚持有所为有所不为，用最务实克制的设计保障系统极端鲁棒与可维护性。",
     RGBColor(0x60, 0xA5, 0xFA)),
    ("边界清晰", "客观剖析 USB 总线瓶颈与硬件供电取舍，真实呈现工程挑战背后的攻坚复盘，演进路线清晰且具备规模化量产潜力。",
     RGBColor(0xFD, 0xBA, 0x74)),
]
ry = 2.10
for tag, body, col in summary:
    rect(s, MARGIN, ry, 8.30, 1.18, fill=RGBColor(0x13, 0x1C, 0x2E),
         line=RGBColor(0x2A, 0x3A, 0x55), radius=0.05)
    tf = textbox(s, MARGIN + 0.25, ry + 0.16, 1.30, 0.28)
    para(tf, tag, size=10.5, bold=True, color=col, first=True)
    tf = textbox(s, MARGIN + 1.60, ry + 0.16, 6.45, 0.85)
    para(tf, body, size=10, color=RGBColor(0xCB, 0xD5, 0xE1), first=True, line_spacing=1.45)
    ry += 1.28

tf = textbox(s, MARGIN, 6.00, 8.30, 0.30)
para(tf, "高通边缘计算赋能现代农业装备，推动农田采收从高耗低效迈向自主智能化", size=9.5,
     color=RGBColor(0xFD, 0xBA, 0x74), first=True)

# 右侧：实车特写与致谢
picture_fit(s, IMG_CAR_TOP, 9.30, 2.10, 3.42, 3.90, border=RGBColor(0x2A, 0x3A, 0x55))

tf = textbox(s, 9.30, 6.15, 3.42, 0.65)
para(tf, "感谢各位评委老师批评指正！", size=14, bold=True, color=WHITE, first=True, line_spacing=1.1)
para(tf, "请各位评委提问 (Q & A)", size=9.5, color=RGBColor(0x7D, 0x8C, 0xA3), space_before=2)

notes(s, """【答辩收尾 - 30秒】
最后用三句话做总结汇报：
第一，我们扎实做成了：在一台高通智能边缘单板上，完整实现了双目感知、移动底盘、六轴臂高精套果剪切的全栈闭环；
第二，我们深度讲透了：通过严谨的五层架构与单一写者机制，保证了多外设协同的高确定性与安全性；
第三，我们客观呈现了真实的工程边界与取舍，明确了下一阶段走向产品量产的清晰演进路径。
以上就是我的全部汇报内容，感谢各位评委老师的倾听，请老师们批评指正！""")

prs.save(OUT)
print("OK ->", OUT)
print("slides:", len(prs.slides._sldIdLst))
