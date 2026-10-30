#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 垫高.stl / 垫高 右边.stl 反推二维轮廓，输出 DXF(供 Fusion 插入草图) + SVG(查看用)。

坐标约定：DXF 的 X = STL 的 X，DXF 的 Y = STL 的 Z，
原点取在轮廓左下角 A 点。

轮廓 = 8 边形（主轮廓，沿 Y 拉伸 30mm）+ 底部 4mm 定位凸台（6.5mm 宽）。
"""
import math, os

OUT = os.path.dirname(os.path.abspath(__file__))

# ---- 设计意图参数（与 STL 实测吻合到 0.001mm）----
ANG = 13.0                      # 斜面倾角
RAD = math.radians(ANG)
U = (math.cos(RAD), math.sin(RAD))     # 沿斜面的方向
V = (-math.sin(RAD), math.cos(RAD))    # 垂直斜面的方向

# C 点（右下角，斜面起点）相对 A 点的位置
K = (50.0, 94.9293)   # A -> C : 水平 50*cos13 = 48.7185, 竖直 94.9293
C = (50.0 * math.cos(RAD), 94.9293)

def add(p, d, s):
    return (p[0] + d[0] * s, p[1] + d[1] * s)

D = add(C, V, 50.0)      # 斜面 CD，长 50
E = add(D, U, -6.0)      # DE，长 6（垂直于 CD）
F = add(E, V, -44.0)     # EF，长 44（平行于 CD）
G = add(F, U, -44.0)     # FG，长 44（垂直于 CD）
H = add(G, V, -6.0)      # GH，长 6（平行于 CD）
A = (0.0, 0.0)
B = (C[0], 0.0)

PROFILE = [A, B, C, D, E, F, G, H]
NAMES = "ABCDEFGH"

# 底部凸台（在 STL 里只出现在 y = -158.5 ~ -152，宽 6.5mm，深 4mm）
RIB_DEPTH = 4.0
RIB = [(0.0, -RIB_DEPTH), (B[0], -RIB_DEPTH), (B[0], 0.0), (0.0, 0.0)]

THICK = 30.0        # 主轮廓沿 Y 拉伸厚度
RIB_W = 6.5         # 凸台沿 Y 宽度


def fmt(x):
    return ("%.4f" % x).rstrip("0").rstrip(".")


# ---------------- DXF (R12) ----------------
def dxf(path, only="PROFILE"):
    L = []
    def g(code, val):
        L.append(str(code)); L.append(str(val))

    g(0, "SECTION"); g(2, "HEADER")
    g(9, "$ACADVER"); g(1, "AC1009")
    g(9, "$INSUNITS"); g(70, 4)
    g(0, "ENDSEC")

    g(0, "SECTION"); g(2, "TABLES")
    g(0, "TABLE"); g(2, "LAYER"); g(70, 1)
    for name, color in (("PROFILE", 7),):
        g(0, "LAYER"); g(2, name); g(70, 0); g(62, color); g(6, "CONTINUOUS")
    g(0, "ENDTAB")
    g(0, "ENDSEC")

    g(0, "SECTION"); g(2, "ENTITIES")

    def poly(pts, layer, closed=True):
        g(0, "POLYLINE"); g(8, layer); g(66, 1); g(70, 1 if closed else 0)
        g(10, 0.0); g(20, 0.0); g(30, 0.0)
        for p in pts:
            g(0, "VERTEX"); g(8, layer); g(10, fmt(p[0])); g(20, fmt(p[1])); g(30, 0.0)
        g(0, "SEQEND"); g(8, layer)

    poly(PROFILE if only == "PROFILE" else RIB, only)

    g(0, "ENDSEC"); g(0, "EOF")
    open(path, "w", encoding="ascii", newline="\r\n").write("\n".join(L) + "\n")


# ---------------- SVG ----------------
def svg(path):
    pts = PROFILE + RIB
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    pad = 26.0
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    S = 5.2  # px per mm

    def X(x): return (x - x0) * S
    def Y(y): return (y1 - y) * S

    w = (x1 - x0) * S; h = (y1 - y0) * S
    out = []
    out.append('<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" '
               'viewBox="0 0 %.0f %.0f">' % (w, h, w, h))
    out.append('<rect width="100%%" height="100%%" fill="#ffffff"/>')
    out.append('<defs><style>'
               '.ol{fill:#eef2f7;stroke:#1f2933;stroke-width:2.2;stroke-linejoin:round}'
               '.rib{fill:#fff7ed;stroke:#ea580c;stroke-width:1.8;stroke-dasharray:7 5}'
               '.dim{stroke:#0b7285;stroke-width:1;fill:none}'
               '.ext{stroke:#0b7285;stroke-width:.6;fill:none;stroke-dasharray:4 4}'
               '.t{font:600 15px -apple-system,Helvetica,Arial;fill:#0b7285}'
               '.vt{font:700 13px Helvetica;fill:#c92a2a}'
               '.nt{font:600 14px -apple-system,Helvetica,Arial;fill:#1f2933}'
               '</style></defs>')

    dpath = "M " + " L ".join("%.2f %.2f" % (X(p[0]), Y(p[1])) for p in PROFILE) + " Z"
    out.append('<path class="ol" d="%s"/>' % dpath)

    dpath2 = "M " + " L ".join("%.2f %.2f" % (X(p[0]), Y(p[1])) for p in RIB) + " Z"
    out.append('<path class="rib" d="%s"/>' % dpath2)

    # 顶点标注
    for i, p in enumerate(PROFILE):
        out.append('<circle cx="%.2f" cy="%.2f" r="3.2" fill="#c92a2a"/>' % (X(p[0]), Y(p[1])))
        out.append('<text class="vt" x="%.2f" y="%.2f">%s</text>'
                   % (X(p[0]) + 7, Y(p[1]) - 7, NAMES[i]))

    # 尺寸线
    cx = sum(p[0] for p in PROFILE) / 8.0
    cy = sum(p[1] for p in PROFILE) / 8.0
    labels = ["48.718", "94.929", "50.000  (13°)", "6.000", "44.000", "44.000", "6.000", "83.682"]
    for i in range(8):
        p = PROFILE[i]; q = PROFILE[(i + 1) % 8]
        dx = q[0] - p[0]; dy = q[1] - p[1]
        Ln = math.hypot(dx, dy)
        ux, uy = dx / Ln, dy / Ln
        nx, ny = uy, -ux          # 法线
        mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
        if (mx - cx) * nx + (my - cy) * ny < 0:
            nx, ny = -nx, -ny     # 朝外
        off = 11.0
        a = (p[0] + nx * off, p[1] + ny * off)
        b = (q[0] + nx * off, q[1] + ny * off)
        out.append('<line class="ext" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>'
                   % (X(p[0]), Y(p[1]), X(a[0]), Y(a[1])))
        out.append('<line class="ext" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>'
                   % (X(q[0]), Y(q[1]), X(b[0]), Y(b[1])))
        out.append('<line class="dim" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>'
                   % (X(a[0]), Y(a[1]), X(b[0]), Y(b[1])))
        # 箭头
        for pt, sgn in ((a, 1), (b, -1)):
            out.append('<line class="dim" x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f"/>'
                       % (X(pt[0]), Y(pt[1]),
                          X(pt[0] + sgn * ux * 3.0 + nx * 0.0), Y(pt[1] + sgn * uy * 3.0)))
        ang = math.degrees(math.atan2(-(q[1] - p[1]), q[0] - p[0]))
        if ang > 90 or ang < -90:
            ang += 180
        if Ln < 12.0:
            ang = 0.0
        tx, ty = (a[0] + b[0]) / 2 + nx * 4.0, (a[1] + b[1]) / 2 + ny * 4.0
        out.append('<text class="t" transform="translate(%.2f %.2f) rotate(%.2f)" '
                   'text-anchor="middle">%s</text>' % (X(tx), Y(ty), ang, labels[i]))

    out.append('<text class="nt" x="18" y="%.0f">'
               '垫高 2D profile &#183; 6t walls &#183; extrude 30 mm &#183; units mm</text>'
               % (h - 18))
    out.append('<text class="nt" x="18" y="%.0f" fill="#ea580c">'
               'orange dashed = 4 mm locating rib (only 6.5 mm wide in Y)</text>' % (h - 40))
    out.append("</svg>")
    open(path, "w", encoding="utf-8").write("\n".join(out))


if __name__ == "__main__":
    dxf(os.path.join(OUT, "dian_gao_profile.dxf"), "PROFILE")
    dxf(os.path.join(OUT, "dian_gao_rib.dxf"), "RIB")
    svg(os.path.join(OUT, "dian_gao_profile.svg"))
    print("A(0,0)")
    for i, p in enumerate(PROFILE):
        print("  %s  (%9.4f, %9.4f)" % (NAMES[i], p[0], p[1]))
    print("rib depth %.1f width %.1f  thickness %.1f" % (RIB_DEPTH, RIB_W, THICK))
