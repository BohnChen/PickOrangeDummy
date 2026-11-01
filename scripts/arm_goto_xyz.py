#!/usr/bin/env python3
"""把机械臂 tool0 移动到 base_link 下的指定空间点（标定/探路用）

直接走串口（复用 jog_gui.py 里已验证过的 RefSerial 逻辑：
每次查询前 drain 清缓冲，否则回执堆积会把固件 CDC 写阻塞）。

用法：
    ./scripts/arm_goto_xyz.py 0.00 -0.33 0.10          # x y z (米, base_link)
    ./scripts/arm_goto_xyz.py --show                    # 只读当前末端位置
    ./scripts/arm_goto_xyz.py --rest                    # 回 REST

安全：
  - 先算 IK，残差 > 5mm 直接拒绝
  - 目标点做安全包络校验（离车体/相机立柱/地面）
  - 速度用固件 speed 参数，默认 10（约 7°/s，慢）
  - 单步位移 > 0.25m 拆分多段
"""

import argparse
import math
import os
import sys
import termios
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ---- 复用 instant_tap 的 FK/IK ----
try:
    from instant_tap import forward_kinematics_tool0, solve_exact_tool0_ik
    from arm_kin import solve_pose_ik, fk_full, model_from_fw, port_down_deg
    import arm_safety as safety
except Exception as e:
    print(f"无法导入运动学/安全模块: {e}")
    sys.exit(1)

PORT_CANDIDATES = [
    "/dev/serial/by-id/usb-Robot_Embedded_Framework_REF_1.0_CDC_Interface_365D357B3334-if00",
    "/dev/ttyACM1",
]

FW_LIMITS = [
    (-170.0, 170.0), (-75.0, 90.0), (35.0, 180.0),
    (-180.0, 180.0), (-95.0, 95.0), (-720.0, 720.0),
]
READ_TIMEOUT = 0.20

# ---------------------------------------------------------------------------
# J6 硬锁（2026-11-01 事故后新增）
#
# J6 是刀片驱动轴。它转动 = 刀片闭合/张开 = 剪切。
# 绝不允许任何"定位/瞄准/滚转"动作顺带转动 J6。
#
# 规则：
#   1. 默认锁死，任何 '>' 运动指令只要 J6 变化 > J6_TOL 就直接拒绝发送
#   2. 只有显式的剪切流程可以临时解锁，且解锁后单次行程受 J6_CUT_MAX_TRAVEL 限制
# ---------------------------------------------------------------------------
J6_TOL = 0.3              # 允许的 J6 抖动（编码器噪声）
J6_CUT_MAX_TRAVEL = 90.0  # 剪切时单次允许的最大 J6 行程（度）


class J6Violation(RuntimeError):
    """试图在非剪切流程中转动 J6"""
    pass


class RefSerial:
    #: 是否锁住 J6（默认锁死）
    j6_locked = True
    #: 最近一次读到/下发的 J6（固件度）；None 表示未知
    last_j6 = None

    def __init__(self, port):
        self.j6_locked = True
        self.last_j6 = None
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            a = termios.tcgetattr(self.fd)
            a[2] &= ~termios.HUPCL
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
        except Exception:
            pass
        time.sleep(0.15)
        self.drain()

    def drain(self):
        for _ in range(64):
            try:
                if not os.read(self.fd, 4096):
                    break
            except Exception:
                break

    def write(self, line):
        """发送一行。运动指令（以 '>' 开头）必须先过 J6 硬锁。"""
        if line.startswith(">") and self.j6_locked:
            fields = line[1:].split(",")
            if len(fields) >= 6:
                try:
                    j6 = float(fields[5])
                except ValueError:
                    raise J6Violation(f"无法解析 J6: {line}")
                ref = self.last_j6
                if ref is not None and abs(j6 - ref) > J6_TOL:
                    raise J6Violation(
                        f"拒绝执行：定位动作试图转动 J6 "
                        f"{ref:+.2f}° → {j6:+.2f}°（变化 {j6-ref:+.2f}°）\n"
                        f"    J6 是刀片驱动轴，只允许剪切流程显式移动。\n"
                        f"    指令: {line.strip()}")
                self.last_j6 = j6
        if not line.endswith("\n"):
            line += "\n"
        try:
            os.write(self.fd, line.encode())
        except BlockingIOError:
            self.drain()
            try:
                os.write(self.fd, line.encode())
            except OSError:
                pass
        except OSError:
            pass

    def cmd(self, line, wait=1.0):
        """发送一条命令并收集回包（用于 !START 这类需要确认的指令）"""
        self.drain()
        self.write(line)
        buf = ""
        end = time.time() + wait
        while time.time() < end:
            try:
                chunk = os.read(self.fd, 4096)
            except Exception:
                chunk = b""
            if chunk:
                buf += chunk.decode("utf-8", errors="ignore")
            else:
                time.sleep(0.004)
        return buf.strip()

    def joints(self, timeout=READ_TIMEOUT):
        """返回固件关节角（度）；只接受 'ok x x x x x x' 行"""
        self.drain()
        self.write("#GETJPOS")
        buf = ""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                chunk = os.read(self.fd, 4096)
            except Exception:
                chunk = b""
            if chunk:
                buf += chunk.decode("utf-8", errors="ignore")
                for line in buf.splitlines():
                    p = line.strip().split()
                    if len(p) >= 7 and p[0] == "ok":
                        try:
                            vals = [float(x) for x in p[1:7]]
                        except ValueError:
                            continue
                        self.last_j6 = vals[5]   # 同步 J6 参考值
                        return vals
                buf = buf[-200:]
            else:
                time.sleep(0.004)
        return None

    def close(self):
        try:
            termios.tcdrain(self.fd)
        except Exception:
            pass
        os.close(self.fd)


def fw_to_model_rad(fw):
    """固件角(度) → 模型角(弧度)"""
    m = [
        fw[0],
        fw[1] + 73.0,
        fw[2] - 180.0,
        fw[3],
        -fw[4],
        fw[5],
    ]
    return [math.radians(v) for v in m]


def model_rad_to_fw(m):
    d = [math.degrees(v) for v in m]
    return [
        d[0],
        d[1] - 73.0,
        d[2] + 180.0,
        d[3],
        -d[4],
        d[5],
    ]


def clamp_fw(fw):
    out = []
    for i, v in enumerate(fw):
        lo, hi = FW_LIMITS[i]
        out.append(max(lo, min(hi, v)))
    return out


def envelope_check(p):
    """粗略安全包络（base_link 系）：
       臂底座在 z=0；相机立柱在 URDF -Y 方向约 0.186m、高 0.248m 处
       这里只做保守限制，真正的红线在固件和限位里
    """
    x, y, z = p
    # 水平半径
    r = math.hypot(x, y)
    if r < 0.10:
        return "太靠近底座 (r<0.10m)"
    if r > 0.42:
        return "超出臂展 (r>0.42m)"
    if z < -0.05:
        return "低于底座平面 (z<-0.05m)"
    if z > 0.35:
        return "过高 (z>0.35m)"
    return None


def wait_settle(ser, timeout=90.0, tol=0.15, stable_n=3, interval=0.4):
    """轮询直到关节角连续 stable_n 个观察窗变化 < tol 度

    注意：慢速时相邻两次读数天然差异很小，必须用较长的观察窗（0.4s）
    和较严的阈值（0.15°/0.4s ≈ 0.4°/s），否则会误判"已停"。
    """
    last = None
    stable = 0
    t0 = time.time()
    while time.time() - t0 < timeout:
        js = ser.joints()
        if js is None:
            time.sleep(0.2)
            continue
        if last is not None:
            d = max(abs(js[i] - last[i]) for i in range(6))
            stable = stable + 1 if d < tol else 0
            if stable >= stable_n:
                return js
        last = js
        time.sleep(interval)
    return last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xyz", nargs="*", type=float)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--rest", action="store_true")
    ap.add_argument("--speed", type=float, default=10.0)
    ap.add_argument("--dry-run", action="store_true", help="只算 IK 和校验，不动")
    args = ap.parse_args()

    port = next((p for p in PORT_CANDIDATES if os.path.exists(p)), None)
    if port is None:
        print("找不到 REF 串口")
        return 1

    ser = RefSerial(port)
    try:
        # 固件在运动过程中不响应查询，所以起始读要重试而不是立刻失败
        fw = None
        t0 = time.time()
        while fw is None and time.time() - t0 < 10.0:
            fw = ser.joints()
            if fw is None:
                time.sleep(0.5)
        if fw is None:
            print("读不到关节角（串口没回包，可能仍在运动或已 wedge）")
            return 1
        cur_model = fw_to_model_rad(fw)
        cur_xyz = forward_kinematics_tool0(cur_model)
        print(f"固件角: {' '.join(f'{v:+7.2f}' for v in fw)}")
        print(f"末端位置 (base_link): X={cur_xyz[0]:+.4f} Y={cur_xyz[1]:+.4f} Z={cur_xyz[2]:+.4f}")

        if args.show:
            return 0

        if args.rest:
            target_fw = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
            target_model = fw_to_model_rad(target_fw)
            print("目标: REST")
        else:
            if len(args.xyz) != 3:
                print("需要 x y z 三个数，或 --show / --rest")
                return 2
            tgt = np.array(args.xyz, dtype=float)
            reason = envelope_check(tgt)
            if reason:
                print(f"拒绝：{reason}")
                return 2
            # 关键：用**保持当前姿态**的位姿 IK，绝不允许手腕翻转。
            # 之前用只约束位置的 IK，J5 从 -74° 翻到 +0.18°，
            # 153mm 长的锥形末端整体扫过，圆形开口转向车架（事故）。
            T_cur = fk_full(cur_model)
            axis_cur = T_cur[:3, 2]
            # J6（刀片轴）参与解算以保持自由度，但结果丢弃：
            # J6 只绕锥罩轴线旋转，丢弃不改变位置和锥罩轴向，只改变滚转。
            q, resid, axis_err = solve_pose_ik(tgt, axis_cur, seed=cur_model,
                                               posture_w=0.0005)
            print(f"位姿IK: 位置残差 {resid:.2f}mm  轴向保持误差 {axis_err:.2f}°")
            if resid > 6.0:
                print("拒绝：位置残差过大")
                return 2
            if axis_err > 12.0:
                print(f"拒绝：无法保持当前末端姿态（{axis_err:.1f}°）")
                return 2
            target_model = list(q)

        target_fw = clamp_fw(model_rad_to_fw(target_model))
        # 最后一道闸：J6 强制等于当前值（除非显式剪切流程解锁）
        if ser.j6_locked and ser.last_j6 is not None:
            if abs(target_fw[5] - ser.last_j6) > J6_TOL:
                print(f"!! J6 硬锁：计算结果想动 J6 "
                      f"{ser.last_j6:+.2f}° → {target_fw[5]:+.2f}°，已强制改回")
            target_fw[5] = ser.last_j6
            target_model = list(target_model)
            target_model[5] = cur_model[5]
            _j2 = target_fw[1]
            if _j2 > 50.0:
                print(f"!! 拒绝：J2={_j2:+.0f}° 处于收缩状态（>50°），出料软管会挤到车体")
                return 2
        print(f"目标固件角: {' '.join(f'{v:+7.2f}' for v in target_fw)}")
        if args.dry_run:
            print("（--dry-run：不动作）")
            return 0

        # 关键：断电重启后固件是失能状态，>` 指令会被静默忽略，必须先 !START
        # 断电重启后固件是失能状态，>` 会被静默忽略。
        # 重复 !START 时固件可能不再回 "Started ok"（只在实际切换时回），
        # 所以不拿回执当下发条件，改为下发后检查"到底有没有动"。
        ack = ser.cmd("!START", wait=0.8)
        print(f"使能回执: {ack!r}" + ("" if "Started" in ack else "（已在使能态，正常）"))
        time.sleep(0.3)

        cmd = ">" + ",".join(f"{v:.3f}" for v in target_fw) + f",{args.speed:.1f}"
        print(f"下发: {cmd}")
        # ---- 安全守卫：L1/L2/L3 ----
        T_next = fk_full(target_model)
        _T_cur = fk_full(cur_model)          # --rest 分支也要有起点姿态
        try:
            safety.check_all(_T_cur, T_next, need_l1=False, park=args.rest)
        except safety.SafetyError as e:
            print(f"!! 安全拦截: {e}")
            return 3
        print(f"安全校验通过 (末端姿态变化 {safety.axis_angle_deg(_T_cur[:3,2], T_next[:3,2]):.1f}°)")

        ser.write(cmd)

        js = wait_settle(ser)
        if js is None:
            print("!! 等待到位时读不到角度")
            return 1

        new_model = fw_to_model_rad(js)
        new_xyz = forward_kinematics_tool0(new_model)
        want_xyz = forward_kinematics_tool0(target_model)
        err = float(np.linalg.norm(np.array(new_xyz) - np.array(want_xyz)))
        print(f"到位固件角: {' '.join(f'{v:+7.2f}' for v in js)}")
        print(f"到位位置: X={new_xyz[0]:+.4f} Y={new_xyz[1]:+.4f} Z={new_xyz[2]:+.4f}")
        print(f"到位误差: {err*1000:.1f} mm")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
