#!/usr/bin/env python3
"""套果 + 剪切 + 退刀（方案 B：侧下方沿 -Y 进刀）

动作序列：
  1. STAGE   移动到预备位：X=果实X, Y=果实Y+13cm(退后), Z=果实Z-3cm(偏下)
  2. FEED    沿 -Y 以 1.5cm 步长慢慢进刀，直到 TCP 到达果实中心（略过 2cm）
  3. HOLD    停住，等剪切指令（--auto-cut 则直接切）
  4. CUT     J6 合刀 -60° → 保压 → 张刀回位
  5. RETREAT 沿 +Y 退 8cm，再抬升 5cm

安全：
  - 每步都重新校验 IK 残差与 J3/J5 限位，任一不满足立即停止
  - 进刀速度慢（speed=10），单步 1.5cm，步间停顿
  - --dry-run 只打印轨迹，不动

用法：
  ./scripts/arm_pick.py --detect            # 只检测并打印目标
  ./scripts/arm_pick.py --stage             # 仅移动到预备位
  ./scripts/arm_pick.py --full --auto-cut   # 完整周期（含自动剪切）
  ./scripts/arm_pick.py --full              # 完整周期，到 HOLD 等待
  ./scripts/arm_pick.py --cut               # 仅执行剪切（已套住时用）
  ./scripts/arm_pick.py --retreat           # 仅退刀
"""

import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import importlib.util

spec = importlib.util.spec_from_file_location(
    "cal", os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_eye_to_hand.py"))
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)
from instant_tap import forward_kinematics_tool0
from arm_kin import fk_full, solve_pose_ik, fw_from_model, model_from_fw

CALIB = "/tmp/opencode/calib_result.json"
AXIS_TOL_DEG = 25.0      # 套筒轴线与"指向果实"的最大允许偏差

# --- 末端执行器几何（从 STL 实测）---
# 外壳 Z 从 -36(法兰面) 到 +117(罩口平面)，共 153mm；罩口内径约 120mm
CONE_LEN_M = 0.153       # tool0(法兰面) → 罩口平面
MOUTH_DIA_M = 0.120      # 罩口内径

# --- 动作参数 ---
# 关键：目标是让【罩口】到达果实，而不是 TCP 到果心。
# TCP 退后 CONE_LEN 时，罩口正好落在果心（实测误差 3mm、轴向 0.00°）。
STAGE_BACK_M = 0.20      # 预备位：TCP 离果心 20cm（罩口离果心约 5cm；再远会超出最小可达距离）
STAGE_DOWN_M = 0.00      # 罩口足够大(120mm)，直接对准果心，不再下压
FEED_STEP_M = 0.015      # 进刀步长
FEED_PAST_M = 0.005     # 终点：刚好到果心（再往前 J3 会触底）
RETREAT_BACK_M = 0.08    # 退刀沿 +Y
RETREAT_UP_M = 0.05      # 退刀抬升
CUT_SPAN_DEG = 60.0
CUT_SPEED = 5.0

J3_MIN = 45.0            # 固件 J3 角下限（红线是 35°，这里留 10° 余量）
J5_MAX = 80.0


def log(msg):
    print(msg, flush=True)


def detect_fruit():
    rclpy_ok = True
    import rclpy
    rclpy.init()
    cam = cal.Cam()
    bgr = dep = None
    for _ in range(8):
        bgr, dep = cal.grab(cam)
        if bgr is not None:
            break
        time.sleep(0.3)
    if bgr is None:
        log("!! 相机无数据")
        rclpy.try_shutdown()
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([10, 80, 70]), np.array([45, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for ct in cnts:
        a = cv2.contourArea(ct)
        if a < 300:
            continue
        M = cv2.moments(ct)
        if M["m00"] <= 0:
            continue
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        m2 = np.zeros_like(mask)
        cv2.drawContours(m2, [ct], -1, 255, -1)
        vv = dep[m2 > 0]
        vv = vv[vv > 0]
        if vv.size < 30:
            continue
        z = float(np.median(vv)) / 1000.0
        if not (0.15 < z < 0.9):
            continue
        if best is None or a > best[0]:
            best = (a, u, v, z, ct)
    k = cam.k
    cam.destroy_node()
    rclpy.try_shutdown()
    if best is None:
        log("!! 没找到近距目标")
        return None
    a, u, v, z, ct = best
    pc = cal.pixel_to_cam(u, v, z, k)
    T = np.array(json.load(open(CALIB))["T_base_link_camera"])
    pb = T[:3, :3] @ np.array(pc) + T[:3, 3]
    # 用轮廓外接圆半径估计果实半径（像素→米）
    (_, _), rpx = cv2.minEnclosingCircle(ct)
    radius_m = float(rpx) * z / k[0]
    return {"pixel": (u, v), "depth": z, "p_base": pb, "radius": radius_m, "area": a}


def check_pose(target, seed, aim_at=None):
    """aim_at 非空时同时约束"套筒轴线指向 aim_at"（J5 就是套筒轴向）"""
    j6_lock = float(seed[5])          # J6 = 刀片轴，绝不参与定位
    if aim_at is None:
        q, res = cal.solve_exact_tool0_ik(np.array(target), seed=seed)
        if res > 5.0:
            return None, f"IK 残差 {res:.1f}mm", 0.0
        q = list(q); q[5] = j6_lock
        axis_err = 0.0
    else:
        d = np.asarray(aim_at, dtype=float) - np.asarray(target, dtype=float)
        n = np.linalg.norm(d)
        if n < 1e-6:
            return None, "瞄准点与目标重合", 0.0
        q, res, axis_err = solve_pose_ik(np.array(target), d / n, seed=seed,
                                         posture_w=0.0005, lock_j6=j6_lock)
        if res > 6.0:
            return None, f"位姿IK 位置残差 {res:.1f}mm", axis_err
        if axis_err > AXIS_TOL_DEG:
            return None, f"轴线偏差 {axis_err:.1f}° 过大", axis_err
    fw = cal.agx.clamp_fw(cal.agx.model_rad_to_fw(q))
    fw[5] = cal.agx.model_rad_to_fw(list(seed))[5]   # 再钉一次，确保与当前一致
    if not (J3_MIN < fw[2] < 179):
        return None, f"J3={fw[2]:.1f}° 不安全", axis_err
    if abs(fw[4]) > J5_MAX:
        return None, f"J5={fw[4]:.1f}° 不安全", axis_err
    return (q, fw), None, axis_err


def move_to(ser, fw, speed=20.0, timeout=50.0):
    js = None
    for k in range(3):
        ser.write(">" + ",".join(f"{v:.3f}" for v in fw) + f",{speed:.1f}")
        js = cal.wait_motion(ser, fw, timeout=timeout)
        if js and max(abs(js[i] - fw[i]) for i in range(6)) < 2.0:
            return js
        log(f"   重发第 {k+2} 次")
        time.sleep(0.8)
    return js


def do_stage(ser, fruit, dry):
    pf = fruit["p_base"]
    stage = np.array([pf[0], pf[1] + STAGE_BACK_M, pf[2] - STAGE_DOWN_M])
    log(f"STAGE 目标(TCP): X={stage[0]:+.3f} Y={stage[1]:+.3f} Z={stage[2]:+.3f}"
        f"   [罩口在 TCP 前方 {CONE_LEN_M*100:.0f}mm]")
    pose, err, ae = check_pose(stage, seed=[0.2, 0.8, -0.7, 0, 0.8, 0], aim_at=pf)
    if pose is None:
        log(f"!! 预备位不可用: {err}")
        return None
    log(f"   套筒瞄准果实，轴线偏差 {ae:.1f}°")
    if dry:
        log(f"   (dry-run) J={[round(v,1) for v in pose[1]]}")
        return stage
    js = move_to(ser, pose[1], speed=20.0)
    if js is None:
        log("!! 预备位移动失败")
        return None
    real = forward_kinematics_tool0(cal.agx.fw_to_model_rad(js))
    log(f"   到位 X={real[0]:+.3f} Y={real[1]:+.3f} Z={real[2]:+.3f}")
    return stage


def do_feed(ser, fruit, stage, dry):
    """沿"套筒实际轴线"分步进刀。

    为什么不是沿 -Y 直线：J5 顶到限位也只能把套筒瞄到 ~13° 以内，
    而越往深处伸偏差越大（42°）。沿套筒自己的轴线推进，
    开口始终"迎面"对着果实，不会用锥壁去推。
    """
    pf = fruit["p_base"]
    cur = stage.copy()
    step = 0
    seed = [0.2, 0.5, -0.5, 0, 0, 0]
    # 进刀到 TCP 距果心 CONE_LEN_M（此时罩口正好罩住果实）
    while np.linalg.norm(pf - cur) > CONE_LEN_M + 0.002:
        step += 1
        if step > 20:
            log("   进刀步数超限，停止")
            return False
        d = pf - cur
        dist = np.linalg.norm(d)
        step_m = min(FEED_STEP_M, dist - CONE_LEN_M)
        cur = cur + d / dist * step_m
        pose, err, ae = check_pose(cur, seed=seed, aim_at=pf)
        if pose is None:
            log(f"   第 {step} 步不可达（{err}），停止进刀")
            return False
        if dry:
            log(f"   (dry-run) 步{step} P=({cur[0]:+.3f},{cur[1]:+.3f},{cur[2]:+.3f}) "
                f"轴向偏差{ae:4.1f}° J={[round(v,1) for v in pose[1]]}")
            seed = pose[0]
            continue
        js = move_to(ser, pose[1], speed=10.0)
        if js is None or max(abs(js[i] - pose[1][i]) for i in range(6)) > 2.0:
            log(f"   第 {step} 步未到位，停止进刀")
            return False
        seed = pose[0]
        real = forward_kinematics_tool0(cal.agx.fw_to_model_rad(js))
        mouth = real + CONE_LEN_M * np.array(fk_full(cal.agx.fw_to_model_rad(js))[:3, 2])
        log(f"   进刀 步{step}: TCP距果心 {np.linalg.norm(real-pf)*1000:4.0f}mm  "
            f"罩口距果心 {np.linalg.norm(mouth-pf)*1000:4.0f}mm  轴向{ae:4.1f}°")
    return True


def do_servo_feed(ser, dry, max_step=0.010, stop_gap=0.015, max_iters=30):
    """实时视觉伺服进刀：每步重新检测果实、重新瞄准

    这解决三件事：
      1. 果实被碰到会摆动 → 每步重新测量
      2. J5 受末端重量下垂 → 闭环吸收
      3. 标定残差 → 闭环吸收
    """
    gap = None
    last_pf = None
    lost = 0
    for it in range(1, max_iters + 1):
        f = detect_fruit()
        if f is None:
            # 锥罩逼近后会挡住果实，属正常遮挡：沿用上一帧目标，最多 4 步
            lost += 1
            if last_pf is None or lost > 4:
                log(f"   目标丢失（连续 {lost} 次），停止")
                return False
            log(f"   目标被遮挡（第 {lost} 次），沿用上一帧位置")
            pf = last_pf
        else:
            lost = 0
            pf = f["p_base"]
            last_pf = pf
        pose_now = ser.joints()
        if pose_now is None:
            log("   读不到关节角，停止")
            return False
        T = fk_full(model_from_fw(pose_now))
        tcp = T[:3, 3]
        z = T[:3, 2]
        mouth = tcp + CONE_LEN_M * z
        gap = float(np.linalg.norm(mouth - pf))
        log(f"   迭代{it}: 罩口距果心 {gap*1000:4.0f}mm  "
            f"(果实 X={pf[0]:+.3f} Y={pf[1]:+.3f} Z={pf[2]:+.3f})")
        if gap <= stop_gap:
            log("   罩口已到位，停止进刀")
            return True
        # 目标：TCP 沿"指向果实"的方向前进一小步（罩口始终朝果实）
        d = pf - tcp
        n = np.linalg.norm(d)
        if n < 1e-6:
            return True
        axis = d / n
        # 罩口要落在果心 => TCP 目标 = 果心 - CONE_LEN * axis
        tcp_goal = pf - CONE_LEN_M * axis
        step_vec = tcp_goal - tcp
        sn = np.linalg.norm(step_vec)
        if sn > max_step:
            step_vec = step_vec / sn * max_step
        tcp_next = tcp + step_vec
        axis_next = (pf - tcp_next)
        axis_next = axis_next / (np.linalg.norm(axis_next) + 1e-9)
        pose, err, ae = check_pose(tcp_next, seed=model_from_fw(pose_now), aim_at=pf)
        if pose is None:
            log(f"   不可达（{err}），停止")
            return False
        if dry:
            log(f"     (dry-run) J={[round(v,1) for v in pose[1]]} 轴向{ae:.1f}°")
            continue
        js = move_to(ser, pose[1], speed=8.0, timeout=30.0)
        if js is None or max(abs(js[i] - pose[1][i]) for i in range(6)) > 2.0:
            log("   单步未到位，停止")
            return False
    log(f"   达到最大迭代数，最终间隙 {gap*1000:.0f}mm" if gap else "   失败")
    return False


def do_cut(ser, dry):
    """J6 合刀 → 保压 → 张刀

    全流程中 J6 是刀片驱动轴。这是**唯一**允许 J6 转动的入口：
      - 先做行程限位检查（|ΔJ6| ≤ J6_CUT_MAX_TRAVEL）
      - 临时解锁 serial 层的 J6 硬锁
      - 结束后无论成败都重新上锁
    """
    js = ser.joints()
    if js is None:
        log("!! 读不到 J6")
        return False
    home = js[5]
    if abs(CUT_SPAN_DEG) > cal.agx.J6_CUT_MAX_TRAVEL:
        log(f"!! 剪切行程 {CUT_SPAN_DEG}° 超过硬限位 "
            f"{cal.agx.J6_CUT_MAX_TRAVEL}°，拒绝")
        return False
    target_close = home - CUT_SPAN_DEG
    log(f"CUT: J6 {home:+.1f}° → {target_close:+.1f}° → 回 {home:+.1f}°")
    base = list(js)
    ser.j6_locked = False            # 仅本次剪切解锁
    try:
        return _cut_impl(ser, dry, base, home, target_close)
    finally:
        ser.j6_locked = True         # 无论如何都重新上锁
        _j = ser.joints()            # 用实测值作为新的锁定基准
        if _j:
            ser.last_j6 = _j[5]


def _cut_impl(ser, dry, base, home, target_close):
    for phase, j6 in (("合刀", target_close), ("张刀", home)):
        cmd = list(base)
        cmd[5] = j6
        if dry:
            log(f"   (dry-run) {phase}")
            continue
        for k in range(3):
            ser.write(">" + ",".join(f"{v:.3f}" for v in cmd) + f",{CUT_SPEED:.1f}")
            got = cal.wait_motion(ser, cmd, timeout=40.0, tol=1.0)
            if got and abs(got[5] - j6) < 2.0:
                break
            log(f"   {phase} 重发 {k+2}")
        if got is None or abs(got[5] - j6) > 3.0:
            log(f"!! {phase} 未到位")
            return False
        log(f"   {phase} 到位: J6={got[5]:+.1f}°")
        if phase == "合刀":
            time.sleep(0.5)
    return True


def do_retreat(ser, fruit, dry):
    pf = fruit["p_base"]
    back = np.array([pf[0], pf[1] + RETREAT_BACK_M, pf[2] - STAGE_DOWN_M])
    log(f"RETREAT: 后退到 Y={back[1]:+.3f} 再抬升 {RETREAT_UP_M*100:.0f}cm")
    for name, tgt in (("后退", back), ("抬升", back + np.array([0, 0, RETREAT_UP_M]))):
        pose, err = check_pose(tgt, seed=[0.2, 0.5, -0.3, 0, 0, 0])
        if pose is None:
            log(f"   {name}不可达（{err}），跳过")
            continue
        if dry:
            log(f"   (dry-run) {name}")
            continue
        js = move_to(ser, pose[1], speed=15.0)
        log(f"   {name}完成" if js is not None else f"   {name}失败")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detect", action="store_true")
    ap.add_argument("--stage", action="store_true")
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--cut", action="store_true")
    ap.add_argument("--retreat", action="store_true")
    ap.add_argument("--auto-cut", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--servo", action="store_true", help="实时视觉伺服进刀")
    args = ap.parse_args()

    fruit = detect_fruit()
    if fruit is None:
        return 1
    pf = fruit["p_base"]
    log(f"\n目标果实: 像素={fruit['pixel']} 深度={fruit['depth']*1000:.0f}mm "
        f"半径≈{fruit['radius']*1000:.1f}mm")
    log(f"  base_link: X={pf[0]:+.4f} Y={pf[1]:+.4f} Z={pf[2]:+.4f}")
    if args.detect:
        return 0

    ser = cal.agx.RefSerial(cal.agx.PORT_CANDIDATES[0]
                            if os.path.exists(cal.agx.PORT_CANDIDATES[0])
                            else cal.agx.PORT_CANDIDATES[1])
    try:
        ser.cmd("!START", 0.8)
        if args.cut:
            return 0 if do_cut(ser, args.dry_run) else 1
        if args.retreat:
            return 0 if do_retreat(ser, fruit, args.dry_run) else 1

        stage = do_stage(ser, fruit, args.dry_run)
        if stage is None:
            return 1
        if args.stage:
            return 0

        fed = do_servo_feed(ser, args.dry_run) if args.servo else do_feed(ser, fruit, stage, args.dry_run)
        if not fed:
            log("进刀失败，执行退刀")
            do_retreat(ser, fruit, args.dry_run)
            return 1

        log("\n>>> 已套住。")
        if args.full and not args.auto_cut:
            log("    等待剪切指令：./scripts/arm_pick.py --cut")
            return 0
        if args.auto_cut or args.full:
            if not do_cut(ser, args.dry_run):
                return 1
            do_retreat(ser, fruit, args.dry_run)
            log("\n=== 采摘动作完成 ===")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
