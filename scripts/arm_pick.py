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
from arm_kin import (fk_full, fk_shell, port_down_deg, port_down_deg_from_q,
                     solve_pose_ik, solve_full_ik, tool_frame_from_axis,
                     fw_from_model, model_from_fw)

CALIB = "/tmp/opencode/calib_result.json"
AXIS_TOL_DEG = 25.0      # 套筒轴线与"指向果实"的最大允许偏差

# --- 末端执行器几何（从 STL 实测）---
# 外壳 Z 从 -36(法兰面) 到 +117(罩口平面)，共 153mm；罩口内径约 120mm
CONE_LEN_M = 0.125       # tool0(法兰面) → 罩口平面（STL 实测 Z=-8→117）
MOUTH_DIA_M = 0.120      # 罩口内径

# --- 动作参数 ---
# 关键：目标是让【罩口】到达果实，而不是 TCP 到果心。
# TCP 退后 CONE_LEN 时，罩口正好落在果心（实测误差 3mm、轴向 0.00°）。
STAGE_GAP_M  = 0.05      # 预备位：罩口离果心 5cm
ELEV_DEG     = 20.0      # 进刀仰角（从斜下方往上套；果梗在上、果实在下，仰角更容易套住）
STAGE_BACK_M = CONE_LEN_M + STAGE_GAP_M
STAGE_DOWN_M = 0.00
FEED_STEP_M = 0.015      # 进刀步长
FEED_PAST_M = 0.005     # 终点：刚好到果心（再往前 J3 会触底）
FEED_DEPTH_PAST_M = 0.020   # 罩口越过果心的深度
# 为什么是 20mm：刀片平面在 tool0 起算 126~136mm，果实半径约 20mm，
# 要让果实**完全进到刀片平面内侧**（剪的是果梗不是果实），
# 果心必须 ≤ 125-20 = 105mm ⇒ 罩口要越过果心 20mm。
RETREAT_BACK_M = 0.08    # 退刀沿 +Y
RETREAT_UP_M = 0.05      # 退刀抬升
CUT_SPAN_DEG = 60.0
CUT_SPEED = 5.0

J3_MIN = 45.0            # 固件 J3 角下限（红线是 35°，这里留 10° 余量）
J5_MAX = 80.0
PORT_MAX_DEG = 50.0   # （仅提示用）出料口偏离"正下方"的角度
# 出料管是软管，主人确认：只要 J2 不处于收缩状态就接得住。
# J2 固件范围 -73°~+90°；实测舒展姿态在 +13~+30°，极限折叠到 +50 以上。
J2_CONTRACT_FW = 50.0   # 超过这个值视为"收缩"，拒绝
J6_NOW_FW = None      # 实测的 J6（固件度），所有规划一律以它为准

# 视觉坐标 → 机械臂真实位置的系统性偏差（由真实套入物理实验反算得到）
# 包含相机标定外参系统漂移、大臂负重下垂以及结构安装累计偏差
OFFSET_CFG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "pick_offset.json"
)

def load_pick_offset():
    try:
        with open(OFFSET_CFG_PATH, "r") as f:
            cfg = json.load(f)
            return np.array([cfg["offset_x_m"], cfg["offset_y_m"], cfg["offset_z_m"]], dtype=float)
    except Exception:
        return np.array([-0.0263, -0.0727, -0.0357], dtype=float)

AIM_OFFSET = load_pick_offset()


def log(msg):
    print(msg, flush=True)


def detect_fruit(use_yolo=True, conf_thresh=0.10):
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

    best = None
    # 优先尝试 YOLO 实例分割模型
    weights_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "dummyarm_AImodel/runs/segment/exp-001_yolo11n-seg_50e/weights/best.pt"
    )
    if use_yolo and os.path.exists(weights_path):
        try:
            from ultralytics import YOLO
            model = YOLO(weights_path)
            res = model(bgr, conf=conf_thresh, verbose=False)[0]
            if len(res.boxes) > 0 and res.masks is not None:
                # 寻找置信度最高或面积最大的 orange / occluded_orange
                for i, box in enumerate(res.boxes):
                    cls_id = int(box.cls[0])
                    cls_name = model.names[cls_id]
                    if cls_name not in ('orange', 'occluded_orange'):
                        continue
                    conf = float(box.conf[0])
                    mask = (res.masks.data[i].cpu().numpy() > 0.5).astype(np.uint8)
                    mask = cv2.resize(mask, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
                    vv = dep[mask > 0]
                    vv = vv[vv > 0]
                    if vv.size < 30:
                        continue
                    z = float(np.median(vv)) / 1000.0
                    if not (0.15 < z < 0.9):
                        continue
                    # 轮廓中心
                    M = cv2.moments(mask)
                    if M["m00"] <= 0:
                        continue
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    a = float(M["m00"])
                    if best is None or conf > best.get("conf", 0):
                        best = {
                            "pixel": (u, v), "depth": z, "area": a, "conf": conf,
                            "class": cls_name, "source": "YOLO", "mask": mask
                        }
                if best is not None:
                    log(f"   [YOLOv11-seg] 识别到目标: 类={best['class']}, 置信度={best['conf']:.2f}")
        except Exception as e:
            log(f"   YOLO 推理异常 ({e})，降级使用 HSV")

    # 若 YOLO 未检测到有效目标，降级为 HSV + 深度过滤分割保底
    if best is None:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([12, 60, 50]), np.array([40, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        
        # 将深度有效性直接融合到 mask 中，过滤远景噪点
        near_mask = (dep > 200) & (dep < 800)
        mask_filtered = (mask > 0) & near_mask
        
        cnts, _ = cv2.findContours(mask_filtered.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for ct in cnts:
            a = cv2.contourArea(ct)
            if a < 150:
                continue
            M = cv2.moments(ct)
            if M["m00"] <= 0:
                continue
            u = int(M["m10"] / M["m00"])
            v = int(M["m01"] / M["m00"])
            m2 = np.zeros_like(mask)
            cv2.drawContours(m2, [ct], -1, 255, -1)
            vv = dep[m2 > 0]
            vv = vv[(vv > 200) & (vv < 800)]
            if vv.size < 20:
                continue
            z = float(np.median(vv)) / 1000.0
            if best is None or a > best["area"]:
                (_, _), rpx = cv2.minEnclosingCircle(ct)
                best = {
                    "pixel": (u, v), "depth": z, "area": a, "conf": 1.0,
                    "class": "duck_target", "source": "HSV_Depth", "radius_px": float(rpx)
                }

    k = cam.k
    cam.destroy_node()
    rclpy.try_shutdown()
    if best is None:
        log("!! 没找到近距目标")
        return None

    u, v = best["pixel"]
    z = best["depth"]
    pc = cal.pixel_to_cam(u, v, z, k)
    T = np.array(json.load(open(CALIB))["T_base_link_camera"])
    pb = T[:3, :3] @ np.array(pc) + T[:3, 3]

    if "radius_px" in best:
        radius_m = best["radius_px"] * z / k[0]
    else:
        # 基于 mask 面积近似等效圆半径
        radius_m = math.sqrt(best["area"] / math.pi) * z / k[0]

    return {
        "pixel": (u, v), "depth": z, "p_base": pb, "radius": radius_m,
        "area": best["area"], "source": best["source"], "conf": best.get("conf", 1.0)
    }


def check_pose(target, seed, aim_at=None):
    """aim_at 非空时同时约束"套筒轴线指向 aim_at"（J5 就是套筒轴向）"""
    # J6 = 刀片轴：**参与解算**（保持解算自由度），但结果丢弃、绝不执行。
    # J6 只绕锥罩轴线转，丢弃它不影响 TCP 位置、也不影响锥罩轴向，
    # 只影响滚转 —— 所以丢弃后要额外检查"出料口是否仍然朝下"。
    if J6_NOW_FW is not None:
        seed = list(seed)
        seed[5] = cal.agx.fw_to_model_rad(
            [0.0, 0.0, 0.0, 0.0, 0.0, float(J6_NOW_FW)])[5]
    j6_lock = float(seed[5])
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
        # 必须用 6 自由度（完整姿态）解算，不能用只约束轴线的 5 自由度：
        # 5DOF 把滚转留给正则项，实测会掉进坏分支
        # （J1 转到 -169.6°、J3 压在 35° 红线、轴线还差 19.9°）。
        # 6DOF 直接指定"出料口朝下"的目标姿态，解干净且稳定。
        _Tdes = tool_frame_from_axis(d / n)
        _Tdes[:3, 3] = np.array(target)
        q, res, axis_err = solve_full_ik(_Tdes, seed=seed, rot_tol=0.06)
        if res > 6.0:
            return None, f"位姿IK 位置残差 {res:.1f}mm", axis_err
        if axis_err > AXIS_TOL_DEG:
            return None, f"轴线偏差 {axis_err:.1f}° 过大", axis_err
    q = list(q); q[5] = j6_lock           # 丢弃 J6 的解，用当前值
    fw = cal.agx.clamp_fw(cal.agx.model_rad_to_fw(q))
    if fw[1] > J2_CONTRACT_FW:
        return None, (f"J2={fw[1]:+.0f}° 处于收缩状态（> {J2_CONTRACT_FW:.0f}°），"
                      f"出料软管会挤到车体，拒绝该解"), axis_err
    if not (J3_MIN < fw[2] < 179):
        return None, f"J3={fw[2]:.1f}° 不安全", axis_err
    if abs(fw[4]) > J5_MAX:
        return None, f"J5={fw[4]:.1f}° 不安全", axis_err
    return (q, fw), None, axis_err


def move_to(ser, fw, speed=20.0, timeout=50.0, park=False):
    """下发一条关节目标并等到位。

    下发前必须过扫掠碰撞检查（末端 125mm 长、外廓 Ø172，扫到车架就是事故）。
    """
    import arm_safety as _safety
    _cur = seed_now(ser)
    if _cur is None:
        log("   !! 读不到当前姿态，无法做扫掠检查，拒绝下发")
        return None
    try:
        _safety.check_all(fk_full(_cur), fk_full(model_from_fw(fw)), park=park)
    except _safety.SafetyError as e:
        log(f"   !! 扫掠拦截: {e}")
        return None
    js = None
    for k in range(3):
        ser.write(">" + ",".join(f"{v:.3f}" for v in fw) + f",{speed:.1f}")
        js = cal.wait_motion(ser, fw, timeout=timeout)
        if js and max(abs(js[i] - fw[i]) for i in range(6)) < 2.0:
            return js
        log(f"   重发第 {k+2} 次")
        time.sleep(0.8)
    return js


def seed_now(ser):
    """从当前实测关节角取 IK 种子。

    绝不能用写死的初值：IK 从远离实际的姿态出发会掉进别的解分支
    （实测踩到过 J5 直接跳到 -89° 的情况）。
    """
    if ser is None:
        return None
    for _ in range(6):
        js = ser.joints()
        if js is not None:
            return model_from_fw(js)
        time.sleep(0.15)
    return None


def approach_axis():
    """进刀轴向：朝车外(-Y) + 向上的仰角

    果梗在上、果实在下，从斜下方往上套能把果实"兜"进锥罩，
    而不是从上方压下去把它推走。
    """
    el = math.radians(ELEV_DEG)
    return np.array([0.0, -math.cos(el), math.sin(el)])


def plan_pick(target_fruit_pos, current_joints_model=None):
    """规划单次套果路径与策略。
    
    返回字典：
      {
        "status": "STAGE_AND_FEED" | "DIRECT_FEED" | "ABORT",
        "reason": str,
        "stage_pose": (q, fw) or None,
        "feed_steps": [(cur_target_pos, q, fw, ae), ...],
        "tcp_goal": np.ndarray
      }
    
    核心决策原则（按用户指令）：
    1. 正常情况下：从预备位 (STAGE) 进刀到套果终点 (FEED)。
    2. 若果实太靠近车体，导致预备位不可达/扫掠碰撞，但直接进刀路径可行：
       主动放弃准备位，采用 DIRECT_FEED 直接套果。
    3. 若既不能到达准备位，又无法直接套果（超出臂展、死区、姿态受限）：
       返回 ABORT 主动放弃该果实，避免强行运动发生碰撞危险。
    """
    pf = np.asarray(target_fruit_pos, dtype=float)
    ax = approach_axis()
    tcp_goal = pf - (CONE_LEN_M - FEED_DEPTH_PAST_M) * ax
    stage_target = pf - STAGE_BACK_M * ax
    
    seed = current_joints_model
    
    # 1. 尝试评估预备位
    stage_pose, stage_err, stage_ae = check_pose(stage_target, seed=seed, aim_at=pf)
    
    # 2. 尝试评估直接到达终点
    goal_pose, goal_err, goal_ae = check_pose(tcp_goal, seed=seed, aim_at=pf)
    
    if stage_pose is not None and goal_pose is not None:
        # 预备位可用
        return {
            "status": "STAGE_AND_FEED",
            "reason": "预备位及套果终点均可达",
            "stage_target": stage_target,
            "stage_pose": stage_pose,
            "goal_pose": goal_pose,
            "tcp_goal": tcp_goal
        }
    
    if stage_pose is None and goal_pose is not None:
        # 预备位不可达（太靠近车架），但直接套果位姿可行
        return {
            "status": "DIRECT_FEED",
            "reason": f"预备位受限({stage_err})，但终点可达，主动放弃准备位直接进刀",
            "stage_target": None,
            "stage_pose": None,
            "goal_pose": goal_pose,
            "tcp_goal": tcp_goal
        }
        
    # 两者均不可达
    return {
        "status": "ABORT",
        "reason": f"准备位不可达({stage_err})且直接进刀不可达({goal_err})，安全放弃",
        "stage_target": None,
        "stage_pose": None,
        "goal_pose": None,
        "tcp_goal": tcp_goal
    }


def do_stage(ser, fruit, dry):
    pf = fruit["p_base"]
    ax = approach_axis()
    stage = pf - STAGE_BACK_M * ax
    log(f"STAGE 目标(TCP): X={stage[0]:+.3f} Y={stage[1]:+.3f} Z={stage[2]:+.3f}"
        f"   [仰角 {ELEV_DEG:.0f}°，罩口在 TCP 前方 {CONE_LEN_M*1000:.0f}mm，"
        f"离果心 {STAGE_GAP_M*1000:.0f}mm]")
    _seed = seed_now(ser)
    pose, err, ae = check_pose(stage, seed=_seed, aim_at=pf)
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
    """沿"进刀轴向"分步进刀（仰角方向）。

    罩口要落在果心 ⇒ TCP 目标 = 果心 - CONE_LEN * axis。
    全程沿同一条轴线推进，开口始终"迎面"对着果实，
    不会用锥壁去推它。
    """
    pf = fruit["p_base"]
    ax = approach_axis()
    # 罩口越过果心 FEED_DEPTH_PAST_M，把果实整个送进刀片平面内侧
    tcp_goal = pf - (CONE_LEN_M - FEED_DEPTH_PAST_M) * ax
    cur = stage.copy()
    step = 0
    seed = seed_now(ser)
    if seed is None:
        log("!! 读不到当前姿态，无法保证 IK 走同一分支，停止")
        return False
    while np.linalg.norm(tcp_goal - cur) > 0.002:
        step += 1
        if step > 24:
            log("   进刀步数超限，停止")
            return False
        d = tcp_goal - cur
        dist = np.linalg.norm(d)
        step_m = min(FEED_STEP_M, dist)
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
        pose, err, _ae = check_pose(tgt, seed=seed_now(ser))
        if pose is None:
            log(f"   {name}不可达（{err}），跳过")
            continue
        if dry:
            log(f"   (dry-run) {name}")
            continue
        js = move_to(ser, pose[1], speed=15.0)
        log(f"   {name}完成" if js is not None else f"   {name}失败")
    return True


def do_rest(ser, dry):
    """安全平滑回到 REST 折叠姿态"""
    log(">> 正在将机械臂安全平稳收拢回到 REST 姿态...")
    rest_fw = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
    if dry:
        log(f"   (dry-run) 回 REST: {rest_fw}")
        return True
    js = move_to(ser, rest_fw, speed=15.0, park=True)
    if js is not None and max(abs(js[i] - rest_fw[i]) for i in range(6)) < 3.0:
        log(">> 机械臂已成功安全到达 REST 折叠位！")
        return True
    else:
        log("!! 回 REST 未能完全到位或被拦截")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detect", action="store_true")
    ap.add_argument("--stage", action="store_true")
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--cut", action="store_true")
    ap.add_argument("--retreat", action="store_true")
    ap.add_argument("--rest", action="store_true", help="安全平滑回 REST 折叠位")
    ap.add_argument("--auto-cut", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--servo", action="store_true", help="实时视觉伺服进刀")
    ap.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="直接指定 base_link 下的果实坐标（跳过视觉检测，用于锥罩遮挡时）")
    ap.add_argument("--aim-offset", nargs=3, type=float, default=None,
                    help="手动覆盖补偿向量 [dx, dy, dz] (m)")
    args = ap.parse_args()

    if args.rest:
        fruit = {"pixel": (0, 0), "depth": 0.0, "p_base": np.array([0.0, 0.0, 0.0]), "radius": 0.0}
    elif args.target:
        pb = np.array(args.target, dtype=float)
        fruit = {"pixel": None, "depth": float(np.linalg.norm(pb)),
                 "p_base": pb, "radius": 0.022, "area": 0, "manual": True}
        log(f"使用手动指定目标（跳过视觉）: {pb}")
    else:
        fruit = detect_fruit()
        if fruit is None:
            return 1
    pf = fruit["p_base"]
    log(f"\n目标果实: 像素={fruit['pixel']} 深度={fruit['depth']*1000:.0f}mm "
        f"半径≈{fruit['radius']*1000:.1f}mm")
    log(f"  base_link(原始视觉): X={pf[0]:+.4f} Y={pf[1]:+.4f} Z={pf[2]:+.4f}")
    
    _off = AIM_OFFSET if args.aim_offset is None else np.array(args.aim_offset, dtype=float)
    if not args.rest:
        fruit["p_base"] = np.asarray(fruit["p_base"], dtype=float) + _off
        log(f"  应用真实物理补偿: ΔX={_off[0]*1000:+.1f}mm, ΔY={_off[1]*1000:+.1f}mm, ΔZ={_off[2]*1000:+.1f}mm")
        log(f"  → 实际目标物理坐标: ({fruit['p_base'][0]:+.4f},{fruit['p_base'][1]:+.4f},{fruit['p_base'][2]:+.4f})")
    if args.detect:
        return 0

    ser = cal.agx.RefSerial(cal.agx.PORT_CANDIDATES[0]
                            if os.path.exists(cal.agx.PORT_CANDIDATES[0])
                            else cal.agx.PORT_CANDIDATES[1])
    try:
        global J6_NOW_FW
        _j = ser.joints()
        if _j is not None:
            J6_NOW_FW = _j[5]
            log(f"J6 实测 {J6_NOW_FW:+.2f}°（本次所有规划都以此为准，不会动它）")
        ser.cmd("!START", 0.8)
        if args.rest:
            return 0 if do_rest(ser, args.dry_run) else 1
        if args.cut:
            return 0 if do_cut(ser, args.dry_run) else 1
        if args.retreat:
            # 如果提供了 fruit 则基于目标退刀，否则直接沿当前工具方向或安全回退
            return 0 if do_retreat(ser, fruit, args.dry_run) else 1

        stage = do_stage(ser, fruit, args.dry_run)
        if stage is None:
            # 评估是否可以放弃准备位直接进刀
            log(">> 准备位不可达，检查是否可直接进刀...")
            plan = plan_pick(fruit["p_base"], current_joints_model=seed_now(ser))
            if plan["status"] == "DIRECT_FEED":
                log(f">> 触发策略: {plan['reason']}")
                # 构造从当前位置直接进刀
                cur_tcp = fk_full(seed_now(ser))[:3, 3]
                fed = do_feed(ser, fruit, cur_tcp, args.dry_run)
            else:
                log(f"!! 无法安全进刀: {plan['reason']}")
                return 1
        elif args.stage:
            return 0
        else:
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
