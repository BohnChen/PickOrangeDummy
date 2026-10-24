#!/usr/bin/env python3
"""
j6_homing.py - J6 刀片「全开限位」自整定（每次上电后必须执行）

为什么必须这样做（2026-10-24 实测 + 固件源码确证）:
    电机固件 dummy-42motor-fw/Ctrl/Motor/motor.cpp 上电首次闭环时：
        realLapPosition = encoder->angleData.rectifiedAngle;   // 多圈计数直接重建
    多圈位置**从不写 EEPROM**。J6 减速比 50:1，所以每次上电后 J6 读数只能在
    ±3.6°（半个电机圈）内，与真实物理角度无关。
    实测：断电前 J6 = -20.00°，重新上电后读数 = +2.99°。

⚠ 2026-10-24 实测补充：**靠堵转顶限位不可靠** —— 顶到全开限位时 J6 驱动器会因
   堵转电流复位，多圈位置再次丢失（读数从 +27.4° 跳回 +1.2°，且期间 CAN 读数连续丢失）。
   因此本脚本仅保留作诊断用途，正式流程改为：「上电前把刀片放到全开位，
   以启动时读数作为本次会话 home」，并且绝不主动顶限位。

原设计（已停用）：每次上电后用**物理限位**重新建立参考：
    低速朝「全开方向(+ 方向)」逐级前进 → 连续两次读数不动 = 顶到全开限位 → 记为 home。

用法:
    python3 scripts/j6_homing.py            # 执行自整定并写入 config/j6_blade.json
    python3 scripts/j6_homing.py --dry-run  # 只读当前值，不动
"""

import os
import sys
import json
import glob
import time
import termios
import datetime

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'config', 'j6_blade.json')

STEP = 2.0          # 每级前进角度
SPEED = 2.0         # 固件速度参数（约 1.4°/s）
STALL_TOL = 0.15    # 判定"没动"的阈值
SETTLE = 0.8        # 每级等待上限
MAX_TRAVEL = 65.0   # 最大搜索行程，超过即判定"没找到限位"并中止


def find_ref_port():
    ports = glob.glob('/dev/serial/by-id/*REF*')
    if ports:
        return ports[0]
    for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'):
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'


def load_config():
    try:
        with open(CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def save_home(home_deg, span=60.0):
    cfg = load_config()
    cfg.update({
        'j6_home_deg': round(home_deg, 2),
        'cut_span_deg': span,
        'measured_at': datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
        'note': '每次上电后由 j6_homing 顶到全开限位重新整定；J6 多圈位置不跨上电保存',
    })
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, 'w') as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
        f.write('\n')
    return cfg


def home_j6(ser, log=print):
    """把 J6 低速开到全开限位，返回该处的读数(本次会话的 home)。失败返回 None。

    ser 需要提供: joints(timeout=None) -> [6 floats] 或 None, write(str)
    """
    start = ser.joints(0.4)
    if not start:
        log('[归位] 读不到关节角，中止')
        return None
    base = start[5]
    log(f'[归位] 起点 J6 = {base:+.2f}°，朝全开(+)方向逐级前进，最大 {MAX_TRAVEL:.0f}°')

    target = base
    last = base
    stall = 0
    none_count = 0
    traveled = 0.0

    while traveled < MAX_TRAVEL:
        target += STEP
        traveled += STEP
        ser.write(f'>{start[0]:.2f},{start[1]:.2f},{start[2]:.2f},'
                  f'{start[3]:.2f},{start[4]:.2f},{target:.2f},{SPEED:.0f}')
        t0 = time.time()
        cur = None
        while time.time() - t0 < SETTLE:
            time.sleep(0.1)
            j = ser.joints(0.25)
            if j:
                cur = j
                if abs(j[5] - target) <= 0.15:
                    break
        if cur is None:
            none_count += 1
            log(f'[归位] 读数缺失 ({none_count})')
            if none_count >= 5:
                log('[归位] 连续读不到关节角，中止（避免盲目继续前进）')
                return None
            continue
        none_count = 0
        moved = abs(cur[5] - last)
        if moved < STALL_TOL:
            stall += 1
            log(f'[归位] {cur[5]:+.2f}° 无位移 (累计 {stall})')
            if stall >= 2:
                log(f'[归位] 已顶到全开限位: J6 = {cur[5]:+.2f}°')
                return cur[5]
        else:
            stall = 0
            log(f'[归位] {cur[5]:+.2f}°')
        last = cur[5]
        time.sleep(0.1)

    log(f'[归位] 走了 {MAX_TRAVEL:.0f}° 仍未遇到限位，中止（请人工检查机械限位）')
    return None


def main():
    dry = '--dry-run' in sys.argv
    from jog_gui import RefSerial  # 复用同一套串口实现

    ser = RefSerial(find_ref_port())
    try:
        if dry:
            j = ser.joints(0.4)
            print('当前读数:', [round(x, 2) for x in j] if j else None)
            print('配置:', load_config())
            return 0

        ser.write('!START')
        time.sleep(0.5)
        home = home_j6(ser)
        if home is None:
            print('[失败] 未完成归位，配置未修改')
            return 1
        cfg = save_home(home)
        print(f'[完成] 已写入 {CONFIG_PATH}')
        print(f'       j6_home_deg = {cfg["j6_home_deg"]:+.2f}°，'
              f'剪切区间 [{cfg["j6_home_deg"] - cfg["cut_span_deg"]:+.2f}, {cfg["j6_home_deg"]:+.2f}]')
        return 0
    finally:
        ser.close()


if __name__ == '__main__':
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    raise SystemExit(main())
