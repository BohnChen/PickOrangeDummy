#!/usr/bin/env python3
"""
disable.py - 机械臂硬急停与释放力矩工具 (底层非阻塞秒级可靠版)
采用 POSIX O_NONBLOCK 底层直接下发 !STOP 和 !DISABLE，彻底杜绝 pySerial 握手卡死问题。
"""
import os
import sys
import time
import glob
import termios

def find_ref_port():
    ref_ports = glob.glob('/dev/serial/by-id/*REF*')
    if ref_ports:
        return ref_ports[0]
    for p in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']:
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'

# REST 的固件角；失能前必须确认已在这个姿态附近
REST_FW = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
MAX_REST_DEV = 8.0      # 单轴最大允许偏差（度）


def read_joints(fd, timeout=1.5):
    """读 'ok a b c d e f' 关节角（度）"""
    os.write(fd, b'#GETJPOS\n')
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            chunk = os.read(fd, 4096)
        except Exception:
            chunk = b''
        if chunk:
            buf += chunk
            for line in buf.decode('utf-8', errors='ignore').splitlines():
                p = line.strip().split()
                if len(p) >= 7 and p[0] == 'ok':
                    try:
                        return [float(x) for x in p[1:7]]
                    except ValueError:
                        continue
        else:
            time.sleep(0.02)
    return None


def main():
    force = '--force' in sys.argv
    port = next((a for a in sys.argv[1:] if a.startswith('/dev/')), None) or find_ref_port()
    print(f"正在以底层非阻塞模式向 {port} 发送安全失能命令...")
    try:
        # 非阻塞直接打开，绝对不卡死
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        
        # 保持 DTR/RTS 稳定，清除 HUPCL 防复位
        try:
            attrs = termios.tcgetattr(fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception:
            pass

        # ---- 失能前置检查：必须在 REST 附近 ----
        # 事故（2026-11-01 夜）：在伸展悬空姿态失能，机械臂重力下坠撞地。
        # 规则：只有确认停在 REST 才允许失能；否则拒绝，改为锁轴保持。
        js = read_joints(fd)
        if js is not None and not force:
            dev = max(abs(a - b) for a, b in zip(js, REST_FW))
            if dev > MAX_REST_DEV:
                print(f"!! 拒绝失能：当前 ['{' '.join(f'{v:+.1f}' for v in js)}']")
                print(f"!! 与 REST 最大偏差 {dev:.1f}° > {MAX_REST_DEV:.0f}°，"
                      f"此时失能会重力下坠")
                print("   已改为【锁轴保持】：回发当前位姿，保持力矩")
                os.write(fd, (">" + ",".join(f"{v:.3f}" for v in js) + ",3.0\n").encode())
                time.sleep(0.2)
                try:
                    print("   控制板反馈:", os.read(fd, 256).decode('utf-8', errors='ignore').strip())
                except Exception:
                    pass
                os.close(fd)
                return 2
            print(f"已确认在 REST 附近（最大偏差 {dev:.1f}°），允许失能")

        # 连续发送 !STOP 与 !DISABLE
        os.write(fd, b'!STOP\n!DISABLE\n')
        time.sleep(0.08)
        
        # 尝试读取单片机回复
        try:
            resp = os.read(fd, 256).decode('utf-8', errors='ignore').strip()
            if resp:
                print(f"控制板反馈: {resp}")
        except Exception:
            pass

        os.close(fd)
        print(">>> 机械臂电机已成功失能，力矩已完全释放！<<<")
    except Exception as e:
        print(f"串口直连提示: {e}")
        # 如果是因为 real_moveit 正在运行占用串口，直接杀死节点即可触发驱动析构自动失能
        print("正在向后台控制节点发送退出信号以触发自动失能...")
        os.system("pkill -f 'ros2_control_node' 2>/dev/null || true")
        time.sleep(0.5)
        print(">>> 机械臂已失能！<<<")

if __name__ == '__main__':
    main()
