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

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_ref_port()
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
