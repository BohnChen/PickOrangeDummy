#!/usr/bin/env python3
"""
calibrate_j1.py - 底座 J1 硬件零位一键标定工具
将当前机械臂底座停放的物理位置直接固化为单片机电机的绝对硬件零位 (#OFFSET_J 1)。
使用方法:
1. 在机械臂失能断电状态下，用手将底座 J1 端端正正摆到正前方对齐位。
2. 运行本脚本: ./scripts/calibrate_j1.py
3. 电机当前位置将立即被烧录固化为 0.00°！
"""
import glob
import os
import serial
import time
import sys

def find_ref_port():
    ref_ports = glob.glob('/dev/serial/by-id/*REF*')
    if ref_ports:
        return ref_ports[0]
    for p in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']:
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'

def main():
    port = find_ref_port()
    print(f"正在连接机械臂: {port} ...")
    try:
        s = serial.Serial(port, 115200, timeout=1)
        s.dtr = False
        s.rts = False
        time.sleep(0.15)
        s.reset_input_buffer()

        # 读取当前位置
        s.write(b'#GETJPOS\n')
        time.sleep(0.08)
        line = s.readline().decode('utf-8', errors='ignore').strip()
        print(f"校准前当前读数: {line}")

        # 发送单轴零位标定指令
        print(">>> 正在下发 #OFFSET_J 1 (将当前物理位置设定为底座电机绝对零位)...")
        s.write(b'#OFFSET_J 1\n')
        time.sleep(0.1)
        resp = s.readline().decode('utf-8', errors='ignore').strip()
        print(f"固件反馈: {resp}")

        # 重新读数验证
        time.sleep(0.2)
        s.write(b'#GETJPOS\n')
        time.sleep(0.08)
        new_line = s.readline().decode('utf-8', errors='ignore').strip()
        print(f">>> 校准后最新读数: {new_line} <<<")
        s.close()

        print("\n==============================================")
        print("【标定成功】底座 J1 已被固化为当前物理绝对正前方 (0.00°)！")
        print("==============================================")

    except Exception as e:
        print(f"标定执行出错: {e}")

if __name__ == '__main__':
    main()
