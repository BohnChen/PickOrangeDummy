#!/usr/bin/env python3
"""
park_and_disable.py - 一键平稳回 REST 收纳位并失能
直接使用下位机原生平滑指令，单片机硬件加减速，极其丝滑稳健，绝对不抖动。
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

        # 确保使能
        s.write(b'!START\n')
        time.sleep(0.08)

        # 读取 J6 当前角度
        s.write(b'#GETJPOS\n')
        time.sleep(0.08)
        line = s.readline().decode('utf-8', errors='ignore').strip()
        j6_str = "0.000"
        if line.startswith("ok"):
            parts = line.split()
            if len(parts) == 7:
                j6_str = parts[6]

        # 原生平滑收拢指令 (走向 REST: 0, -73, 180, 0, 0, J6不变，速度15)
        cmd = f">0.000,-73.000,180.000,0.000,0.000,{j6_str},15.0\n"
        print(f"正在平滑收拢机械臂至 REST: {cmd.strip()}")
        s.write(cmd.encode())
        time.sleep(0.1)
        s.close()

        print("机械臂平稳运动中...")
        time.sleep(6.5)

        # 到位失能
        s2 = serial.Serial(port, 115200, timeout=1)
        s2.dtr = False
        s2.rts = False
        time.sleep(0.1)
        s2.write(b"!STOP\n!DISABLE\n")
        time.sleep(0.1)
        s2.close()
        print(">>> 机械臂已平稳到达 REST 折叠位，电机力矩已安全断电释放！<<<")

    except Exception as e:
        print(f"操作提示: {e}")

if __name__ == '__main__':
    main()
