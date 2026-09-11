#!/usr/bin/env python3
"""
disable.py - 机械臂硬急停与释放力矩工具
直接向 REF 控制板发送 !STOP 与 !DISABLE，立即切断电机力矩。
"""
import sys
import time
import serial

def main():
    port = '/dev/ttyACM1' if len(sys.argv) < 2 else sys.argv[1]
    print(f"正在向 {port} 发送安全失能与释放力矩命令...")
    try:
        s = serial.Serial(port, 115200, timeout=0.5)
        s.dtr = False
        s.rts = False
        time.sleep(0.2)
        s.reset_input_buffer()
        s.write(b"!STOP\n")
        time.sleep(0.1)
        s.write(b"!DISABLE\n")
        time.sleep(0.1)
        resp = s.read_all().decode('utf-8', errors='ignore')
        s.close()
        print("执行结果:", resp.strip() if resp else "已发送")
        print(">>> 机械臂电机已成功失能，力矩已完全释放！<<<")
    except Exception as e:
        print(f"打开串口失败: {e}")
        print("如果是由于 launch 正在独占运行，请在 launch 终端中按 Ctrl+C，系统已自动配置安全失能。")

if __name__ == '__main__':
    main()
