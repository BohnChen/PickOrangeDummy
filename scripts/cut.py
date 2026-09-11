#!/usr/bin/env python3
"""
cut.py - 采摘旋切切削工具
指挥第 6 个电机 (J6) 旋转一圈 (360度) 完成果梗切削，然后复位。
"""
import sys
import time
import serial
import glob
import os

def find_ref_port():
    ref_ports = glob.glob('/dev/serial/by-id/*REF*')
    if ref_ports:
        return os.path.realpath(ref_ports[0])
    for p in ['/dev/ttyACM0', '/dev/ttyACM1']:
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_ref_port()
    print(f"正在通过 {port} 执行果梗切削动作 (J6 旋转 360°)...")
    try:
        # 底层非阻塞发送
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        
        # 旋转 360 度切削
        os.write(fd, b'>0,0,0,0,0,360,50\n')
        time.sleep(1.5)
        
        # 复位
        os.write(fd, b'>0,0,0,0,0,0,50\n')
        time.sleep(0.5)
        os.close(fd)
        print(">>> 采摘果梗切削动作完成！<<<")
    except Exception as e:
        print(f"切削执行异常: {e}")

if __name__ == '__main__':
    main()
