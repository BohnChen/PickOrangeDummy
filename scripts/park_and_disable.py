#!/usr/bin/env python3
"""
park_and_disable.py - 一键安全收纳并失能工具 (智能自适应版)
自动检测环境：
- 若 ROS 2 节点正在运行: 通过 ROS 2 Action 安全驱动到位并确认后失能
- 若 ROS 2 未运行: 通过串口原生平滑指令驱动到位后失能
"""
import sys
import os

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sh_path = os.path.join(script_dir, "safe_park_and_exit.sh")
    ret = os.system(f"bash {sh_path}")
    sys.exit(ret)

if __name__ == '__main__':
    main()
