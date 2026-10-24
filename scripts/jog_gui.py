#!/usr/bin/env python3
"""
jog_gui.py - Dummy 机械臂轻量拖动窗口（J1~J5 可拖，J6 锁定）

用途:
    用鼠标拖 J1~J5，把机械臂摆到目标姿态（例如 START/HOME），并单独控制刀片合/开。

安全约定（见 docs/M2_spec_j6_blade_home.md §4.3）:
    1. J6 **不可拖动**：只显示相对角，且只有「合刀 -60°」「回位 0°」两个按钮；
    2. 静止检测: 目标与实测差 < 0.02° 不下发（规避固件 MoveJ 的 0/0 = NaN 速度限幅）；
    3. 所有目标先按固件区间夹紧（J3 红线 35°、J5 ±95° 等）；
    4. ros2_control 在跑时拒绝启动（串口独占）；
    5. 急停按钮 / 空格 = 固件 !STOP（锁位 + 清 FIFO + 失能），且只在用户主动触发时发送；
    6. 关窗 = 保持当前姿态悬停，不自动失能、不自动回位。

用法:
    python3 scripts/jog_gui.py
    python3 scripts/jog_gui.py --home -1.2     # 指定 J6 零位（默认 0.0）
    python3 scripts/jog_gui.py --port /dev/ttyACM0
"""

import os
import glob
import time
import threading
import termios
import argparse
import subprocess

import tkinter as tk
from tkinter import ttk, messagebox

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

# 固件目标角校验区间（J6 保持 ±720 多圈，仅剪切层另有限制）
FW_RANGES = [
    (-170.0, 170.0), (-75.0, 90.0), (35.0, 180.0),
    (-180.0, 180.0), (-95.0, 95.0), (-720.0, 720.0),
]

HOME_POSE = [0.0, 0.0, 90.0, 0.0, 0.0, None]   # None = J6 保持当前零位
JOG_RATE = 25.0        # 拖动限速 (deg/s)
CUT_RATE = 6.0         # 刀片合/开速度 (deg/s)
CUT_SPAN = 60.0        # 剪切行程 (deg)
SEND_HZ = 20.0
READ_HZ = 50.0


def find_ref_port():
    ports = glob.glob('/dev/serial/by-id/*REF*')
    if ports:
        return ports[0]
    for p in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2'):
        if os.path.exists(p):
            return p
    return '/dev/ttyACM0'


def ros_control_running():
    try:
        out = subprocess.run(['pgrep', '-af', 'ros2_control_node|controller_manager'],
                             capture_output=True, text=True, timeout=3)
        return out.stdout.strip()
    except Exception:
        return ''


class RefSerial:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(self.fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        except Exception:
            pass
        time.sleep(0.15)
        self.drain()

    def drain(self):
        try:
            while os.read(self.fd, 4096):
                pass
        except Exception:
            pass

    def write(self, line):
        if not line.endswith('\n'):
            line += '\n'
        try:
            os.write(self.fd, line.encode())
        except BlockingIOError:
            self.drain()
            try:
                os.write(self.fd, line.encode())
            except OSError:
                pass
        except OSError:
            pass

    def read_text(self, timeout=0.1):
        buf = b''
        end = time.time() + timeout
        while time.time() < end:
            try:
                chunk = os.read(self.fd, 4096)
                if chunk:
                    buf += chunk
                    end = time.time() + 0.01
            except Exception:
                time.sleep(0.003)
        return buf.decode('utf-8', errors='ignore')

    def joints(self):
        self.drain()
        self.write('#GETJPOS')
        for l in self.read_text(0.12).splitlines():
            l = l.strip()
            if l.startswith('ok'):
                parts = l.split()
                if len(parts) >= 7:
                    try:
                        return [float(x) for x in parts[1:7]]
                    except ValueError:
                        pass
        return None

    def close(self):
        try:
            termios.tcdrain(self.fd)
        except Exception:
            pass
        try:
            os.close(self.fd)
        except Exception:
            pass


def clamp(v, i):
    lo, hi = FW_RANGES[i]
    return max(lo, min(hi, v))


class ArmWorker(threading.Thread):
    """串口独占线程：读 50Hz、发 20Hz，UI 线程只读共享状态。"""

    def __init__(self, port, j6_home):
        super().__init__(daemon=True)
        self.ser = RefSerial(port)
        self.j6_home = j6_home
        self.j6_range = (j6_home - CUT_SPAN, j6_home)
        self.lock = threading.Lock()
        self.alive = True
        self.meas = None
        self.cmd = None
        self.target = None
        self.last_sent = None
        self.enabled = False
        self.event = None            # 'estop' / 'home' / 'cut' / 'uncut' / 'arm' / 'disarm'
        self.status = '已连接，未使能'
        self.msg = ''
        self.sync = True             # True = 让 UI 把滑条同步到实测/目标

    # ---------- UI -> worker ----------
    def set_target(self, idx, value):
        with self.lock:
            if self.target is None:
                return
            self.target[idx] = clamp(value, idx)

    def set_j6_target(self, value):
        with self.lock:
            if self.target is None:
                return
            lo, hi = self.j6_range
            self.target[5] = max(lo, min(hi, value))

    def request(self, what):
        with self.lock:
            self.event = what

    def set_enabled(self, flag):
        with self.lock:
            self.enabled = flag

    def snapshot(self):
        with self.lock:
            sync = self.sync
            self.sync = False
            return (list(self.meas) if self.meas else None,
                    list(self.target) if self.target else None,
                    self.enabled, self.status, self.msg, self.j6_range, sync)

    # ---------- 线程主体 ----------
    def run(self):
        # 遵守架构安全约定: 进程启动不自动使能、不自动运动
        meas = self.ser.joints()
        with self.lock:
            self.meas = meas
            self.cmd = list(meas) if meas else None
            self.target = list(meas) if meas else None
            self.enabled = False
            self.status = '已连接（未使能，请点「使能」）'
            self.sync = True

        last_send = 0.0
        last_read = 0.0
        while self.alive:
            now = time.time()

            ev = None
            with self.lock:
                if self.event:
                    ev, self.event = self.event, None

            if ev == 'estop':
                self.ser.write('!STOP')
                self.ser.read_text(0.3)
                with self.lock:
                    self.enabled = False
                    self.status = '急停：!STOP（已锁位并失能）'
                continue

            if ev == 'disarm':
                self.ser.write('!DISABLE')
                self.ser.read_text(0.3)
                with self.lock:
                    self.enabled = False
                    self.status = '已失能（!DISABLE）'
                continue

            if ev == 'arm':
                self.ser.write('!START')
                self.ser.read_text(0.3)
                j = self.ser.joints()
                with self.lock:
                    if j:
                        self.meas = j
                        self.cmd = list(j)          # 使能瞬间把目标对齐实测，绝不产生突跳
                        self.target = list(j)
                    self.enabled = True
                    self.last_sent = None
                    self.sync = True
                    self.status = '已使能（!START）'
                continue

            if ev == 'home':
                with self.lock:
                    for i in range(5):
                        self.target[i] = HOME_POSE[i]
                    self.target[5] = self.j6_home
                    self.status = '回到 HOME (0,0,90,0,0)'

            if ev == 'cut':
                with self.lock:
                    self.target[5] = self.j6_range[0]
                    self.status = f'合刀 -> {self.j6_range[0]:+.0f}°'

            if ev == 'uncut':
                with self.lock:
                    self.target[5] = self.j6_home
                    self.status = f'回位 -> {self.j6_home:+.0f}°'
            # 读状态 50Hz
            if now - last_read >= 1.0 / READ_HZ:
                j = self.ser.joints()
                if j:
                    with self.lock:
                        self.meas = j
                last_read = now

            # 20Hz 下发
            if now - last_send >= 1.0 / SEND_HZ:
                should_send = False
                with self.lock:
                    if self.enabled and self.meas and self.cmd and self.target:
                        dt = now - last_send if last_send else 1.0 / SEND_HZ
                        dt = max(min(dt, 0.2), 0.01)
                        for i in range(6):
                            rate = CUT_RATE if i == 5 else JOG_RATE
                            step = rate * dt
                            d = self.target[i] - self.cmd[i]
                            if abs(d) <= step:
                                self.cmd[i] = self.target[i]
                            else:
                                self.cmd[i] += step if d > 0 else -step
                            self.cmd[i] = clamp(self.cmd[i], i)

                        if self.last_sent is None or any(
                                abs(self.cmd[i] - self.last_sent[i]) >= 0.02
                                for i in range(6)):
                            should_send = True
                            payload = (f'>{self.cmd[0]:.2f},{self.cmd[1]:.2f},{self.cmd[2]:.2f},'
                                       f'{self.cmd[3]:.2f},{self.cmd[4]:.2f},{self.cmd[5]:.2f},'
                                       f'{JOG_RATE:.0f}')
                if should_send:
                    self.ser.write(payload)
                    self.ser.read_text(0.02)
                    self.last_sent = list(self.cmd)
                last_send = now

            time.sleep(0.004)

        # 退出: 若仍使能则保持当前姿态悬停（不再下发新目标）
        try:
            with self.lock:
                still_enabled = self.enabled
                j = list(self.meas) if self.meas else None
            if still_enabled and j:
                self.ser.write(f'>{j[0]:.2f},{j[1]:.2f},{j[2]:.2f},'
                               f'{j[3]:.2f},{j[4]:.2f},{j[5]:.2f},{JOG_RATE:.0f}')
                self.ser.read_text(0.1)
        except Exception:
            pass
        self.ser.close()


class JogApp:
    def __init__(self, root, worker):
        self.root = root
        self.worker = worker
        root.title('Dummy 机械臂 · Jog（J6 刀片锁定）')
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        root.bind('<space>', lambda e: self.estop())

        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky='ew')
        ttk.Button(top, text='使能 !START', command=lambda: worker.request('arm')).pack(side='left', padx=3)
        ttk.Button(top, text='失能 !DISABLE', command=lambda: worker.request('disarm')).pack(side='left', padx=3)
        ttk.Button(top, text='回 HOME', command=lambda: worker.request('home')).pack(side='left', padx=3)
        ttk.Button(top, text='急停 (空格)', command=self.estop).pack(side='left', padx=12)

        body = ttk.Frame(root, padding=8)
        body.grid(row=1, column=0, sticky='ew')

        self.sliders = []
        self.labels = []
        for i in range(6):
            ttk.Label(body, text=JOINT_NAMES[i], width=4).grid(row=i, column=0, sticky='w')
            var = tk.DoubleVar(value=0.0)
            lo, hi = FW_RANGES[i]
            s = ttk.Scale(body, from_=lo, to=hi, orient='horizontal', length=420)
            s.grid(row=i, column=1, padx=6, pady=2, sticky='ew')
            lab = ttk.Label(body, text='--', width=18)
            lab.grid(row=i, column=2, sticky='w')
            self.sliders.append(s)
            self.labels.append(lab)
            if i < 5:
                s.configure(command=lambda v, idx=i: self.on_jog(idx, v))
            else:
                s.state(['disabled'])

        j6box = ttk.LabelFrame(root, text='J6 刀片（锁定，仅这两个动作）', padding=6)
        j6box.grid(row=2, column=0, sticky='ew', padx=8, pady=4)
        ttk.Button(j6box, text='合刀 (-60°)', command=lambda: worker.request('cut')).pack(side='left', padx=4)
        ttk.Button(j6box, text='回位 (0°)', command=lambda: worker.request('uncut')).pack(side='left', padx=4)
        self.j6_label = ttk.Label(j6box, text='相对角: --')
        self.j6_label.pack(side='left', padx=16)

        self.status = ttk.Label(root, text='初始化...', anchor='w')
        self.status.grid(row=3, column=0, sticky='ew', padx=10, pady=(0, 8))

        body.columnconfigure(1, weight=1)
        self.pulling = False
        self.root.after(100, self.refresh)

    def on_jog(self, idx, value):
        if self.pulling:
            return
        self.worker.set_target(idx, float(value))

    def estop(self):
        self.worker.request('estop')
        self.worker.set_enabled(False)

    def refresh(self):
        meas, target, enabled, status, msg, j6_range, sync = self.worker.snapshot()
        if meas:
            self.pulling = True
            for i in range(6):
                if sync and target:
                    lo, hi = FW_RANGES[i]
                    self.sliders[i].set(max(lo, min(hi, target[i])))
                self.labels[i].configure(text=f'{meas[i]:8.2f}°')
            self.pulling = False
            rel = meas[5] - self.worker.j6_home
            self.j6_label.configure(
                text=f'相对角 {rel:+7.2f}°  (绝对 {meas[5]:+8.2f}°, '
                     f'行程 [{j6_range[0]:+.0f}, {j6_range[1]:+.0f}])')
        self.status.configure(text=f'{status}   |   {"使能" if enabled else "失能"}   |   {msg}')
        if self.worker.alive:
            self.root.after(100, self.refresh)

    def on_close(self):
        self.worker.alive = False
        self.worker.join(timeout=1.5)
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description='Dummy 机械臂 Jog 窗口')
    ap.add_argument('--home', type=float, default=0.0, help='J6 零位（默认 0.0，零位固化后即 0）')
    ap.add_argument('--port', type=str, default='', help='串口（默认自动查找 REF）')
    args = ap.parse_args()

    busy = ros_control_running()
    if busy:
        root0 = tk.Tk()
        root0.withdraw()
        messagebox.showerror('串口被占用',
                             '检测到 ros2_control / controller_manager 正在运行：\n\n'
                             + busy.splitlines()[0] +
                             '\n\n请先停止 ROS 控制栈，再启动本窗口。')
        root0.destroy()
        return 2

    port = args.port or find_ref_port()
    worker = ArmWorker(port, args.home)
    worker.start()

    root = tk.Tk()
    JogApp(root, worker)
    root.mainloop()
    worker.alive = False
    worker.join(timeout=1.5)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
