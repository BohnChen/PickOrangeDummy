#!/usr/bin/env python3
"""
jog_gui.py - Dummy 机械臂轻量拖动窗口（J1~J5 可拖，J6 锁定）

用途:
    用鼠标拖 J1~J5 把机械臂摆到目标姿态，并单独控制刀片合/开。

安全约定（见 docs/M2_spec_j6_blade_home.md §4.3）:
    1. J6 不可拖动：只显示相对角，仅「合刀 -60°」「回位 0°」两个按钮；
    2. 静止检测: 目标与上一条下发差 < 0.02° 不下发（规避固件 MoveJ 的 0/0 = NaN 速度限幅）；
    3. 所有目标先按固件区间夹紧（J3 红线 35°、J5 ±95° 等）；
    4. ros2_control 在跑时拒绝启动（串口独占）；
    5. 急停按钮 / 空格 = 固件 !STOP（锁位 + 清 FIFO + 失能），只在用户主动触发时发送；
    6. 关窗 = 保持当前姿态悬停，不自动失能、不自动回位；
    7. 启动不自动使能（architecture.md §8 上电不自动使能、不自动运动）。

v2 修复（2026-10-24 首轮真机测试后）:
    - 读数曾长期不刷新：`>` 指令的回执（"15" / "ok"）与 `#GETJPOS` 的 "ok a b c d e f"
      混在同一接收流里，旧实现只读固定时间窗，容易抓到不含应答的一批数据。
      现改为「按行严格匹配 7 字段 ok 行，超时重试」，读数不再假死。
    - 使能瞬间用旧读数初始化目标（导致 J5 被带偏）：现在 !START 后先等待 200ms 再取读数，
      取不到有效读数就保持"未使能"状态并提示，绝不用陈旧数据下发。
    - 增加「回 REST」按钮、发送计数与最后一条指令显示，便于现场判断链路是否在动。

用法:
    python3 scripts/jog_gui.py
    python3 scripts/jog_gui.py --home -1.2     # 指定 J6 零位（默认 0.0）
"""

import os
import glob
import time
import threading
import termios
import argparse
import subprocess
import traceback

import tkinter as tk
from tkinter import ttk, messagebox

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

FW_RANGES = [
    (-170.0, 170.0), (-75.0, 90.0), (35.0, 180.0),
    (-180.0, 180.0), (-95.0, 95.0), (-720.0, 720.0),
]

HOME_POSE = [0.0, 0.0, 90.0, 0.0, 0.0]      # 固件 !HOME   (J6 保持零位)
REST_POSE = [0.0, -73.0, 180.0, 0.0, 0.0]   # 固件 !RESET  (J6 保持零位)

JOG_RATE = 25.0        # 拖动限速 (deg/s)
CUT_RATE = 6.0         # 刀片合/开速度 (deg/s)
CUT_SPAN = 60.0        # 剪切行程 (deg)
SEND_PERIOD = 0.05     # 下发周期 (s)
READ_TIMEOUT = 0.20    # 单次 #GETJPOS 等待上限 (s)


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

    def joints(self, timeout=READ_TIMEOUT):
        """查询 6 关节角。只接受 'ok x x x x x x' 行，其它回执（15 / ok）一律忽略。"""
        self.drain()
        self.write('#GETJPOS')
        buf = ''
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                chunk = os.read(self.fd, 4096)
            except Exception:
                chunk = b''
            if chunk:
                buf += chunk.decode('utf-8', errors='ignore')
                for line in buf.splitlines():
                    parts = line.strip().split()
                    if len(parts) >= 7 and parts[0] == 'ok':
                        try:
                            return [float(x) for x in parts[1:7]]
                        except ValueError:
                            continue
                buf = buf[-200:]
            else:
                time.sleep(0.004)
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
    """串口独占线程：读姿态 + 限速下发。UI 线程只读写共享状态。"""

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
        self.event = None
        self.status = '已连接，未使能'
        self.msg = ''
        self.sync = True
        self.send_count = 0
        self.last_cmd = ''
        self.error = ''

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

    def resync(self):
        """把目标重新对齐实测（拖动异常或想放弃当前指令时用）"""
        with self.lock:
            if self.meas:
                self.target = list(self.meas)
                self.cmd = list(self.meas)
                self.sync = True
                self.status = '目标已重新对齐实测'

    def snapshot(self):
        with self.lock:
            sync = self.sync
            self.sync = False
            return (list(self.meas) if self.meas else None,
                    list(self.target) if self.target else None,
                    self.enabled, self.status, self.msg, self.j6_range,
                    sync, self.send_count, self.last_cmd, self.error)

    # ---------- worker 内部工具 ----------
    def _read_valid(self, timeout=READ_TIMEOUT, retries=3):
        for _ in range(retries):
            j = self.ser.joints(timeout)
            if j:
                return j
            time.sleep(0.03)
        return None

    def _seed_from_meas(self, status_text):
        j = self._read_valid()
        if not j:
            return False
        with self.lock:
            self.meas = j
            self.cmd = list(j)
            self.target = list(j)
            self.last_sent = None
            self.sync = True
            self.enabled = True
            self.status = status_text
        return True

    # ---------- 线程主体 ----------
    def run(self):
        try:
            # 启动只读，不使能、不发运动指令
            j = self._read_valid()
            with self.lock:
                self.meas = j
                if j:
                    self.cmd = list(j)
                    self.target = list(j)
                self.status = '已连接（未使能，请点「使能」）' if j else '已连接，但读不到关节角'
                self.sync = True

            while self.alive:
                ev = None
                with self.lock:
                    if self.event:
                        ev, self.event = self.event, None

                if ev == 'estop':
                    self.ser.write('!STOP')
                    time.sleep(0.05)
                    self.ser.drain()
                    with self.lock:
                        self.enabled = False
                        self.status = '急停：!STOP（已锁位并失能）'
                        self.last_sent = None
                    continue

                if ev == 'disarm':
                    self.ser.write('!DISABLE')
                    time.sleep(0.05)
                    self.ser.drain()
                    with self.lock:
                        self.enabled = False
                        self.status = '已失能（!DISABLE）'
                        self.last_sent = None
                    continue

                if ev == 'arm':
                    self.ser.write('!START')
                    time.sleep(0.05)
                    self.ser.drain()
                    time.sleep(0.15)            # 等 REF 控制环把新姿态反馈上来
                    if self._seed_from_meas('已使能（!START）'):
                        pass
                    else:
                        with self.lock:
                            self.enabled = False
                            self.status = '使能后读不到关节角，已停止下发'
                    continue

                if ev in ('home', 'rest'):
                    with self.lock:
                        if self.target is None:
                            continue
                        pose = HOME_POSE if ev == 'home' else REST_POSE
                        for i in range(5):
                            self.target[i] = pose[i]
                        self.target[5] = self.j6_home
                        self.status = '回 HOME (0,0,90,0,0)' if ev == 'home' else '回 REST (0,-73,180,0,0)'

                if ev == 'cut':
                    with self.lock:
                        if self.target is not None:
                            self.target[5] = self.j6_range[0]
                            self.status = f'合刀 -> {self.j6_range[0]:+.0f}°'

                if ev == 'uncut':
                    with self.lock:
                        if self.target is not None:
                            self.target[5] = self.j6_home
                            self.status = f'回位 -> {self.j6_home:+.0f}°'

                if ev == 'resync':
                    self.resync()

                # 1) 读实测
                j = self._read_valid(timeout=READ_TIMEOUT, retries=1)
                if j:
                    with self.lock:
                        self.meas = j

                # 2) 限速逼近目标并下发
                payload = None
                with self.lock:
                    if self.enabled and self.meas and self.cmd and self.target:
                        for i in range(6):
                            rate = CUT_RATE if i == 5 else JOG_RATE
                            step = rate * SEND_PERIOD
                            d = self.target[i] - self.cmd[i]
                            if abs(d) <= step:
                                self.cmd[i] = self.target[i]
                            else:
                                self.cmd[i] += step if d > 0 else -step
                            self.cmd[i] = clamp(self.cmd[i], i)

                        if self.last_sent is None or any(
                                abs(self.cmd[i] - self.last_sent[i]) >= 0.02 for i in range(6)):
                            payload = (f'>{self.cmd[0]:.2f},{self.cmd[1]:.2f},{self.cmd[2]:.2f},'
                                       f'{self.cmd[3]:.2f},{self.cmd[4]:.2f},{self.cmd[5]:.2f},'
                                       f'{JOG_RATE:.0f}')
                if payload:
                    self.ser.write(payload)
                    with self.lock:
                        self.last_sent = list(self.cmd)
                        self.send_count += 1
                        self.last_cmd = payload

                time.sleep(SEND_PERIOD)

        except Exception:
            err = traceback.format_exc(limit=3)
            with self.lock:
                self.error = err
                self.status = '工作线程异常退出（见下方错误）'
            print(err)

        finally:
            if self.alive:
                self.alive = False
            # 退出: 若仍使能则保持当前姿态悬停
            try:
                with self.lock:
                    still_enabled = self.enabled
                    j = list(self.meas) if self.meas else None
                if still_enabled and j:
                    self.ser.write(f'>{j[0]:.2f},{j[1]:.2f},{j[2]:.2f},'
                                   f'{j[3]:.2f},{j[4]:.2f},{j[5]:.2f},{JOG_RATE:.0f}')
                    time.sleep(0.05)
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
        ttk.Button(top, text='回 HOME', command=lambda: worker.request('home')).pack(side='left', padx=10)
        ttk.Button(top, text='回 REST', command=lambda: worker.request('rest')).pack(side='left', padx=3)
        ttk.Button(top, text='目标=实测', command=lambda: worker.request('resync')).pack(side='left', padx=10)
        ttk.Button(top, text='急停 (空格)', command=self.estop).pack(side='left', padx=6)

        body = ttk.Frame(root, padding=8)
        body.grid(row=1, column=0, sticky='ew')

        self.sliders = []
        self.labels = []
        for i in range(6):
            ttk.Label(body, text=JOINT_NAMES[i], width=4).grid(row=i, column=0, sticky='w')
            lo, hi = FW_RANGES[i]
            s = ttk.Scale(body, from_=lo, to=hi, orient='horizontal', length=420)
            s.grid(row=i, column=1, padx=6, pady=2, sticky='ew')
            lab = ttk.Label(body, text='--', width=22)
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
        self.status.grid(row=3, column=0, sticky='ew', padx=10, pady=(2, 0))

        self.detail = ttk.Label(root, text='', anchor='w', foreground='#555')
        self.detail.grid(row=4, column=0, sticky='ew', padx=10, pady=(0, 6))

        self.errlabel = ttk.Label(root, text='', anchor='w', foreground='#b00')
        self.errlabel.grid(row=5, column=0, sticky='ew', padx=10, pady=(0, 8))

        body.columnconfigure(1, weight=1)
        self.pulling = False
        self.root.after(100, self.refresh)

    def on_jog(self, idx, value):
        if self.pulling:
            return
        try:
            self.worker.set_target(idx, float(value))
        except Exception:
            pass

    def estop(self):
        self.worker.request('estop')

    def refresh(self):
        try:
            (meas, target, enabled, status, msg, j6_range,
             sync, send_count, last_cmd, error) = self.worker.snapshot()
            if meas:
                self.pulling = True
                for i in range(6):
                    if sync and target:
                        lo, hi = FW_RANGES[i]
                        self.sliders[i].set(max(lo, min(hi, target[i])))
                    self.labels[i].configure(text=f'实测 {meas[i]:+8.2f}°')
                self.pulling = False
                rel = meas[5] - self.worker.j6_home
                self.j6_label.configure(
                    text=f'相对角 {rel:+7.2f}°  (绝对 {meas[5]:+8.2f}°, '
                         f'行程 [{j6_range[0]:+.0f}, {j6_range[1]:+.0f}])')
            else:
                self.j6_label.configure(text='相对角: 读数丢失')

            self.status.configure(text=f'{status}   |   {"使能" if enabled else "失能"}')
            self.detail.configure(text=f'已下发 {send_count} 条   |   最后: {last_cmd or "—"}')
            self.errlabel.configure(text=(error.strip().splitlines()[-1] if error else ''))

            if not self.worker.is_alive():
                self.status.configure(text=f'{status}   |   工作线程已停止')
                return
        except Exception as exc:      # 绝不让 UI 刷新链断掉
            try:
                self.status.configure(text=f'界面刷新异常: {exc}')
            except Exception:
                pass
        self.root.after(100, self.refresh)

    def on_close(self):
        self.worker.alive = False
        self.worker.join(timeout=2.0)
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description='Dummy 机械臂 Jog 窗口')
    ap.add_argument('--home', type=float, default=0.0, help='J6 零位（默认 0.0）')
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
    worker.join(timeout=2.0)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
