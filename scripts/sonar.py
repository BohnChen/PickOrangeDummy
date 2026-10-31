#!/usr/bin/env python3
"""前向超声波 / 舵机 命令行工具（宿主机直连，不需要 docker）

依赖 scripts/router_proxy.sh 提供的中继，或已加入 arduino-router 组。

用法：
    ./scripts/sonar.py read                 # 读一次
    ./scripts/sonar.py park 0               # 舵机转到 0° 并锁定
    ./scripts/sonar.py sweep                # 0..180 每 30° 扫一遍
    ./scripts/sonar.py scan 0 180 15        # 连续扫描（固件内扫）
    ./scripts/sonar.py stop                 # 停止扫描，保持当前位置

环境变量：
    ROUTER_TARGET=tcp:127.0.0.1:7501        # 默认
    ROUTER_TARGET=unix:/var/run/arduino-router.sock
"""

import os
import socket
import sys
import time

import msgpack

TARGET = os.environ.get("ROUTER_TARGET", "tcp:127.0.0.1:7501")
REQ, RESP = 0, 1


class Rpc:
    def __init__(self, target):
        self.target = target
        self.sock = None
        self.mid = 0

    def _connect(self):
        if self.target.startswith("unix:"):
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect(self.target[len("unix:"):])
        else:
            host, _, port = self.target[len("tcp:"):].rpartition(":")
            s = socket.create_connection((host or "127.0.0.1", int(port)), timeout=2.0)
        return s

    def call(self, method, *params):
        if self.sock is None:
            self.sock = self._connect()
        self.mid += 1
        self.sock.sendall(msgpack.packb([REQ, self.mid, method, list(params)]))
        unp = msgpack.Unpacker(raw=False)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            for obj in unp:
                if isinstance(obj, (list, tuple)) and len(obj) == 4 and obj[0] == RESP:
                    if obj[2]:
                        raise RuntimeError(f"{method}: {obj[2]}")
                    return obj[3]
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("router closed connection")
            unp.feed(chunk)
        raise RuntimeError("timeout")

    def close(self):
        if self.sock:
            self.sock.close()


def parse(s):
    return dict(kv.split("=", 1) for kv in str(s).split(",") if "=" in kv)


def main(argv):
    rpc = Rpc(TARGET)
    try:
        if not argv:
            print(__doc__)
            return 2
        cmd = argv[0]

        if cmd == "read":
            d = parse(rpc.call("sonar_read"))
            flag = "ok " if d.get("valid") == "1" else "---"
            print(f"angle={d.get('angle'):>3}°  {flag}  "
                  f"{d.get('mm'):>5} mm  scan={d.get('scan')}")
            raw = parse(rpc.call("sonar_raw"))
            print(f"  raw: rise={raw.get('rise')}us width={raw.get('width')}us "
                  f"echo={raw.get('echo')}")

        elif cmd == "park":
            ang = int(argv[1])
            rpc.call("sonar_park", ang)
            time.sleep(0.5)
            print(f"parked at {ang}°")

        elif cmd == "scan":
            amin = int(argv[1]) if len(argv) > 1 else 0
            amax = int(argv[2]) if len(argv) > 2 else 180
            step = int(argv[3]) if len(argv) > 3 else 15
            rpc.call("sonar_scan_start", amin, amax, step, 300)
            print(f"scanning {amin}..{amax} step {step} (300ms/step)")

        elif cmd == "stop":
            rpc.call("sonar_scan_stop")
            print("scan stopped")

        elif cmd == "sweep":
            step = int(argv[1]) if len(argv) > 1 else 30
            print("角度 |   距离 | 有效")
            for ang in range(0, 181, step):
                rpc.call("sonar_park", ang)
                time.sleep(0.6)
                d = {}
                for _ in range(3):
                    time.sleep(0.25)
                    d = parse(rpc.call("sonar_read"))
                    if d.get("valid") == "1":
                        break
                ok = d.get("valid") == "1"
                print(f"{int(d.get('angle', ang)):>4}° | {d.get('mm'):>5} | {'ok' if ok else '--'}")
            rpc.call("sonar_park", 90)

        else:
            print(__doc__)
            return 2
    finally:
        rpc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
