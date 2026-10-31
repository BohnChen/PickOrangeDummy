#!/usr/bin/env bash
# 把 arduino-router 的 unix socket 通过 TCP 暴露给宿主机（免 sudo）。
#
# 背景：/var/run/arduino-router.sock 是 root:arduino-router 0660，
#       宿主机用户不在该组就连接不上。这个脚本用一个一次性容器做中继，
#       ROS 节点因此仍然跑在宿主机上（不需要在容器里跑 ROS）。
#
# 用法：
#   ./scripts/router_proxy.sh start     # 起中继（127.0.0.1:7501）
#   ./scripts/router_proxy.sh status    # 看是否在跑
#   ./scripts/router_proxy.sh stop      # 停掉
#
# 节点侧：
#   ros2 run pickorange_chassis chassis_node \
#     --ros-args -p rpc_target:=tcp:127.0.0.1:7501
#
# 长期方案仍是把用户加进 arduino-router 组，然后改用 unix: 直连。
set -euo pipefail

NAME="pickorange-router-proxy"
IMG="ghcr.io/arduino/app-bricks/python-apps-base:0.13.1"
SOCK="/var/run/arduino-router.sock"
PORT="${PROXY_PORT:-7501}"

RELAY='
import socket, threading, sys
UP, DOWN = "/var/run/arduino-router.sock", ("127.0.0.1", int(sys.argv[1]))
def pump(a, b):
    try:
        while True:
            d = a.recv(65536)
            if not d: break
            b.sendall(d)
    except OSError:
        pass
    finally:
        for s in (a, b):
            try: s.shutdown(socket.SHUT_RDWR)
            except OSError: pass
def handle(c):
    u = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        u.connect(UP)
    except OSError as e:
        print("relay: cannot connect socket:", e, flush=True); c.close(); return
    threading.Thread(target=pump, args=(c, u), daemon=True).start()
    pump(u, c)
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(DOWN); srv.listen(8)
print(f"relay listening on {DOWN}", flush=True)
while True:
    c, _ = srv.accept()
    threading.Thread(target=handle, args=(c,), daemon=True).start()
'

case "${1:-start}" in
  start)
    if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
      echo "already running: $NAME"; exit 0
    fi
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" --network host --restart unless-stopped \
      --user root \
      -v "$SOCK:$SOCK" \
      --entrypoint python3 "$IMG" -c "$RELAY" "$PORT" >/dev/null
    sleep 2
    docker logs "$NAME" 2>&1 | tail -3
    echo "→ 节点用 -p rpc_target:=tcp:127.0.0.1:$PORT"
    ;;
  status)
    docker ps --filter "name=$NAME" --format '{{.Names}}  {{.Status}}'
    ;;
  stop)
    docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped" || echo "not running"
    ;;
  *)
    echo "usage: $0 {start|status|stop}"; exit 2 ;;
esac
