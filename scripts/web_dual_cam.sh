#!/usr/bin/env bash
# 启动双相机预览服务（http://<板子IP>:8888）
set -euo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws_orbbec/install/setup.bash" ] && source "$HOME/ros2_ws_orbbec/install/setup.bash"
pkill -f web_dual_cam.py 2>/dev/null || true
sleep 1
exec python3 scripts/web_dual_cam.py
