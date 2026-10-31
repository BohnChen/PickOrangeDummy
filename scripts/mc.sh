#!/usr/bin/env bash
# 在 App 容器里跑 mc.py（宿主机没有 arduino python 包，也没有 socket 权限）
#
# 用法：./scripts/mc.sh status
#       ./scripts/mc.sh steer 40 1.5
#       ./scripts/mc.sh stop
set -euo pipefail

IMG="ghcr.io/arduino/app-bricks/python-apps-base:0.13.1"
DIR="$(cd "$(dirname "$0")" && pwd)"

exec docker run --rm --user root --entrypoint python3 \
  -v /var/run/arduino-router.sock:/var/run/arduino-router.sock \
  -v "$DIR:/w:ro" -w /w \
  "$IMG" /w/mc.py "$@"
