#!/usr/bin/env bash
# PickOrangeDummy 构建脚本
# 背景: 本机 colcon(ament_python) 把 console_scripts 装到 install/<pkg>/bin/,
#       而 ros2 run / launch_ros 只在 install/<pkg>/lib/<pkg>/ 下找可执行文件,
#       故构建后为每个入口脚本在 lib/<pkg>/ 下补符号链接。
set -e
cd "$(dirname "$0")/.."

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install "$@"

for pkg in install/*/; do
  pkg=${pkg%/}
  name=$(basename "$pkg")
  bin_dir="$pkg/bin"
  lib_dir="$pkg/lib/$name"
  [ -d "$bin_dir" ] || continue
  mkdir -p "$lib_dir"
  for exe in "$bin_dir"/*; do
    [ -f "$exe" ] || continue
    target="$lib_dir/$(basename "$exe")"
    if [ -e "$target" ] && [ ! -L "$target" ]; then rm -f "$target"; fi
    ln -sfn "../../bin/$(basename "$exe")" "$target"
    echo "link: $target -> bin/$(basename "$exe")"
  done
done
