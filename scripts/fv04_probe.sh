#!/usr/bin/env bash
# 在 FV04（AidLux）上跑一遍，收集迁移需要的事实。
# 用法（在你的 Mac 上）：
#   ssh aidlux 'bash -s' < scripts/fv04_probe.sh  > fv04_report.txt
# 或先 scp 到板子上再执行。只读，不改系统。

say() { printf '\n===== %s =====\n' "$1"; }

say "系统"
uname -a 2>/dev/null
cat /etc/os-release 2>/dev/null | head -6
echo "pid1: $(ps -p 1 -o comm= 2>/dev/null || cat /proc/1/comm 2>/dev/null)"
python3 --version 2>/dev/null
echo "CPU: $(nproc 2>/dev/null) cores"
free -h 2>/dev/null | head -3

say "ROS"
ls /opt/ros 2>/dev/null || echo "no /opt/ros"
command -v ros2 >/dev/null && ros2 --version 2>/dev/null || echo "ros2 not in PATH"

say "PWM（决定能否板载生成 3 路 50Hz）"
ls -l /sys/class/pwm/ 2>/dev/null || echo "no /sys/class/pwm"
for c in /sys/class/pwm/pwmchip*; do
  [ -e "$c" ] || continue
  echo "--- $c npwm=$(cat $c/npwm 2>/dev/null)"
done
cat /sys/kernel/debug/pwm 2>/dev/null || echo "no debugfs pwm (需要 root 且挂载 debugfs)"

say "PWM 引脚复用"
grep -i pwm /sys/kernel/debug/pinctrl/*/pinmux-pins 2>/dev/null | head -20 || echo "no pinctrl info"

say "GPIO"
ls /sys/class/gpio/ 2>/dev/null
grep -i gpio-ranges /sys/kernel/debug/pinctrl/*/gpio-ranges 2>/dev/null | head -5
cat /sys/kernel/debug/pinctrl/*/gpio-ranges 2>/dev/null | head -5

say "串口设备"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "no ttyACM/ttyUSB"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "no /dev/serial/by-id"
echo "dialout group: $(getent group dialout 2>/dev/null || grep dialout /etc/group 2>/dev/null)"

say "USB / 相机"
lsusb 2>/dev/null | head -20 || echo "no lsusb"
ls /dev/video* 2>/dev/null
command -v v4l2-ctl >/dev/null && v4l2-ctl --list-devices 2>/dev/null | head -20 || echo "no v4l2-ctl"

say "I2C / SPI"
ls /dev/i2c-* /dev/spidev* 2>/dev/null || echo "no i2c/spi devices"

say "网络"
ip -4 addr 2>/dev/null | grep -E "inet " | head -5

say "AidLux 相关"
ls /home 2>/dev/null
ls /root 2>/dev/null | head -10
mount 2>/dev/null | grep -E "overlay|ubuntu|aidlux" | head -5
