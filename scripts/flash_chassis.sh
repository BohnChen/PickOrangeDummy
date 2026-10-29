#!/usr/bin/env bash
# 编译并烧录底盘固件（VENTUNO Q / STM32H5F5）
#
# 背景：这块板 OpenOCD 用 linuxgpiod 位操作 SWD，target 脚本复位时执行
#       `adapter speed 480` 会失败，MCU 被留在 ROM/DFU 里，程序不会跑。
#       这里自己写 flash，再用 AIRCR.SYSRESETREQ 复位。
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="$REPO/firmware/ventuno_q_chassis_pwm"
BUILD="/tmp/chassis_build"
FQBN="arduino:zephyr:ventunoq:wait_linux_boot=no"
OCD="/opt/openocd/bin/openocd"
OCD_CFG="/opt/openocd/openocd_gpiod.cfg"
ADDR="0x08100000"

echo "==> 编译"
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD" "$SKETCH" | tail -3

BIN="$(ls "$BUILD"/*.elf-zsk.bin | head -1)"
echo "==> 写入 $BIN -> $ADDR"
"$OCD" -f "$OCD_CFG" \
  -c "stm32h5x.cpu configure -event reset-start {}" \
  -c "init" -c "halt" \
  -c "flash write_image erase $BIN $ADDR bin" \
  -c "verify_image $BIN $ADDR bin" \
  -c "mww 0xE000ED0C 0x05FA0004" \
  -c "shutdown" 2>&1 | grep -E "wrote|verified|Error" || true

sleep 2
if lsusb -d 0483:df11 >/dev/null 2>&1; then
  echo "!! 仍停在 DFU：复位没生效"
  exit 1
fi
echo "✓ 固件已写入并启动"
