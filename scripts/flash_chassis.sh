#!/usr/bin/env bash
# 编译并烧录底盘固件（VENTUNO Q / STM32H5F5）
#
# 背景：这块板 OpenOCD 用 linuxgpiod 位操作 SWD，target 脚本复位时执行
#       `adapter speed 480` 会失败，MCU 被留在 ROM/DFU 里，程序不会跑。
#       这里自己写 flash，再用 AIRCR.SYSRESETREQ 复位。
#
# 2026-10-31 修正（重要）：
#   1) 写 flash 前必须 `reset halt`。只 `halt` 时，STM32 flash 算法可能超时：
#        Error: timeout waiting for algorithm, a target reset is recommended
#        Error: block write failed
#      此时镜像只写了一部分，MCU 起来后 loader 加载 sketch 失败，
#      STM 区 LED1 红通道会「闪 3 下、停约 1 秒」循环 —— 见
#      docs/chassis_loader_red_led_incident.md
#   2) 必须检查 openocd 的 wrote / verified 输出。旧版本用 `|| true`，
#      写失败也会打印「✓ 已写入并启动」，等于把故障静默掉。
#   3) 成功后确认 MCU 的 USB 已重新枚举（1209:ca01），这是「sketch 真跑起来」的判据。
#
# 用法：
#   ./scripts/flash_chassis.sh                    # 底盘控制固件（chassis_v2_atomic）
#   ./scripts/flash_chassis.sh chassis_neutral    # 中位待机固件（安全态）
#   ./scripts/flash_chassis.sh <sketch目录名>      # 指定 firmware/ 下的任意 sketch
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH_NAME="${1:-chassis_v2_atomic}"
SKETCH="$REPO/firmware/$SKETCH_NAME"
BUILD="/tmp/chassis_build_$SKETCH_NAME"
FQBN="arduino:zephyr:ventunoq:wait_linux_boot=no"
OCD="/opt/openocd/bin/openocd"
OCD_CFG="/opt/openocd/openocd_gpiod.cfg"
ADDR="0x08100000"
LOG="/tmp/flash_$SKETCH_NAME.log"

[ -d "$SKETCH" ] || { echo "!! 找不到 sketch 目录：$SKETCH"; exit 1; }

echo "==> 编译 $SKETCH_NAME"
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD" "$SKETCH" | tail -3

BIN="$(ls "$BUILD"/*.elf-zsk.bin | head -1)"
SIZE="$(stat -c%s "$BIN")"
echo "==> 写入 $BIN ($SIZE 字节) -> $ADDR"

"$OCD" -f "$OCD_CFG" \
  -c "stm32h5x.cpu configure -event reset-start {}" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase $BIN $ADDR bin" \
  -c "verify_image $BIN $ADDR bin" \
  -c "mww 0xE000ED0C 0x05FA0004" \
  -c "shutdown" >"$LOG" 2>&1 || true

# --- 严格判定：必须真的写了、并且校验通过 ---
if grep -qE "block write failed|error writing to flash|timeout waiting for algorithm" "$LOG"; then
  echo "!! flash 写入失败（算法超时/块写失败），镜像是残的。"
  grep -E "Error|failed" "$LOG" | tail -5
  exit 1
fi
if ! grep -q "^wrote " "$LOG"; then
  echo "!! 没有写入记录，openocd 输出见 $LOG"
  tail -20 "$LOG"
  exit 1
fi
if ! grep -q "^verified " "$LOG"; then
  echo "!! 校验未通过，openocd 输出见 $LOG"
  tail -20 "$LOG"
  exit 1
fi
grep -E "^wrote |^verified " "$LOG" | sed 's/^/    /'

sleep 2
if lsusb -d 0483:df11 >/dev/null 2>&1; then
  echo "!! 仍停在 DFU：复位没生效"
  exit 1
fi

# --- 成功判据：MCU 的 USB CDC 重新枚举 ---
for _ in $(seq 1 10); do
  if lsusb -d 1209:ca01 >/dev/null 2>&1; then
    echo "✓ 固件已写入、校验通过、MCU 已启动（1209:ca01 在线）"
    exit 0
  fi
  sleep 0.5
done

echo "!! 写入校验都过了，但 MCU 的 USB 没枚举：sketch 可能没加载起来"
echo "   看 STM 区 LED1：闪 3 下 + 停 1 s = loader 加载失败"
exit 1
