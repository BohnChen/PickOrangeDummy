# 底盘烧录事故与 STM 红灯含义（2026-10-31）

> 现象：VENTUNO Q 的 STM 区 LED1（丝印 `1`）红通道**闪 3 下、停约 1 秒、再闪 3 下**，无限循环。
> 结论：**跟「没接信号线」没有任何关系**。是烧录时 flash 没写完整，MCU 里的 loader
> 加载不了 sketch，用红灯报错。重写镜像即恢复。
> 本文记录现象、证据链、根因与以后必须遵守的烧录步骤。

---

## 1. 现象

| 项 | 内容 |
|---|---|
| 时间 | 2026-10-31 |
| 板子 | Arduino VENTUNO Q（STM32H5F5 + Qualcomm QRB2210 双脑） |
| 现象 | STM 区 LED1 红通道 3 短闪 + ~1.2 s 停顿，循环 |
| 伴随 | MCU 的 USB CDC（`1209:ca01`）**完全不枚举** → Linux 侧没有 `/dev/ttySTM0` → RouterBridge 连不上 |
| 误导性前提 | 当时板子上没有接任何信号线，很容易误判成「有固件在跑但没有信号输入」 |

**为什么「没信号」这个解释不成立**：

- 这套系统里 D3/D5/D6 是**输出**，VENTUNO Q 是发 PWM 的一方，固件里没有任何输入检测；
- 固件根本不知道线插没插，也就不可能因为「没信号」去点灯；
- 现场反证：把中位固件写进去之后，同样没接线，灯立刻正常。变量只有镜像。

---

## 2. 这颗红灯到底是谁点亮的

不是 sketch，是 MCU 里预装的 **loader**（`ArduinoCore-zephyr/loader/error_led.c`）。

板级设备树把 LED1 的红通道定义成错误指示灯：

```dts
/* variants/arduino_ventuno_q_stm32h5f5xx/arduino_ventuno_q_stm32h5f5xx.overlay */
aliases {
    error-led = &red_led_0;      /* red_led_0 = LED1 的 R = PG3 */
};
```

对应 `variant.h` 里的 `LED1_R / LED1_G / LED1_B`，即 STM 区丝印 `1` 的 RGB 灯。
所以：**STM 区 LED1 红 = 错误指示**，与用户程序无关。

---

## 3. 红灯节奏对照表（从 loader 固件反汇编确认）

| 节奏 | 触发函数 | 含义 |
|---|---|---|
| **3 短闪 + 停 ~1.2 s，循环** | `recoverable_error_led()` | **sketch 没能加载/启动**（镜像问题） |
| 4 × 150 ms 快闪 + 4 × 400 ms 慢闪，循环 | `k_sys_fatal_error_handler()` | sketch **跑起来之后崩了**，会打印 `Sketch crashed, fatal error reason %u` |
| LED1~LED4 一起跑动画 | loader 的 `matrixPlay()` 启动动画 | 在等 Linux / 等 App（Startup mode 不是 Immediate） |

`recoverable_error_led()` 只在 `loader()` 的五条失败路径上被调用，每条都会先打印一行：

```c
flash_area_open(3, &fa)        -> "Failed to open flash area, rc %d"
flash_area_read(fa, hdr, 16)   -> "Failed to read header, rc %d"
sketch_header_v1_verify(hdr)   -> "Invalid sketch header"
llext_load(...)                -> "Failed to load sketch, rc %d"
llext_find_sym(ext, "main")    -> "Failed to find main function"
```

> 这行日志走 `zephyr,console = &usart3`，USART3 = PB10/PB11 = 排针 **D1/D0，115200**。
> 想直接看原因，用 USB-TTL 的 RX 接 D1、GND 接 GND，按复位即可抓。

---

## 4. 根因

**烧录时 flash 没有写完整，脚本还把失败静默掉了。**

复现到的原始错误（写 `0x08100000`）：

```
Error: timeout waiting for algorithm, a target reset is recommended
Error: error executing stm32l4 flash write algorithm
Error: block write failed
Error: error writing to flash at address 0x08000000 at offset 0x00100000
```

关键点：

1. OpenOCD 在 linuxgpiod（GPIO 位操作 SWD）下，**只 `halt` 就写 flash 会超时**；
   先 `reset halt` 再写，同样的镜像一次成功。
2. 旧的 `scripts/flash_chassis.sh` 用
   `... | grep -E "wrote|verified|Error" || true`
   收集输出 —— 写失败时也能打印「✓ 固件已写入并启动」，
   只检查了 `0483:df11`（DFU）这一种失败，**等于把半截镜像的故障静默掉**。
3. 结果是 sketch 槽里留了一个**头合法、但加载失败**的镜像：
   实读 `0x08100000` 前 16 字节，`version=1`、magic `0x2341`、长度 `0x184FC=99580`
   与文件大小一致，ELF 21 个 section 也完整，`main` 符号存在
   —— 也就是说 **header 校验过了，坏在 `llext_load` 阶段**。

> 教训：镜像头合法 ≠ 能加载。判断「到底跑没跑」不能看烧录脚本的 ✓，
> 要看 MCU 的 USB 有没有重新枚举（`1209:ca01`）。

---

## 5. 定位过程（可复现）

```bash
# 1) 看 MCU 有没有在跑：USB CDC 是否枚举
lsusb | grep 1209        # 当时：无

# 2) 读 sketch 槽的头（只读，不改）
/opt/openocd/bin/openocd -f /opt/openocd/openocd_gpiod.cfg \
  -c "init" -c "halt" -c "mdw 0x08100000 6" -c "shutdown"
# 0x08100000: 464c457f 01010101 000184fc 00042341 ...  -> 头合法

# 3) 整块 dump 下来用 readelf 验证
#    - e_shoff + 21*40 == 文件大小 → 文件完整
#    - 有 main 符号              → 不是 "Failed to find main function"
#    → 结论：坏在 llext_load

# 4) 二分：把最小固件写进去
/opt/openocd/bin/openocd -f /opt/openocd/openocd_gpiod.cfg \
  -c "stm32h5x.cpu configure -event reset-start {}" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase <bin> 0x08100000 bin" \
  -c "verify_image <bin> 0x08100000 bin" \
  -c "mww 0xE000ED0C 0x05FA0004" -c "shutdown"
# 结果：MCU USB 立刻枚举出 1209:ca01，/dev/ttySTM0 -> ttyACM0 出现
#       arduino-router 日志：Opened serial connection serial=/dev/ttySTM0
#       → loader / flash / 烧录链路都正常，问题锁定在那份镜像
```

---

## 6. 处置与验证

| 步骤 | 结果 |
|---|---|
| `reset halt` 后重新写入 `rc_min`（最小固件） | 写入校验通过，MCU USB 枚举，红灯熄灭 |
| 写入 `chassis_neutral`（中位待机固件） | 同上，三路 1500 µs，车停住 |
| `arduino-router` | 自动连上 `/dev/ttySTM0` |

**成功判据（以后都用这个）：**

1. openocd 输出里有 `wrote ...` 和 `verified ...`；
2. 没有 `block write failed` / `timeout waiting for algorithm`；
3. `lsusb | grep 1209:ca01` 有输出；
4. 红灯不再 3 短闪。

---

## 7. 以后必须遵守（硬性）

1. **烧录脚本已修正**：`scripts/flash_chassis.sh` 现在会
   `reset halt` → 写 → 校验 → `SYSRESETREQ` → 等 `1209:ca01` 枚举；
   写失败/校验失败/枚举失败都会 `exit 1`，不再打印假 ✓。
2. **不要用 `arduino-cli upload` / App Lab 的 restart** 烧 MCU（复位那步会失败，把人留在 DFU）。
3. **不要在没确认写入结果的情况下接线**：半截镜像的表现就是「车不动 + 红灯 3 闪」。
4. **`rc_min` 绝对不能当待机固件**：它在 D3 上恒定输出 **2000 µs = 满油门**。
   待机一律用 `firmware/chassis_neutral`（三路 1500 µs）。
5. 板上跑什么固件，接线前必须知道：接上电源和信号线，车就会按当前固件动作。

---

## 8. 本次附带确认的事实

1. **当前 core 是 `arduino:zephyr 1.1.0`**（`arduino-cli core list`），
   loader 版本 `v4.4.2-188-ga12e8f37553a`，与包内 `firmwares/zephyr-*.bin` 一致；
   本文里的 rc_min、chassis_neutral、以及重新编译的底盘固件都在 1.1.0 下正常加载。
   （第 3 节「必须是 1.0.0」是 2026-10-29 的记录，已被 1.1.0 实测覆盖。）
2. **只接 3 路（3/4/5 → D3/D5/D6 + GND）也能跑**：
   `rc_min` 只给 D3=2000 µs、D5/D6 无信号时，车仍然满油门直行。
   说明 1/2/6 缺失**不会**阻止运动 —— 所以「缺通道会进失效保护」不能当作安全依赖。
3. **缺档位（D6）时默认是前进**。倒车必须显式给 D6=1000 µs，不能靠「没信号」。
4. **无信号 ≠ 停**。要停就给 1500 µs 中位并保持，别用断电/停 MCU 来「停车」。

---

## 9. 相关文件

| 文件 | 说明 |
|---|---|
| `docs/chassis_control_verified.md` | 底盘接线 / 信号 / 烧录主规范 |
| `docs/chassis_wiring_and_control_spec.md` | 旧版接线规范（部分已被取代） |
| `firmware/ventuno_q_chassis_pwm/` | **旧版**底盘固件（`drive/set_pwm/stop/brake/ping`），已被取代，仅作历史参考 |
| `firmware/chassis_v2_atomic/` | **当前主固件**：原子动作 + 300 ms 租约 + 固件锁存急停 |
| `firmware/chassis_neutral/` | 中位待机固件（安全态，2026-10-31 新增） |
| `scripts/estop.py` | 锁存急停的命令行入口（需 socket 权限） |
| `scripts/mc.py` / `mc.sh` | 底盘单步测试工具（在 App 容器里跑，免 sudo） |
| `scripts/flash_chassis.sh` | 唯一推荐的烧录入口（2026-10-31 加固） |
