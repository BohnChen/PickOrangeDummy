# scripts/diag/chassis — 底盘硬件诊断工具（历史 / 兜底）

> ⚠️ **这里不是主控制路径。**
> 底盘正式控制是：`firmware/ventuno_q_chassis_pwm/`（STM32H5 固件）
> + `scripts/flash_chassis.sh`（烧录）
> + Arduino RouterBridge RPC（`drive` / `set_pwm` / `stop` / `brake` / `ping`）
> + `scripts/car.py`（Linux 侧封装）。
> 见 `docs/chassis_control_verified.md`。

本目录保留的脚本，是底盘调试期（2026-10 中下旬）留下的**硬件诊断工具**。
它们直接操作 Linux GPIO（`lgpio` / `gpiochip2`），走的是**已被废弃的路线**——
直接用 Linux 发 50 Hz 软 PWM 抖动大、且会与 MCU 抢引脚。保留它们只因为在两种场合仍然有用：

1. **改线 / 换车 / 排障时**，用万用表确认某根线是强上拉还是弱上拉；
2. **固件刷坏、MCU 起不来时**，临时确认信号线本身是通的、底盘控制器还能被驱动。

**注意**：这些脚本里的引脚编号是**调试期的历史值**（GPIO 17/13/6/18/24 等），
与最终实测的 `D3/D5/D6` 映射**不一致**。用之前必须先看
`docs/chassis_control_verified.md` 的接线表核对，不要直接跑。

## 文件说明

| 文件 | 用途 | 何时用 |
|---|---|---|
| `electrical_diag.py` | 判断信号线是 5V 推挽强上拉还是弱上拉（能否被 MCU 拉低） | 改线、换底盘、怀疑电平不匹配 |
| `hold_all_low.py` | 把所有目标 GPIO 拉低并保持，供万用表逐点测量 | 通断排查 |
| `hold_high_verify.py` | 三个目标引脚同时输出稳定高电平 | 验证引脚确实到了排针 |
| `sniff_remote.py` | 监听 3/4/5 号测试点，还原遥控器发出的脉宽 | 仅当**恢复原装接收机**（需焊回 R89/R90/R91）时 |
| `direct_chassis_40pin.py` | 从 40-pin 直接发 50 Hz 三通道 PWM 驱动底盘 | MCU 固件不可用时的临时兜底 |

## 运行前提

```bash
# lgpio 需要能访问 gpiochip2
python3 -c "import lgpio; h=lgpio.gpiochip_open(2); print('ok'); lgpio.gpiochip_close(h)"
```

跑任何会驱动电机的脚本之前：**驱动轮必须离地**，并确认底盘控制器 SW2 第 8 位是打开的
（否则外部 PWM 完全无效）。
