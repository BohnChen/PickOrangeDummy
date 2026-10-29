# 底盘控制实测规范（v2，已验证）

> 本文取代 `chassis_wiring_and_control_spec.md` 中关于引脚映射与信号行为的部分。
> 所有结论均在实车（VENTUNO Q + 该底盘）上实测确认，日期 2026-10-29。

---

## 1. 结论速览

| 项目 | 结论 |
|---|---|
| 信号 | 50 Hz，1000–2000 µs 正脉冲，3.3 V 高有效 |
| 3 号 | 油门大小：1500 停，>1500 前进，<1500 刹车/减速 |
| 4 号 | 转向：1500 居中，±400 µs 满舵 |
| 5 号 | 档位/方向：1500 与 1000 两个方向，**必须配合油门** |
| 1/2/6 号 | 未使用，但**必须保持 1500**（缺通道会进失效保护） |
| 板子前提 | **SW2 拨码第 8 位必须打开**，否则外部 PWM 完全无效 |
| 接收机 | 串阻 R89/R90/R91 已拆除，遥控失效 |

---

## 2. 接线表

| 小车「接收机测试点」 | VENTUNO Q | 说明 |
|---|---|---|
| `1` | `D9` | 保持中位 |
| `2` | `D10` | 保持中位 |
| `3` | `D3` | 油门 |
| `4` | `D5` | 转向 |
| `5` | `D6` | 档位/方向 |
| `6` | `D11` | 保持中位 |
| `-` | `GND` | **唯一一根地线** |
| `+` | **不接** | 板子给接收机供电的 5 V，接上会反灌 |

> 注意：旧文档里 5 号接 `D9` 是错的，实测应为 **D6**；`D9/D10/D11` 用于 1/2/6 号中位。

---

## 3. 平台侧坑（与本项目强相关）

1. **Arduino core 必须是 1.0.0**
   `arduino:zephyr 0.90.0` 里 VENTUNO Q 的板定义是照 UNO Q（STM32U585）抄的，
   烧进去的 loader 会在这颗 STM32H5F5 上启动即 bus fault（`K_ERR_ARM_BUS_PRECISE_DATA_BUS`），
   MCU 停在 `arch_system_halt`，程序永远不跑。

2. **烧录后必须软件复位**
   这块板 OpenOCD 用 linuxgpiod 位操作 SWD，`stm32h5x.cfg` 复位时执行
   `adapter speed 480` 不被支持 → `reset` 失败 → MCU 被留在 DFU（USB `0483:df11`）。
   解决：烧完后用 `AIRCR.SYSRESETREQ`（`mww 0xE000ED0C 0x05FA0004`）复位。
   已封装为 `scripts/flash_chassis.sh`。

3. **不要与 App Lab 的部署流程混用**
   `arduino-app-cli app restart` 会重新编译烧录，同样卡在复位那一步；
   调试期统一用 `flash_chassis.sh`。

---

## 4. Linux 控制接口

MCU 通过 RouterBridge 暴露 RPC：

| 方法 | 参数 | 返回 | 说明 |
|---|---|---|---|
| `drive` | `speed, steer`（-100..100） | `true` | 常规驾驶；`speed<0` 自动切倒档 |
| `set_pwm` | `thr_us, steer_us, gear_us` | `true` | 直接写脉宽，诊断用 |
| `stop` | — | `true` | 立即回中位 |
| `brake` | — | `true` | 轻刹（油门 1400） |
| `ping` | — | `"PONG"` | 心跳 |

### 4.1 安全语义

- **500 ms 看门狗**：超时无新指令 → 自动回中位（Linux 崩溃也不会失控）
- **斜率限制**：每帧最多变 1%（约 50%/秒），起步/停车平滑
- **死区**：|指令| < 3% 视为 0
- **上电默认**：六路全部 1500

### 4.2 Python 用法

```python
import sys
sys.path.insert(0, "scripts")
from car import Car

car = Car()            # 后台 20 Hz 持续下发
car.drive(30, 0)       # 油门 30% 直行
car.drive(30, 20)      # 边走边转（符号方向待标定）
car.drive(-25, 0)      # 倒车
car.stop()
car.close()
```

直接调 Bridge 也可以（必须持续调用，否则看门狗停车）：

```python
from arduino.app_utils import Bridge
Bridge.call("drive", 30, 0)
```

### 4.3 运行位置（权限）

`/var/run/arduino-router.sock` 属于 `root:arduino-router`。因此：

- **推荐**：把控制代码放在 App Lab 的 app 容器里运行（容器内可直接访问）
- 或者把用户加入该组（需重新登录）：

```bash
sudo usermod -aG arduino-router $USER
```

---

## 5. 烧录

```bash
./scripts/flash_chassis.sh
```

脚本做的事：编译 → 用 OpenOCD 把 sketch 写入 `0x08100000` → 校验 → `SYSRESETREQ` 复位。

---

## 6. 排障清单（按顺序查）

1. **SW2 第 8 位**是否打开？（90% 的“完全没反应”都是这里）
2. 小车是否上电？蜂鸣器有节奏地响 = 板子活着且在收信号
3. 六路信号线是否都插好？`GND` 是否只有一根、接触良好？
4. `flash.sh` 是否报“已写入并启动”？MCU 是否还在 DFU（`lsusb | grep 0483`）？
5. 是否用了 `arduino:zephyr 1.0.0`（`arduino-cli core list`）？
6. R89 是否短接（3 号通路）？

---

## 7. 待标定项

| 项 | 当前值 | 待定 |
|---|---|---|
| 转向符号 | `steer>0` 对应一侧（待确认） | 视觉程序里的左右约定 |
| 倒车档 | `GEAR_FWD=1500` / `GEAR_REV=1000` | 若方向相反则对调 |
| 实用速度 | 30% 可稳定起步 | 各场景的速度上限 |
| 转向幅度 | ±100% = ±400 µs | 机械限位保护值 |
