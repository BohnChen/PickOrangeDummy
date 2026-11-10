# 交接文档：PickOrangeDummy（VENTUNO Q）→ PickOrangesFV04

> 写于 2026-11-10。读者：接手新仓库的新会话/新同事。
> **旧仓库只读参考，不要在其中改代码。** 新仓库：
> `/Users/bohn/04_Project/02_My_Dummy/PickOrangesFV04`
> 远端：`git@github.com:BohnChen/PickOrangesFV04.git`

---

## 0. 一句话任务

把旧仓库里**仍然成立**的资产（机械臂链路、底盘实测、双相机几何、安全机制、供电教训）
搬到新仓库，在 **FV04 + AidLux + ROS 2 Humble** 上重建整机，
并把**交互模块**（移动车子的手动/状态界面）作为最终形态的一部分。

---

## 1. 三个环境

| 环境 | 路径 | 用途 |
|---|---|---|
| 旧仓库（参考） | `/Users/bohn/04_Project/02_My_Dummy/PickOrangeDummy` | 资产来源、事故史、实测数字；**只读** |
| 新仓库（目标） | `/Users/bohn/04_Project/02_My_Dummy/PickOrangesFV04` | 重建，干净历史 |
| FV04 设备 | 局域网可连（AidLux 融合系统） | 目标运行平台 |

---

## 2. 已定决策（用户已拍板，不要重新讨论）

| # | 决策 |
|---|---|
| 1 | 平台：**广翼智联 FV04**（QCS8550），系统 **AidLux 融合系统**（Android + Ubuntu 22.04） |
| 2 | ROS 2 目标：**Humble**（Ubuntu 22.04） |
| 3 | 新仓库 **干净历史**（旧仓库归档保留，不迁 git history） |
| 4 | 包名保留：`pickorange_*` / `dummyarm_*` |
| 5 | 打印文件（`models/*.3mf/.gcode`）**不进仓库**，用户之后放 MakerWorld |
| 6 | 最终形态一次排进去：**交互模块必须有**（方便移动车子） |
| 7 | 底盘 PWM 方案：**用户希望直接在板子上做**（排针 → 底盘控制器），但需先实测，见 §8.1 |

---

## 3. FV04 平台事实（已查证）

| 项 | 参数 |
|---|---|
| SoC | Qualcomm QCS8550，8 核（1×Kryo Gold 3.2 GHz + 4×2.8 + 3×Silver 2.0），Adreno 740，**48 TOPS** |
| 内存/存储 | 16 GB LPDDR5x + 128 GB UFS（也有 8 GB 版本） |
| 系统 | AidLux 融合系统：Android + Ubuntu 22.04 共享内核；浏览器 `ip:8000` 进桌面，也可 VNC/SSH。**实测 PID1 = systemd**，AidLux 各服务以 systemd 单元运行 |
| ROS | 应用中心可装 **ROS 2 Humble + Gazebo + MoveIt2**（官方目前只在 Ubuntu 22.04 支持） |
| 算力工具 | **AidLite-SDK**（模型转换/量化/推理），不是 ONNX Runtime/TFLite |
| 接口 | 4× USB3.1 + 1× USB-C、HDMI IN/OUT、4 路网口、Micro SD、Wi-Fi 7 / BT 5.3 |
| 排针 | **两个 40-pin**：① Raspberry 40PIN（GPIO/UART，**3.3 V**）；② Qualcomm 40PIN（UART/CAN/I2S，**1.8 V**） |
| 供电 | DC 12–24 V，135×115×55 mm，−20~60 °C |
| GPIO | `/sys/class/gpio` 可用（`gpio-ranges` 查映射） |

> 关键：**没有板载 MCU、没有 `arduino-router`**。VENTUNO Q 时代的底盘 PWM 生成方式在这里不存在。

参考：
[赛事设备介绍](https://forum.aidlux.com/t/topic/72112) ·
[FV04 GPIO 教程](https://forum.aidlux.com/t/topic/72716) ·
[机器人软件安装指南](https://docs.aidlux.com/guide/hardware/products-introduction/Robot-Kit-software)

---

## 4. 必须继承的资产（真机/事故验证过，重做是浪费）

来自旧仓库：

| 资产 | 旧仓库位置 | 说明 |
|---|---|---|
| 机械臂真值表、限位、关节映射 | `docs/HANDOFF_交接文档.md`、`docs/M0_spec.md` | J3 < 35° 红线、J5 ≤ 95°、J6 多圈 |
| 运动学与采摘策略 | `scripts/arm_kin.py`、`arm_pick.py`、`arm_safety.py` | IK、进刀、扫掠碰撞 |
| 手眼标定链路 | `scripts/calib_eye_to_hand.py` | ArUco + 6 点 SVD，留出 7.6 mm |
| J6 刀片零点/剪切 | `scripts/zero_j6.py`、`j6_homing.py`、`cut.py`、`docs/M2_spec_j6_blade_home.md` | 三道闸 |
| 底盘信号语义与非线性 | `docs/chassis_control_verified.md`、`docs/architecture_v2.md` §3.3 | 油门/转向/档位、档位是"窗口"、无编码器 |
| 双相机几何结论 | `docs/architecture_v2.md` §3.5 | Astra 盲区 <0.6 m、D435i 左 100°、首次可见 +0.44 m |
| 安全三件套 | `docs/safety_and_incidents.md` | J6 三道闸 / 扫掠检查 / 锁轴不失能 |
| 供电事故 D 与 v2 方案 | `docs/POWER_v2_SPEC.md` | 12 V Buck + 独立支路 + 禁止热插拔 |
| 接口契约 | `src/pickorange_interfaces` | 5 msg / 3 action / 1 srv |
| 整车 TF 与几何 | `src/pickorange_bringup`、`dummyarm_description/urdf` | base_footprint 纪律 |
| 末端几何 | `docs/ARCHITECTURE_REVIEW_2026-11-10.md` | 罩口 125 mm、外廓 172 mm、出料口 −Y |
| 图 | `docs/diagrams/12-system-overview.svg`、`13-power-failure.svg` | 手写 SVG，可直接改 |

---

## 5. 必须改的（迁移的技术核心）

| 项 | 旧 | 新 |
|---|---|---|
| ROS 2 | Jazzy（24.04） | **Humble（22.04）** |
| 底盘传输 | `arduino-router` msgpack RPC（unix/tcp） | **USB 串口**（外置 MCU）或**板载 PWM**（见 §8.1） |
| 底盘 PWM 生成 | 板载 STM32H5 固件 `chassis_v3_sonar.ino` | 待定：硬件 PWM / PCA9685 / 外置 MCU |
| `ros2_control` | 4.x（Jazzy） | **2.x（Humble）**，`dummyarm_hardware` 要改 |
| MoveIt 2 | 2.9/2.10 代 | **2.5/2.6 代**，launch/yaml 要调 |
| Python | 3.12 | **3.10**（注意语法与 numpy 版本） |
| NPU | 未用 | **AidLite-SDK**（YOLO 要重新量化导出） |
| 交互 | `web_dual_cam.py` 预览 | **正式交互模块**（Web 或 Android App，AidLux 允许 Android 写 UI） |
| 开机自启 | systemd | **AidLux 有 systemd**（实测），写 `*.service` 即可 |

---

## 6. 迁移清单（逐目录）

| 旧路径 | 体积 | 处理 | 说明 |
|---|---|---|---|
| `src/pickorange_interfaces` | 48 K | **搬** | 无需改动 |
| `src/pickorange_bringup` | 20 K | **搬** | TF + transforms.yaml，Humble launch 微调 |
| `src/pickorange_chassis` | 40 K | **改** | 传输层重写（串口/板载），去掉 RPC |
| `src/pickorange_arm` | 28 K | **搬** | Action server，改 Humble |
| `src/dummyarm_hardware` | 32 K | **改** | ros2_control 4.x → 2.x |
| `src/dummyarm_moveit_config` | 56 K | **改** | MoveIt 2.5 配置 |
| `src/dummyarm_description` | 154 M | **部分搬** | **只搬 URDF/xacro + `meshes/*.stl`（约 10 M）**；`meshes/export/*.dae`（143 M 高模）不要 |
| `src/dummyarm_cad_description` | 148 M | **不搬** | 纯视觉高模，规划不需要 |
| `src/dummyarm_perception` | 68 K | **选搬** | HSV/预览可作参考，主链路重写 |
| `src/dummyarm_bridge` / `dummyarm_bringup` | 36 K | **不搬** | 历史遗留 |
| `firmware/chassis_v3_sonar` | 24 K | **暂留参考** | 若最终改用外置 MCU 再启用 |
| `scripts/` 62 个 | 620 K | **选搬** | 见下 |
| `config/` | 8 K | **搬** | `j6_blade.json`、`pick_offset.json` |
| `docs/` | 4.5 M | **选搬** | 关键文档见 §4 |
| `DH_Params/` | 496 K | **搬** | DH 参数图 |
| `tests/` | 8 K | **搬** | kin 测试 |
| `models/` | 42 M | **不搬** | 打印文件，用户放 MakerWorld |
| `dummyarm_AImodel/` | 1.2 G | **只搬代码** | `scripts/`、`config/`、`README.md`；不含 datasets/runs |
| `carPhotos/` | 10 M | **不搬** | 进旧仓库归档即可 |
| `presentation_assets/` | 4.4 M | **不搬** | 旧答辩材料，留在旧仓库 |

**scripts 选搬建议**：
- 必搬：`arm_kin` `arm_pick` `arm_safety` `calib_eye_to_hand` `zero_j6` `j6_homing`
  `disable` `estop` `arm_goto_xyz` `safe_park_and_exit.sh` `cut` `car` `test_chassis_control`
  `web_dual_cam` `sonar` `pick_and_cut_pipeline`（作参考）
- 可丢：`step_*` `parse_step_assembly` `align_*` `assemble_stack` `extract_step_joints`
  （CAD/STEP 一次性流水线）、`*_touch_eval`（旧实验）

---

## 7. 新仓库结构建议

```
PickOrangesFV04/
├── README.md                 # 项目简介 + 快速开始 + 目录说明
├── docs/
│   ├── architecture_fv04.md  # 新架构（L0–L4 在 FV04 上如何落）
│   ├── fv04_platform.md      # AidLux 实测记录（系统/权限/串口/PWM/GPIO）
│   ├── safety_and_incidents.md   # 继承 + 事故 D
│   ├── hardware/
│   │   ├── arm_truth_table.md
│   │   ├── chassis_signals.md
│   │   └── power_v2.md
│   └── migration_note.md     # 本文的浓缩版：从哪搬的、改了什么
├── hardware/                 # 接线图、BOM、电源树
├── firmware/                 # 仅当保留外置 MCU
├── src/                      # ROS 2 Humble 包
│   ├── pickorange_interfaces/
│   ├── pickorange_bringup/
│   ├── pickorange_chassis/
│   ├── pickorange_arm/
│   ├── pickorange_ui/        # 新：交互模块
│   ├── dummyarm_description/
│   ├── dummyarm_hardware/
│   └── dummyarm_moveit_config/
├── config/
├── scripts/
└── tools/
```

`.gitignore` 至少要盖住：`build/ install/ log/ __pycache__/ *.pyc .DS_Store _tmp_* .venv/`
以及 `*.pt *.onnx *.dae`（除非改用 LFS）。

---

## 8. 关键技术问题

### 8.1 底盘 PWM：能不能全在板子上做（**已实测，结论：不能纯硬件 PWM**）

在 FV04 上实测（2026-11-10，见 §12），结论：

| 路线 | 实测结果 | 可用性 |
|---|---|---|
| 板载硬件 PWM（`/sys/class/pwm`） | 只有 **3 个 PMIC（PM8550）PWM chip**：RGB LED、屏幕背光、风扇；**没有一个引到 40pin 排针**；SoC PWM 未在设备树使能 | ❌ 不可用 |
| I2C + PCA9685 | `CONFIG_I2C_CHARDEV is not set`、无 `/dev/i2c-*`、无 i2c-dev 模块 | ❌ 不可用（要改内核，不现实） |
| SPI 舵机控制 | 无 spidev / spi_master | ❌ 不可用 |
| **排针 GPIO 软件 PWM** | GPIO 301–511 可用（排针 pin N → GPIO 336+N） | ⚠️ 唯一"纯排针"路线，抖动待实测 |
| **USB → 3 路 PWM 小板** | `CONFIG_USB_ACM=y`、ch341/cp210x/ftdi 内置，USB 3.0 四路 | ✅ 最稳的"板子出信号"方案 |
| 外置 MCU（旧 Arduino） | 同上，USB 串口 | ✅ 安全性最好（可带租约看门狗） |

**安全缺口（与方案无关）**：Linux 侧生成 PWM 时，进程崩溃/系统卡死 **PWM 会保持最后值**，
而旧项目已实测「档位信号浮空可能被判为前进」。
所以无论选哪条，**物理急停是必需项**，并必须确认底盘控制器在信号丢失时的行为：

> 车架空、低速前进中，直接拔掉油门信号线 → 停 / 滑行 / 继续跑？

- 会停 → GPIO/USB 方案可接受；
- 继续跑 → 必须加"信号看门狗"（小 MCU 只做钳位），或改用外置 MCU 生成。

**GPIO 软件 PWM 的实测方法**（需要先拿到 FV04 排针引脚定义）：
1. 从阿加犀 A8550PF1 手册查 RPi 40PIN 的 3.3 V GPIO 引脚；
2. 选两个空闲脚做**回环**（输出 → 输入），用 `gpiomon` 打时间戳测 50 Hz 抖动；
3. 抖动 < 100 µs 再考虑接底盘，否则走 USB 方案。

### 8.2 ROS 2 Humble 具体牺牲了什么

| 维度 | Humble | Jazzy | 对我们的影响 |
|---|---|---|---|
| 支持期 | 2027-05 EOL | 2029-05 EOL | 少 2 年；2026 年内无影响 |
| Python | 3.10 | 3.12 | 不能用 3.11+ 语法（`tomllib`/`ExceptionGroup`/`Self`）；numpy 1.21 |
| ros2_control | 2.x | 4.x | `dummyarm_hardware` 要改 API（中等工作量） |
| MoveIt 2 | 2.5/2.6 | 2.9/2.10 | launch/yaml 要调；planner 行为差异小 |
| DDS | Fast DDS 2.6 | 2.14 | 无感 |
| 新特性 | — | 类型适配、Zenoh RMW、服务内省、bag 压缩改进 | **我们一个都不用** |

**结论：我们失去的是"生态新鲜度"和 2 年支持期，不是项目实际用到的能力。**
Actions / TF / URDF / MoveIt / ros2_control / rclpy 的核心 API 在两者间是稳定的。
唯一要认真验证的是：**Astra Pro Plus 的 OrbbecSDK v1.x 驱动在 Humble 上是否可用**
（旧项目在 Jazzy 上验证过；Humble 需重新确认）。

### 8.3 NPU 与模型链路

- 旧项目：YOLO11-seg 的 ONNX，权重不在板上，回退 HSV。
- 新平台：**AidLite-SDK**（模型转换 + 量化 + 推理）。
- 迁移路径：`best.pt → ONNX → AidLite 量化/编译 → Python/C++ 调用`。
- 先跑通"一张图推理"再谈实时；不要把 NPU 做成第一周的阻塞项。

### 8.4 AidLux 环境注意

- **有 systemd**（实测 PID1=systemd，`aid-*` 服务在跑）：节点可以做成 systemd 单元，崩溃自愈用 `Restart=`。
- **权限**：容器内通常 root，但 `/dev/ttyACM*` 的设备号会漂移 → 用 `/dev/serial/by-id/`。
- **USB 相机**：4× USB3.1 是优势；D435i/Astra 尽量直连，别经 hub。
- **共享内核**：Android 侧与 Ubuntu 侧共享 `/dev`，注意不要两边同时抢设备。

---

## 9. 开工顺序（新会话照做）

0. 先在 FV04 上跑 `scripts/fv04_probe.sh`，把结果贴进 `docs/fv04_platform.md`。
1. 读本文 + 旧仓库 `docs/safety_and_incidents.md`、`docs/HANDOFF_交接文档.md`、
   `docs/ARCHITECTURE_REVIEW_2026-11-10.md`。
2. 建骨架：README、`.gitignore`、`docs/` 目录；`git commit` 一个"目录骨架"。
   （可用旧仓库的 `scripts/migrate_to_fv04.sh --dry-run` 先看会搬什么，再去掉 `--dry-run` 执行）
3. 搬"零改动"资产：`interfaces`、`bringup`、`config`、`DH_Params`、URDF + STL。
4. 改 ROS 版本：`package.xml`、`CMakeLists.txt`、launch、`ros2_control` 2.x。
5. 在 FV04 上先跑**纯软件**验证：`ros2 launch` 起 TF、RViz 看车+臂+双相机（无硬件）。
6. 再接机械臂 REF（USB CDC）→ 验证 `/joint_states`。
7. 最后碰底盘：先做 §8.1 的 PWM 实测与"信号丢失"安全测试，再写 `chassis_node`。
8. 交互模块放在"能手动开车"这个最小可用点上：Web（浏览器）优先，Android App 后置。

---

## 10. 待用户提供/确认

1. ~~PWM 探测~~ → **已完成，见 §12**；
2. 底盘控制器"拔掉油门信号"的真实行为（**仍未测，决定安全方案**）；
3. ~~FV04 SSH 访问~~ → **已完成：`ssh aidlux@172.20.10.2`，密码同用户名**；
4. 新仓库是否要公开（影响 README 内容与图片）；
5. **FV04 排针引脚定义**（阿加犀 A8550PF1 手册"表 3"）——GPIO 软件 PWM 选脚必需；
6. 是否同意在板子上装 `ros-humble-ros2-control` / `ros-humble-moveit`（apt，来自 TUNA 镜像）。

---

## 11. 旧仓库关键文件索引

```
docs/safety_and_incidents.md          事故史 + 硬性约束（必读）
docs/HANDOFF_交接文档.md               机械臂真值表/映射/避坑（必读）
docs/architecture_v2.md               VENTUNO Q 时代整机架构（参考，不要照搬）
docs/chassis_control_verified.md      底盘信号实测
docs/POWER_v2_SPEC.md                 供电 v2
docs/ARCHITECTURE_REVIEW_2026-11-10.md 现状复盘 + 下一版继承清单
docs/M0_spec.md                       机械臂规格冻结
docs/M2_spec_j6_blade_home.md         J6 零点与剪切行程
scripts/arm_kin.py                    正逆解
scripts/arm_safety.py                 扫掠碰撞
scripts/calib_eye_to_hand.py          手眼标定
src/pickorange_interfaces             msg/action/srv
src/pickorange_bringup                TF
```


---

## 12. FV04 实测记录（2026-11-10，只读探测）

连接方式：`ssh aidlux@172.20.10.2`（用户名密码相同，`sudo` 可用）。
探测脚本：`scripts/fv04_probe.sh`。

### 12.1 系统

| 项 | 实测 |
|---|---|
| OS | Ubuntu 22.04.2 LTS (Jammy)，aarch64 |
| 内核 | `5.15.78-android13`，SMP **PREEMPT**（非 RT） |
| PID 1 | **systemd**；`aid-appmon` / `aid-desktop` / `aid-lms` / `aidcode` 等在跑 |
| CPU / 内存 | 8 核 / 14 GiB（可用 12 GiB） |
| 磁盘 | 84 G，已用 19 G，**剩 65 G** |
| Python | 3.10.12；numpy 1.26.4；cv2 4.12.0 |
| 网络 | `wlan0 172.20.10.2/28`；`br-lan 192.168.1.1/24`；**外网可达**（github 200） |

### 12.2 ROS

| 项 | 实测 |
|---|---|
| 安装 | `/opt/ros/humble` 存在，286 个包 |
| 使用 | 需先 `source /opt/ros/humble/setup.bash`（非登录 shell 里 `ros2` 不在 PATH） |
| **缺失** | **MoveIt2 / ros2_control / controller_manager 未安装**（只有 `joint_state_publisher` 等） |
| 可装性 | apt 源已配 TUNA：`ros-humble-ros2-control` 候选 2.54.3 ✅；`packages.ros.org` 不通（000），**必须走 TUNA** |

### 12.3 外设与总线

| 项 | 实测 | 影响 |
|---|---|---|
| USB | 4 条总线（2×USB3 + 2×USB2） | 相机/机械臂够用 |
| USB 串口 | `CONFIG_USB_ACM=y`、`CONFIG_USB_SERIAL=y`，ch341/cp210x/ftdi 内置 | **REF 机械臂可直插**，/dev/ttyACM* 会出现 |
| I2C | `CONFIG_I2C_CHARDEV not set`，无 `/dev/i2c-*`，无 i2c-dev 模块 | **PCA9685 / 任何 I2C 外设都不可用** |
| SPI | 无 spidev / spi_master | SPI 外设不可用 |
| 摄像头 | `/dev/video0/1` 存在（平台），未接 UVC 设备 | 需插入相机后复测 |
| GPIO | `f000000.pinctrl`：GPIO 301–511（PINS 0–210）；排针 pin N → GPIO 336+N | 软件 PWM 的唯一通道 |

### 12.4 PWM（结论：无排针硬件 PWM）

| chip | npwm | 占用情况 |
|---|---|---|
| `pwmchip0`（PM8550 pwms@e800） | 5 | red / green / blue LED、`pwm-fan`、1 路空闲 |
| `pwmchip5`（pwms@eb00） | 1 | 屏幕背光 `qcom,mdss_dsi_ft8719`（enabled） |
| `pwmchip6`（pwms@ea00） | 1 | 空闲 |

- 三个 chip **全部是 PMIC（PM8550）内部通道**，驱动 LED/背光/风扇，**没有引到 40pin 排针**；
- `pinmux-pins` 里唯一与 PWM 相关的占用是 `pin 9 → soc:pwm-fan`；
- **SoC pinmux 的 274 个函数里没有任何通用 PWM**：全表只有 3 条
  `tsense_pwm1/2/3`（热敏激励，都在 `gpio50`），其余是 gpio/uart/i2s/i2c/cam_mclk/can 等；
- 设备树里**没有 PWM 控制器节点**（只有 PMIC 下的 `pwms@e800/ea00/eb00`）。

⇒ 结论不依赖"排针有没有引出 PWM 脚"：**当前固件根本没有把 PWM 控制器使能/复用到排针**，
80 个排针里没有任何一脚能输出硬件 PWM。要改必须动设备树 + 内核（vendor 固件），不在应用层范围内。

⇒ **"从排针直接输出 3 路 50Hz 舵机 PWM"只能靠 GPIO 软件 PWM**，
其余路线（I2C/SPI/硬件 PWM）在本板上不成立。

### 12.5 排针

- **Raspberry 40PIN**（3.3 V 域，含 GPIO/UART）← 接舵机信号用这个；
- **Qualcomm 40PIN**（1.8 V 域，含 UART/CAN/I2S）← 不要用；
- 具体引脚定义需查阿加犀 A8550PF1 手册（"表 3"），**尚未拿到**。
