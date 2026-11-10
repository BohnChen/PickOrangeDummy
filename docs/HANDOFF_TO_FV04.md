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
| 系统 | AidLux 融合系统：Android + Ubuntu 22.04 共享内核；浏览器 `ip:8000` 进桌面，也可 VNC/SSH |
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
| 开机自启 | systemd | **AidLux 无 systemd**，用 AidLux 应用自启动或脚本 |

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

### 8.1 底盘 PWM：能不能全在板子上做（**待实测，先别写代码**）

需要 **3 路 50 Hz 舵机 PWM**（油门 / 转向 / 档位，脉宽约 1000–2000 µs）。
FV04 上没有 MCU，所以有三条路：

| 方案 | 前提 | 稳定性 | 安全性 | 结论 |
|---|---|---|---|---|
| A. 板载**硬件 PWM**（`/sys/class/pwm`） | 设备树把 QCS8550 的 PWM 引到了排针 | 高（内核定时） | 中：进程死/PWM 保持最后值 | **首选，先验证** |
| B. **PCA9685**（I2C 16 路 PWM 模块） | I2C 可用 | 高（芯片硬件定时） | 同上 | A 不成立时的替代，¥10 |
| C. GPIO 软件模拟 PWM | 任意 GPIO | 低（抖动，负载下漂移） | 低 | **不推荐**用于油门/转向 |
| D. 外置 MCU（旧方案） | 一块 Arduino | 高 | 高（租约 + 锁存急停） | 安全兜底方案 |

**在 FV04 上先跑这三条命令**（决定 A 是否成立）：

```bash
ls /sys/class/pwm/                       # 有没有 pwmchip
cat /sys/kernel/debug/pwm                # 各通道占用情况（root）
cat /sys/kernel/debug/pinctrl/*/pinmux-pins | grep -i pwm   # PWM 引到哪些脚
```

再看排针：**用 Raspberry 40PIN（3.3 V）**，不要用 Qualcomm 40PIN（1.8 V）。
需要确认这三个脚在设备树里是 PWM 功能、且未被背光/风扇占用。

**安全缺口（比 PWM 本身更重要）**：
Linux 侧生成 PWM 时，进程崩溃/系统卡死 **PWM 会保持最后值**，
而旧实测已证明「档位信号浮空可能被判为前进」。
所以必须补一条：**底盘控制器在信号丢失时会不会停车？** 用 5 分钟实测：

> 车架空、低速前进中，直接拔掉油门信号线 → 车是停、是滑行、还是继续跑？

- 若会停 → 方案 A/B 可接受，但**物理急停仍是必需项**；
- 若继续跑 → 必须加"信号看门狗"（外置 MCU 只做钳位，不生成 PWM），
  或回到方案 D。

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

- **没有 systemd**：节点拉起用启动脚本 + AidLux 应用自启动；崩溃自愈要自己写。
- **权限**：容器内通常 root，但 `/dev/ttyACM*` 的设备号会漂移 → 用 `/dev/serial/by-id/`。
- **USB 相机**：4× USB3.1 是优势；D435i/Astra 尽量直连，别经 hub。
- **共享内核**：Android 侧与 Ubuntu 侧共享 `/dev`，注意不要两边同时抢设备。

---

## 9. 开工顺序（新会话照做）

1. 读本文 + 旧仓库 `docs/safety_and_incidents.md`、`docs/HANDOFF_交接文档.md`、
   `docs/ARCHITECTURE_REVIEW_2026-11-10.md`。
2. 建骨架：README、`.gitignore`、`docs/` 目录；`git commit` 一个"目录骨架"。
3. 搬"零改动"资产：`interfaces`、`bringup`、`config`、`DH_Params`、URDF + STL。
4. 改 ROS 版本：`package.xml`、`CMakeLists.txt`、launch、`ros2_control` 2.x。
5. 在 FV04 上先跑**纯软件**验证：`ros2 launch` 起 TF、RViz 看车+臂+双相机（无硬件）。
6. 再接机械臂 REF（USB CDC）→ 验证 `/joint_states`。
7. 最后碰底盘：先做 §8.1 的 PWM 实测与"信号丢失"安全测试，再写 `chassis_node`。
8. 交互模块放在"能手动开车"这个最小可用点上：Web（浏览器）优先，Android App 后置。

---

## 10. 待用户提供/确认

1. §8.1 三条 PWM 命令的输出；
2. 底盘控制器"拔掉油门信号"的真实行为；
3. FV04 的 SSH/ADB 访问方式（IP、账号）；
4. 新仓库是否要公开（影响 README 内容与图片）。

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
