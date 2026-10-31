# PickOrangeDummy

低位柑橘采摘机器人：**移动底盘 + 六轴机械臂 + 双相机（结构光 + RGB-D）+ 离线语音**，
全部跑在一台 **Arduino VENTUNO Q** 上（Ubuntu 24.04 / ROS 2 Jazzy / aarch64）。

起点是 peng-zhihui Dummy-Robot 复刻的超紧凑六轴臂，现在扩展到整机自主作业。

## 先读哪几篇

| 文档 | 内容 |
|---|---|
| [`docs/architecture_v2.md`](docs/architecture_v2.md) | **整机软件架构 v2.0（当前）**：分层、进程模型、TF、双相机分工、停车策略、语音、里程碑、待确认项 |
| [`docs/architecture_diagrams.md`](docs/architecture_diagrams.md) | **架构图册**：11 张图（分层 / 进程 / 通信 / TF / 状态机 / 时序 / 安全 / 语音 / 里程碑） |
| [`docs/safety_and_incidents.md`](docs/safety_and_incidents.md) | 事故史 + 硬性安全约束（**必读**） |
| [`docs/HANDOFF_交接文档.md`](docs/HANDOFF_交接文档.md) | 机械臂真值表、映射、避坑清单（**必读**） |
| [`docs/chassis_control_verified.md`](docs/chassis_control_verified.md) | 底盘接线与信号实测结论 |
| [`docs/architecture.md`](docs/architecture.md) | v1.0 机械臂阶段架构（历史，决定 D1–D6 仍有效） |

## 仓库布局

```
docs/         架构 / 安全 / 底盘 / 视觉模型 / 数据集 文档
firmware/     底盘 MCU 固件（VENTUNO Q / STM32H5 / Zephyr）
scripts/      标定、诊断、标定工具与一次性脚本
config/       运行期配置（J6 刀片零点等）
models/       末端执行器与结构件 CAD / 打印文件
dummyarm_AImodel/  柑橘实例分割模型工程（训练/数据/导出）
src/
  pickorange_interfaces/     接口定义（msg / action / srv）— S0 已完成
  pickorange_bringup/        顶层 launch + config/transforms.yaml — S0 进行中
  dummyarm_description/     URDF/Xacro 数字样机（固件限位对齐）
  dummyarm_cad_description/ CAD 导出高质量网格版
  dummyarm_hardware/        ros2_control SystemInterface (C++)
  dummyarm_moveit_config/   MoveIt 2 配置（真机可用）
  dummyarm_perception/      [迁移中] 现有感知节点（HSV 检测器）
  dummyarm_bridge/          3a 阶段桥接（保留，非主路径）
  dummyarm_bringup/         机械臂专属 launch
```

> 整机层新增包（`pickorange_interfaces` / `_chassis` / `_perception` / `_skills` /
> `_mission` / `_voice` / `_safety` / `_bringup`）见架构文档 §7，按 S0–S3 逐步落地。

## 硬件基线（实测）

- **计算**：VENTUNO Q（8 核 Kryo：4×A78C + 4×A55 / 16 GB LPDDR5 / Hexagon NPU 40 TOPS）
- **机械臂**：Dummy V2 复刻，STM32F405 REF，USB-CDC ASCII @115200
  （串口必须用 `/dev/serial/by-id/*REF*`，禁止 `ttyACM*`）
- **底盘**：3 通道 50 Hz 舵机 PWM（油门 / 转向 / 档位），STM32H5 生成，
  500 ms 看门狗 + 斜率限制；**无轮速编码器**
- **相机**：Orbbec Astra Pro Plus（正前方，避障）+ RealSense D435i（朝左 100°，找果）
- **语音**：USB 声卡，离线 KWS + 预生成反馈语

## 当前状态

- [x] 机械臂真机链路（`dummyarm_link` 协议验证）
- [x] 数字样机 + TF 验证
- [x] **ros2_control（C++）+ MoveIt 2 真机闭环**，J6 驱动层隔离锁定
- [x] 视觉测距与高精度触碰验证（SVD 4 点手眼标定，误差 1 mm）
- [x] 底盘 PWM 驱动 + 固件（实车验证）
- [x] **整机架构 v2.3 定稿**（几何已实测，P0 清空）
- [x] **S0-a 接口包** `pickorange_interfaces`（5 msg / 3 action / 1 srv，编译通过）
- [x] **S0-b 整车 TF 树** `pickorange_bringup`（`ros2 launch pickorange_bringup tf.launch.py`）
- [ ] S0-c `chassis_node`（传输层 + 20 Hz 续租约 + `/cmd_vel`）
- [ ] S1 感知：Astra 避障 + YOLO 接入
- [ ] S2 采摘：`/cutter` Action（废弃串口子进程）+ 单目标闭环
- [ ] S3 编排：状态机 + 一停多采 + 语音

## 快速开始（机械臂单机调试）

```bash
cd ~/01_MyProject/PickOrangeDummy
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dummyarm_moveit_config real_moveit.launch.py

# 收工：平稳回 REST → 确认到位 → 失能
./scripts/safe_park_and_exit.sh
```

> ⚠️ 动臂前先读 `docs/safety_and_incidents.md`：**J3 固件角 < 35° 绝对禁止**（历史断臂）。
