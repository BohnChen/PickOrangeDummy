# PickOrangeDummy · 整机软件架构（v2.0）

> 状态：**待确认（草案）**　编写：2026-10-30
> 适用范围：移动底盘 + 六轴机械臂 + 双目（结构光 + RGB-D）+ 离线语音 的整机系统
> 关系：本文取代并扩展 `docs/architecture.md`（v1.0，仅覆盖机械臂阶段 3）。
> v1.0 中仍然有效的决定（D1 混合架构、D2 关节命名、D3 标准服务、D6 仓库根）继续沿用，
> 本文不再重复，只在其上叠加整机层。
>
> ⚠️ 本文末尾 §22 有 **21 个待确认问题**，其中 5 个（P0）会阻塞 S0 开工，
> 4 个（R12–R15）是**已经核实存在、尚未解决**的硬问题。

---

## 0. 一页纸摘要

系统分五层，**只允许向下依赖**：

```
L4 交互/监督   voice_cmd · safety_supervisor · 人工确认
L3 任务编排   mission（tick 驱动状态机 + 转移表 + 守卫）
L2 能力层     全部 Action：arm_move · cutter · chassis_drive · vision_targets · speak
L1 抽象层     ros2_control 硬件插件 + 设备节点（底盘 / 两路相机 / 音频）
L0 实时层     MCU 固件：STM32H5 底盘 PWM 看门狗 · REF 六轴限位 · 物理急停
```

三条不可退让的硬规则：

1. **单一写者**：每个执行器只有一个进程能发指令。禁止第二个进程开同一个串口。
2. **决策层零阻塞**：L3 及以上不允许 `sleep` / `subprocess` / 直接开串口 / 直接读相机。
3. **最底层保护最硬**：任何会让机器动的兜底保护都在 MCU 固件里，不依赖 Linux 活着。

三个关键技术判断（详见对应章节）：

| 判断 | 结论 | 章节 |
|---|---|---|
| 导航方案 | 无轮速编码器 ⇒ 不做 Nav2 建图定位，走**视觉伺服 + 反应式** | §3.3 / §11 |
| 转向机构 | 中心摇臂 + 左右横拉杆，属**阿克曼族**；运动学用自行车模型；`steer→δ` 非线性，必须查表 | §3.4 / §10 |
| 任务编排 | **不用行为树**，用「异步技能层 + tick 驱动状态机」；编排器可替换 | §14 |

三个**必须先解决的既成事实**（不解决则对应阶段无法开工）：

| 事实 | 影响 | 章节 |
|---|---|---|
| 底盘串口 `/dev/ttySTM0` 被 `arduino-router` 独占，`arduino` 用户无 socket 权限 | **S0 第一天卡死** | §6.5 / Q4b |
| `cut.py` 与 ros2_control 抢串口 | **S2 剪切必然失败** | §20.1 |
| D435i 要到目标 `x≈+0.42 m` 才首次看见 ⇒ 远距离预判不可行 | 停车策略必须闭环逼近 | §3.5 / §12.3 |

一个必须提前知道的数字：**单次剪切约 17 秒**（J6 单程 60°，固件 speed=5.0 ≈ 3.6°/s；
合刀 + 张刀合计约 34 秒）。这意味着一次"停车 → 采一个果 → 继续走"的完整循环约 **60–90 秒**。

---

## 1. 目标与验收场景

### 1.1 近期验收场景（S3 的 Definition of Done）

> 楼道里随机悬挂若干个橘子 → 小车低速巡航 → 发现目标 → **立刻停车 + creep 闭环逼近** →
> 机械臂套住目标 → **等待剪切指令** → 剪切 → 张刀 → 退刀 →
> **直接移动到下一个目标** → 队列采完 → 回 REST → 继续巡航。

约束与关键词：

- **无地图、无里程计、无全局定位**
- **一停多采**：一次停车尽量采完该位置可达的多个目标
- **语音可中断**：任何时刻可以用语音暂停/停止
- **剪切必须人工确认**（第一版）；确认后手臂自动完成剩余动作
- 运行环境：室内楼道，光照稳定，地面平整

### 1.2 中期目标（不在本轮实现，但架构要留口）

- 校园/果园土路，颠簸、逆光、枝叶遮挡
- 多目标连续作业、自动换位、避障绕行
- 力/接触感知（判断果实是否套到位、果梗是否被夹住）
- NPU 加速全链路

---

## 2. 非目标与反目标

**非目标（本轮明确不做）**

- Nav2 / SLAM / 栅格地图 / 全局路径规划
- 视觉里程计（VIO）—— 可选，但不是第一版依赖
- 多机协同、云端推理、远程遥控
- 力控 / 阻抗控制
- PREEMPT_RT 内核

**反目标（技术上能跑但项目上失败的情形）**

| 反目标 | 为什么算失败 |
|---|---|
| 演示时全靠人工按顺序点脚本 | 架构没解决问题，只是把复杂度挪到人身上 |
| 任意一个节点崩溃，车继续动 | 安全不可接受 |
| 加一个新模块要改 5 个地方 | 违背"便于加模块"的初衷 |
| 只有作者本人能改流程 | 编排逻辑耦合在代码细节里，无法演进 |
| 为了"先进"引入 Nav2 / 行为树，但实际只有一个线性流程 | 复杂度高于收益，长期维护成本反噬 |

---

## 3. 硬件基线（实测）

### 3.1 计算平台（`arduino` 用户实测，2026-10-30）

| 项 | 实测值 |
|---|---|
| 内核 | `6.8.0-1084-qcom` aarch64 |
| CPU | 8 核：CPU 0–3 = Cortex-A78C（2×2.11 GHz + 2×2.36 GHz），CPU 4–7 = Cortex-A55（1.96 GHz） |
| 内存 | 14 GiB 可见（物理 16 GB LPDDR5） |
| 存储 | eMMC，根分区 54 G，已用 33 G，**剩余 19 G** |
| NPU | Hexagon Tensor Processor，标称 40 dense TOPS（`fastrpc-cdsp` 设备存在） |
| GPU | Adreno 623 |
| 抢占模型 | `CONFIG_PREEMPT_DYNAMIC=y`，**不是 PREEMPT_RT** |
| 调频 | `schedutil` |
| 系统 | Ubuntu 24.04 noble / ROS 2 Jazzy / Python 3.12.3 |
| 已装相机驱动 | `ros-jazzy-realsense2-camera` 4.58.4、`ros-jazzy-librealsense2` 2.58.4 |
| 用户组 | `dialout`（串口）、`audio`（USB 声卡）、`video` ✔；**不在 `arduino-router` 组** |

**底盘链路的真实形态（实测，S0 的第一道坎）**

```
STM32H5 ──/dev/ttySTM0──► arduino-router（systemd, root:arduino-router）
                              │  msgpack-RPC
                              ├── /var/run/arduino-router.sock  (srw-rw---- root:arduino-router)
                              └── 127.0.0.1:7500  (MCU monitor proxy，只读串口输出)
```

- `arduino-router` **独占** `/dev/ttySTM0`，任何进程都不能直接开这个串口。
- 宿主机的 `/usr/bin/python3` **没有** `arduino` 模块（`import arduino.app_utils` 失败）；
  该模块只存在于 App Lab 容器镜像里。
- `arduino` 用户不在 `arduino-router` 组（组 gid=128），**读不到 unix socket**。

⇒ **写 chassis_node 之前必须先解决 socket 访问权**，方案见 §6.5。

**推论**

- 存储是**当前最紧的资源**。rosbag + 模型权重 + 数据集很快会撞墙。建议加 M.2 NVMe（见 Q11）。
- NPU 不是第一版依赖：工具链需在 x86 上做 QNN/QAIRT 转换，且 ROS2 生态现成的是 TFLite / ONNX Runtime。第一版全部走 CPU。

### 3.2 机械臂（沿用 v1.0，实测）

- Dummy V2 六轴复刻，STM32F405 REF 主控，USB-CDC ASCII @115200
- 串口**必须**用 `/dev/serial/by-id/usb-Robot_Embedded_Framework_REF_1.0_CDC_Interface_365D357B3334-if00`
- 关节映射、硬限位、避坑指南见 `docs/HANDOFF_交接文档.md`
- **J3 红线**：固件角 < 35° 绝对禁止（历史断臂）
- **J5 红线**：|角| ≤ 95°（超过 105° 与 J4 壳体物理干涉）
- J6 多圈不跨上电，会话零点存 `config/j6_blade.json`；刀片全开 = `j6_home_deg`，合刀 = `-60°`

### 3.3 底盘

- 3 通道 50 Hz 舵机 PWM 控制（油门 / 转向 / 档位），其余 3 路必须保持中位
- 通道映射：`1→D9, 2→D10, 3→D3(油门), 4→D5(转向), 5→D6(档位), 6→D11`
- 板子前提：**SW2 拨码第 8 位必须打开**；接收机串阻 R89/R90/R91 已拆（遥控失效）
- MCU 侧：500 ms 看门狗、每帧 1% 斜率限制、|指令|<3% 死区、上电全中位
- Linux 侧 RPC：`drive(speed, steer)` / `set_pwm` / `stop` / `brake` / `ping`
- **无轮速编码器**（已确认）⇒ 无 `odom` 来源
- 满舵 ±400 µs，实用油门 30% 可稳定起步

详细引脚与排障见 `docs/chassis_control_verified.md`（v2 已验证版，优先于 wiring 文档）。

### 3.4 转向机构（用户描述，2026-10-30）

> 车头中间一台减速电机，输出轴固定一块带左右螺孔的厚板；左轮转向节经横拉杆接板左侧螺孔，
> 右轮转向节经横拉杆接板右侧螺孔；电机转动，两轮随之转向。

**定性**：这是**中心摇臂 + 双横拉杆**结构，两个前轮各有独立转向节，绕摇臂中心旋转时
两铰点的位移方向与量不同 ⇒ **属于阿克曼族**（用摇臂代替齿条），会产生内外轮差角。

**对软件的三个结论**

1. 接口仍是**一个标量 `steer`**，内部映射到一个"有效转角 δ"。
2. 运动学用**自行车模型** `R = L / tan(δ)`，`L` = 轴距。控制器不做阿克曼补偿（机构已做一部分）。
3. 摇臂经连杆传递是**正弦型非线性** ⇒ `steer → δ` **必须用查表 + 插值**，不能用线性 gain。

### 3.5 相机（安装几何，用户描述）

| 相机 | 安装位置 | 朝向 | 用途 |
|---|---|---|---|
| Orbbec Astra Pro Plus | 车头最前方横梁上竖立的 2040 型材顶端，离地约 **310 mm** | 正前方 | 通行避障（只测距离，不做语义） |
| RealSense D435i | Astra **正下方约 15 mm** | 从正前方**逆时针 100°**（即朝向车体**左侧**），俯仰与 Astra 不同 | 柑桔目标检测与三维定位 |

Astra Pro Plus 规格参考：深度 640×480@30，量程约 0.6–8 m，H 约 58°、V 约 45.5°。
D435i 规格：深度量程约 0.2–10 m，H 约 87°、V 约 58°。

**几何推论（已核算，与"一停多采"直接相关）**

相机光轴 = 从车头前方向左转 100° = **(cos100°, sin100°) = (−0.174, +0.985)**（前, 左）。
相机相对 `base_link`（机械臂基座）在 **(+0.165, −0.040)**。则目标距相机 `d` 时：

```
x_base = 0.165 − 0.174·d
y_base = −0.040 + 0.985·d
```

| d (m) | x_base | y_base | 是否在停车窗口 |
|---|---|---|---|
| 0.40 | +0.096 | +0.354 | ✅ 窗口中心 |
| 0.50 | +0.078 | +0.452 | ⚠️ 侧向偏远 |
| 0.60 | +0.061 | +0.552 | ❌ 超臂展 |

**这条式子解释了一件关键的事**：相机装在机械臂**前方** 165 mm、又朝**后侧方** 100°，
两者恰好抵消——**目标"看起来很靠侧方"的时候，正好落在机械臂侧向可达区**。
这不是巧合，是安装时选的角度合适。

但它也带来一个**必须接受的事实**：

> 目标只有在 `x_base ≈ +0.42 m` 时才**第一次进入** D435i 视场
> （此时相对相机的方位角 = 56.5°，刚好是视场边缘）。

**含义**：巡航阶段 D435i 是"侧扫"而非"前视"，**没有远距离预判能力**。
所以"远距离提前减速"这条路不通——停车策略必须改成 §12 的
**"发现即停 + 视觉闭环逼近"**。这是本次架构里被实测推翻重写的一处。

### 3.6 机械臂安装位置（用户描述，待补高度）

- 机械臂底座中心：在相机立杆**后方约 165 mm**、**向左约 40 mm**
- **高度差未知**（Q1，P0 阻塞项）

### 3.7 语音

- **USB 声卡**（输入输出同一设备）
- 板载另有 MAX98091 编解码器（JMISC），本轮不用
- 用户 `audio` 组已具备

---

## 4. 设计原则

1. **单一写者**：每个执行器只有一个写者。串口/设备句柄所有权必须唯一。
2. **动作即 Action**：任何"有完成语义"的操作都是 Action（带 feedback / result / cancel / 超时），
   不是"发一条 Topic 然后猜"。
3. **只向下依赖**：L3 不知道 L1 的存在；L3 只会调 L2 的 Action。
4. **决策无阻塞**：L3 及以上零 `sleep`、零 `subprocess`、零设备句柄。
5. **保护往下沉**：越底层的保护越硬、越不依赖上层活着。
6. **每一步都可失败**：每个动作有明确失败语义、错误码、回退路径，并计入 `/robot_state`。
7. **编排器可替换**：L2 的 Action 契约是稳定边界。L3 换实现（状态机 ↔ 行为树 ↔ 脚本）不应波及 L1/L2。

---

## 5. 分层架构

```text
┌──────────────────────────────────────────────────────────────────────┐
│ L4  交互 / 监督                                                       │
│   voice_cmd（KWS→指令闸门）   safety_supervisor（全局否决）  人工确认 UI │
├──────────────────────────────────────────────────────────────────────┤
│ L3  任务编排                                                          │
│   mission：tick 驱动状态机 + 转移表 + 状态守卫 + 目标队列                 │
├──────────────────────────────────────────────────────────────────────┤
│ L2  能力层（Action / Service，非阻塞、带 feedback、可取消、有超时）        │
│   /arm/move_to_pose  /cutter/cut  /chassis/creep  /voice/speak         │
│   /arm/check_reachable  /vision/query_targets                         │
├──────────────────────────────────────────────────────────────────────┤
│ L1  抽象层（硬件抽象与设备驱动）                                        │
│   dummyarm_hardware(C++ ros2_control)   chassis_node（20 Hz 喂狗）      │
│   astra_guard（深度→障碍）  target_detector（YOLO→3D）  audio_io        │
├──────────────────────────────────────────────────────────────────────┤
│ L0  实时层（MCU 固件，永不进 ROS）                                      │
│   STM32H5：6 路 50 Hz PWM + 500 ms 看门狗 + 斜率限制                    │
│   STM32F405：六轴步进 + 硬限位 + !STOP + 本机保护                       │
└──────────────────────────────────────────────────────────────────────┘
   横切：TF 树 · /robot_state 黑板 · /diagnostics · rosbag2 · 参数与标定文件
```

### 5.1 各层职责边界

| 层 | 有权做 | 禁止做 |
|---|---|---|
| L4 | 读全局状态、发指令请求、否决 | 直接驱动任何执行器 |
| L3 | 决定"下一步做什么" | 开串口、读相机、阻塞等待 |
| L2 | 把一个请求变成设备动作并报告完成 | 决定"要不要做" |
| L1 | 设备读写、协议编解码、本地安全兜底 | 业务判断 |
| L0 | 字节级时序与本地保护 | 任何与 ROS 相关的事 |

---

## 6. 进程与线程模型

### 6.1 进程划分（这是"有序高效"的物理基础）

| 进程 | 频率 | 为什么必须独立 |
|---|---|---|
| `ros2_control_node` + `dummyarm_hardware` | **20 Hz**（`moveit_config` 现值）/ 50 Hz（`bringup` 值） | 串口读写线程，被视觉或规划卡 1 帧即轨迹超时。**不要盲目提到 100 Hz**：`#GETJPOS` 在 115200 波特率下的带宽上限只有约 50 Hz 量级，提频会挤掉写通道 |
| `chassis_node` | 20 Hz 心跳 | 500 ms 看门狗，慢一拍就停车；还要做斜率/互锁 |
| `astra_guard` | 30 Hz | 前方障碍，只出"最近距离 + 建议" |
| `target_detector` | 15–30 Hz | CPU 最重的推理，独立进程 + 独立 QoS |
| `mission` | 10 Hz tick | 唯一决策大脑，单线程，零阻塞 |
| `voice_io` | 事件驱动 | 常开监听，播放时半双工静音 |
| `safety_supervisor` | 50 Hz | 心跳监控 + 互锁，**优先级最高** |

**原则：会动的各占一个进程，会卡住的绝不共进程。**

### 6.2 进程内并发

- rclpy：`MultiThreadedExecutor`；每个 Action server 一个 `ReentrantCallbackGroup`，
  定时器用 `MutuallyExclusiveCallbackGroup`，避免回调互相饿死。
- C++ 硬件插件：沿用现有「串口读线程 + 双缓冲快照」；`send_commands()` **保持纯异步写**
  （历史坑：阻塞 `readLine` 会饿死 Action Server 事件循环，导致 19 s 超时强杀）。
- `chassis_node` 的心跳线程**独立于 ROS 回调**，用独立 `threading.Thread` + `Event`，
  即使 ROS executor 卡住也能继续喂狗或主动刹车。

### 6.3 CPU 绑核与优先级（已实测硬件拓扑）

| 用途 | 绑定 | 说明 |
|---|---|---|
| 控制/通信（`ros2_control`、`chassis_node`、`safety_supervisor`） | CPU 0–3（A78C） | 高频、低负载、要低抖动 |
| 视觉 / ASR / TTS | CPU 4–7（A55） | 重负载、允许几十 ms 抖动 |
| `chrt -f 80` | 施加于控制线程 | 50 Hz 环抖动可从 ms 级降到数百 µs |
| governor | `schedutil` → `performance`（演示时） | 消除调频引入的周期抖动 |

这些是**配置项**，不是代码，落在 `pickorange_bringup/config/` 的 systemd 服务或 launch 里。

### 6.4 为什么不用 PREEMPT_RT

- 当前内核 `PREEMPT_DYNAMIC`，非 RT。
- 本项目的实时性需求最高是 50 Hz 关节环（20 ms 周期）+ 50 Hz PWM（20 ms 周期），
  **都在 MCU 或普通 Linux 上足够**。
- RT 内核在 Qualcomm BSP 上重编译风险高、收益 <1 ms。
- 唯一会改变结论的场景：做**力控/阻抗控制**。届时更优解是**把那个环下放到 STM32H5**，
  而不是给 Linux 打补丁。

---

### 6.5 底盘接入方案（S0 必须先解决，实测结论）

`arduino-router` 是 msgpack-RPC 服务端，独占 `/dev/ttySTM0`；`arduino` 用户读不到它的 unix socket，
宿主机的 Python 也没有 `arduino.app_utils` 模块。三个可选路径：

| 方案 | 做法 | 优点 | 缺点 | 建议 |
|---|---|---|---|---|
| **A. 加组 + 自写 msgpack 客户端** | `sudo usermod -aG arduino-router arduino`；`chassis_node` 用 `msgpack` 直连 `/var/run/arduino-router.sock`，发 `[0, msgid, "drive", [speed, steer]]` | 脱离 App Lab；一条 `ros2 launch` 起全栈；依赖极少（只要 `python3-msgpack`） | 需要一次 sudo + 重新登录（或用 `sg`） | ✅ **推荐** |
| B. 给 router 加 TCP 端口 | systemd drop-in 加 `--listen-port 127.0.0.1:7501` | 不需要组权限 | 改动系统服务；多开一个攻击面 | 备选 |
| C. 跑进 App Lab 容器 | 用 `arduino-app-cli` 部署 | 官方路径 | 破坏"一条 launch 起全栈"；容器内 DDS 与宿主 ROS 需要额外打通 | ❌ 不推荐 |

**A 的落地细节**

- 不重新登录的临时办法：`sg arduino-router -c "ros2 launch pickorange_bringup robot.launch.py"`
- 更稳的办法：把整栈做成 systemd 服务，加 `SupplementaryGroups=arduino-router`，
  开机会员资格生效，且天然获得 `Restart=always`。
- `chassis_node` 必须在**连接断开后自动重连**（router 重启时 `Restart=always` 会重启它）。
- 不要用 `Bridge` 的 Python 包，也不要 `pip install arduino`——那是容器内 SDK。

---

## 7. 包与仓库布局

```text
src/
  pickorange_interfaces/     # msg / action / srv（新增，无外部依赖）
  pickorange_chassis/        # chassis_node：/cmd_vel → RPC、心跳、互锁、状态
  pickorange_perception/     # astra_guard + target_detector + 目标融合
  pickorange_skills/         # 异步/可轮询技能库（Python，L3 只依赖它）
  pickorange_mission/        # 状态机编排 + 目标队列
  pickorange_voice/          # KWS + TTS + 指令闸门
  pickorange_safety/         # safety_supervisor：心跳、互锁、estop 统一入口
  pickorange_bringup/        # 顶层 launch、参数、标定文件、systemd 单元

  dummyarm_description/      # 机械臂 URDF（保持不变）
  dummyarm_cad_description/  # CAD 网格版（保持不变）
  dummyarm_hardware/         # C++ ros2_control 插件（扩展：/cutter 接口）
  dummyarm_moveit_config/    # MoveIt 配置（保持不变）
  dummyarm_bringup/          # 机械臂专属 launch（保留，供单臂调试）
  dummyarm_bridge/           # 3a 阶段桥接（保留，非主路径）
  dummyarm_perception/       # 现有 HSV 检测节点（迁移到 pickorange_perception 后废弃）
```

### 7.1 依赖方向

```
pickorange_interfaces  ←──  所有其它包
pickorange_skills      ←──  pickorange_mission
pickorange_mission     ──→  pickorange_skills（仅此一处）
pickorange_bringup     ──→  全部（仅 launch 聚合）
```

**禁止**：`pickorange_mission` 直接依赖 `pickorange_chassis` / `pickorange_perception` / 任何 driver。
mission 只通过 L2 的 Action/Service 说话。

---

## 8. 接口契约

### 8.1 Topic

| 话题 | 类型 | 频率 | 发布者 → 订阅者 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | 50 Hz | mission/chassis → chassis_node |
| `/chassis/state` | `ChassisState`（自定义） | 20 Hz | chassis_node → all |
| `/perception/targets` | `TargetArray` | 15–30 Hz | target_detector → mission / RViz |
| `/perception/hazards` | `HazardReport` | 30 Hz | astra_guard → mission |
| `/robot_state` | `RobotState`（自定义） | 10 Hz | mission → all（含 voice / UI） |
| `/diagnostics` | `diagnostic_msgs` | 1 Hz | 各节点 |

用 `TwistStamped` 而非 `Twist`：Jazzy 的 Nav2 已切 Stamped，将来接 Nav2 不用改。

### 8.2 Action（L2 能力层）

| Action | Goal | Feedback | Result | 实现位置 |
|---|---|---|---|---|
| `/arm/move_to_pose` | `PoseStamped target`, `Vector3 approach_dir`, `float32 vel_scale` | `progress`, `current_pose` | `success`, `error_code` | 包装 MoveIt `MoveGroup` |
| `/cutter/cut` | `float32 span_deg`, `float32 dwell_s` | `current_angle_deg`, `phase` | `success`, `final_angle_deg`, `error_code` | **扩展 `dummyarm_hardware`** |
| `/chassis/creep` | `float32 distance_m`（±）, `float32 max_speed` | `traveled_m`, `remaining_m` | `success`, `error_code` | chassis_node |
| `/voice/speak` | `string text`, `bool wait` | — | `success` | voice_io |

### 8.3 Service

| Service | 类型 | 作用 |
|---|---|---|
| `/arm/check_reachable` | `CheckReachable` | 用 MoveIt `compute_ik` + 碰撞检测判可达（§11.3） |
| `/mission/start` `/stop` `/pause` `/resume` | `Trigger` | 任务生命周期 |
| `/mission/approve_cut` | `Trigger` | **剪切人工确认**（HOLD 状态才有效） |
| `/safety/estop` | `Trigger` | 软急停：取消所有 Action + 底盘 brake + 臂 `!STOP` |
| `/safety/reset` | `Trigger` | 清错误、恢复 IDLE（需重新 enable） |
| `/safety/enable` | `Trigger` | 允许运动（上电默认不允许） |

### 8.4 消息定义草案（`pickorange_interfaces`）

```text
# msg/Target.msg
uint32   id
string   class_name          # orange | occluded_orange | pedicel | tree
float32  confidence
geometry_msgs/Point  position    # base_link 下，米
float32  radius_m
bool     reachable
string   reject_reason       # 不可采原因，便于调试
builtin_interfaces/Time  stamp

# msg/TargetArray.msg
std_msgs/Header header        # frame_id = base_link
Target[]        targets       # 已按优先级排序
uint32          selected_id

# msg/ChassisState.msg
int8     MODE_STOP=0
int8     MODE_FWD=1
int8     MODE_REV=2
int8     MODE_FAULT=3
int8     mode
float32  throttle_pct
float32  steer_pct
bool     watchdog_ok
float32  watchdog_remaining_s
uint32   cmd_seq               # 已接受的指令序号

# msg/HazardReport.msg
float32  nearest_distance_m
float32  nearest_bearing_rad   # 相对车头，左正右负
int8     SUGGEST_GO=0
int8     SUGGEST_SLOW=1
int8     SUGGEST_STOP=2
int8     suggestion
bool     valid                 # 数据是否可信（丢帧/遮挡时为 false）

# msg/RobotState.msg
string   mode                  # IDLE | CRUISE | SLOW | STOP | PICK | HOLD | FAULT | ESTOP
string   task
string   substate
uint32   error_code
string   error_text
bool     voice_listening
bool     estop_active

# action/CutStem.action
float32 span_deg
float32 dwell_s
---
float32 current_angle_deg
string  phase                  # close | dwell | open | verify
---
bool    success
float32 final_angle_deg
string  error_code

# action/MoveToPose.action
geometry_msgs/PoseStamped target
geometry_msgs/Vector3    approach_dir
float32                  vel_scale
---
float32 progress
geometry_msgs/PoseStamped current_pose
---
bool   success
string error_code

# action/Creep.action
float32 distance_m
float32 max_speed
---
float32 traveled_m
float32 remaining_m
---
bool   success
string error_code

# srv/CheckReachable.srv
geometry_msgs/PoseStamped target
float32 clearance_m
---
bool   reachable
string reason
```

---

## 9. TF 树与坐标系

### 9.1 目标 TF 树

```text
odom  ──(静态单位变换, 占位)
 └── base_footprint            ← 后轴中心地面投影（新增根）
      ├── chassis_link         ← 车体
      ├── base_link            ← 机械臂基座法兰中心（现有，不动）
      │    └── Joint1_link … Joint6_link ── tool0 ── tcp
      ├── astra_link           ← 车头立杆顶，离地 ~310 mm
      │    └── astra_depth_optical_frame
      └── d435i_link           ← Astra 正下方 ~15 mm，yaw +100°
           └── camera_color_optical_frame
```

### 9.2 关键决定

1. **`base_link` 继续等于机械臂基座中心，不变。**
   这样 URDF / SRDF / MoveIt 配置 / 现有 IK 链**零改动**，只在它上面加父节点。
2. **先发一个单位 `odom → base_footprint` 静态变换。**
   现在没有编码器，它是占位，但树上留了这个位置。将来加编码器或 VIO，下游一行都不用改。
3. **`base_footprint` 选在后轴中心地面投影。** 自行车模型的旋转中心在此，转弯几何最自然。
   （若 Q4 有更好选择，以 Q4 为准。）
4. 所有静态变换参数放 `pickorange_bringup/config/transforms.yaml`，**不写死在 launch 里**
   （v1 把 6 个外参写死在 `perception_bringup.launch.py`，是维护陷阱）。

### 9.3 ⚠️ 现有手眼外参整体作废

`src/dummyarm_perception/launch/perception_bringup.launch.py` 中的：

```
cam_x=-0.1959  cam_y=-0.2986  cam_z=+0.4678
cam_roll=0.5252  cam_pitch=1.2661  cam_yaw=0.4923
```

是"相机固定在机械臂**右前上方 46.8 cm 高位俯视**"时用 SVD 4 点法标定的。
**新机位（车头 310 mm、朝左 100°）与之几何关系完全不同，不能微调，必须整块重建。**

### 9.4 标定顺序（一次性，做完长期有效）

因为相机与机械臂现在都**固定在车体上**，属于 eye-to-hand 固定外参，标一次即可。

1. 用卷尺/卡尺量 `base_footprint → base_link`、`→ astra_link`、`astra → d435i_link` 的
   平移与姿态（Q1–Q3 提供数据）。
2. 写进 `transforms.yaml`，发 `static_transform_publisher`。
3. 用现有 `scripts/calibrate_camera_extrinsics.py` 做 4 点法（或棋盘格）精修 D435i 外参。
4. 复验：机械臂 `tool0` 与相机反投影目标在 `base_link` 下重合误差目标 **< 1 cm**。
5. 把最终值写回 `transforms.yaml`，并把标定日期、残差记入 §21 风险表旁。

---

## 10. 转向模型与标定

### 10.1 运动学

自行车模型：

```
ẋ = v · cos(θ)
ẏ = v · sin(θ)
θ̇ = v / L · tan(δ)
```

- `L` = 轴距（Q3）
- `δ` = 有效前轮转角（左右两轮的平均）
- `v` = 前进速度（**没有测量值**，只能由油门百分比 + 标定曲线近似）

### 10.2 `steer → δ` 标定（必须做，不能猜）

因为中心摇臂经连杆传递是**正弦型非线性**，用线性 gain 会在中位附近过灵、在极限附近迟钝。

标定步骤：

1. 地面铺胶带，标出后轴中心起点与一条参考直线。
2. `steer` 取 −100, −75, −50, −25, 0, 25, 50, 75, 100（九点）。
3. 每个点以固定低油门直行 3 s，记录轨迹。
4. 拟合圆弧半径 `R`，反算 `δ = atan(L/R)`。
5. 生成 `steer_pct → δ_deg` 查表（9 点 + 线性插值），写入 `config/steering_table.yaml`。
6. 同时记录**最小转弯半径**与**满舵是否顶死**（顶死会堵转发热）。

### 10.3 `throttle → v` 标定（近似即可）

无编码器，速度只能靠时间/距离估算：

1. 铺 3 m 直线胶带。
2. 油门 10/15/20/30% 各跑 3 次，秒表计时，取平均。
3. 得到 `v ≈ k · throttle`，写入 `config/throttle_speed.yaml`。
4. **注意**：电池电压下降会显著变慢，所以这是"标称值"，不是测量值。
   所有基于它的距离控制都必须有**视觉闭环兜底**（§12）。

---

## 11. 感知设计

### 11.1 分工：两台相机严格不混

| 相机 | 唯一职责 | 输出 | 明确不做 |
|---|---|---|---|
| Astra Pro Plus | **通行安全** | `/perception/hazards`：前方最近障碍距离 + 建议 | 不做语义、不识别橘子 |
| D435i | **目标检测** | `/perception/targets`：`base_link` 下目标列表 | 不判断能否通行 |

**不要合并成一个"感知节点"。** 两者频率、QoS、失败模式、标定都不同。

### 11.2 Astra：走廊 ROI 过滤（否则永远停车）

楼道里朝前看总会看到远处墙，必须只关心**车体前方实际会撞到的空间**：

```
ROI（相机系）：
  z ∈ [0.30, 1.50] m        # 不关心地面和天花板
  |x| ≤ 0.35 m              # 车宽半宽 + 余量
取 ROI 内有效深度的 5% 分位数作为 nearest_distance
```

三级建议（带迟滞，避免抖动）：

| 距离 | 建议 | 动作 |
|---|---|---|
| > 1.0 m | `SUGGEST_GO` | 允许巡航 |
| 0.6 – 1.0 m | `SUGGEST_SLOW` | 降到最低速 |
| < 0.6 m | `SUGGEST_STOP` | 刹车并进入 FAULT（等人工） |

### 11.3 D435i：目标检测

```
YOLO11n-seg → mask（orange / occluded_orange / pedicel / tree）
  → mask 一阶矩求质心 (u, v)
  → 取 mask 内部 50% 区域的深度中位数（去除边缘与反光污染）
  → 反投影到相机系 → 经 TF 到 base_link
  → 去重/跟踪（按 3D 距离关联，分配稳定 id）
  → 排序（可达 > 距离近 > 置信度高）
  → /perception/targets
```

**可达性判定要换掉现在的球壳判据。** 现有代码用"离基座 0.12–0.34 m"，这个假设建立在
"机械臂单独放桌上"的年代。在车上，可达区是一个由 IK 决定的**不规则体**。

正确做法：`/arm/check_reachable` 服务 → MoveIt `compute_ik` + 碰撞检测。
项目已经有一条残差 <0.1 mm 的严格 CAD IK 链，这正是它该干的活。

**但要注意（Q21）**：当前 `dummyarm_*` 只有机械臂 URDF，**没有底盘外形**。
`compute_ik` + 碰撞检测只会按"裸臂"判可达，会给出**撞车身或撞墙**的解。
所以 S1 必须给 planning scene 加：

- 一块底盘简化碰撞体（包围盒即可，长宽高待 Q3 之外的实测）
- 可选的"左侧作业空间"虚拟墙（防止规划穿透到车体内部）

> 性能注意：`compute_ik` 每次几十 ms，**不能对每个检测框每帧都调**。
> 策略：只对**排序前 3** 的目标、且**每 200 ms 最多一次**做判定，结果缓存 1 s。

### 11.4 模型接入方式

> ⚠️ **权重不在板上（Q20）**：`dummyarm_AImodel/` 目前只有 `config/` 和 `scripts/`，
> `runs/`（`.gitignore` 忽略）与任何 `.onnx` 都**不存在**。
> 所以 S1 的起跑条件不是"等一个现成部署件"，而是必须包含
> **"在 M4 上导出 opset17 静态 640 ONNX → 传到板上 → 跑通一次推理 → 记录延迟"** 这一段。

第一版：`best.pt → best.onnx (opset 17, 静态 640×640)` + `onnxruntime` CPU（XNNPACK）。
理由：安装简单、无 Python 推理框架依赖、ARM 上实测 19.7 ms 已够 30 FPS。

第二版（可选）：`onnxruntime` QNN EP 或 TFLite delegate → NPU。
前置条件：在 x86 机器上用 Qualcomm QAIRT 做量化与图切分，产出 context binary。
**这是一条独立的支线，不应阻塞主线。**

### 11.5 500 张数据的评估方法（不受架构阻塞，可立刻做）

不要用 `mAP` 判断"够不够"，用**任务指标**判断：

1. 在新机位（车上实际安装角度）拍 60–100 张，覆盖：可采距离、不同光照、部分遮挡。
2. 用现有 `scripts/test_citrus_detector_standalone.py` 跑 `best.pt`。
3. 统计三个数：
   - **可采窗口内检出率**（目标存在时，是否被检出并给出坐标）
   - **果心定位误差**（像素 + 反投影后的 mm）
   - **误检率**（无果实时的假阳性，这会直接导致乱动）
4. 判定线建议：可采窗口检出率 ≥ 90%、果心误差 ≤ 15 mm、误检率 ≤ 5%。
5. 如果明显不达标，先查**机位**——那 500 张若是老的高位俯视机位，
   新视角会有显著域偏移，此时结论是"数据不对"而不是"模型不行"。

---

## 12. 停车策略（"梯度有序停车"）

### 12.1 核心思想

> **车只负责把目标送进"可采窗口"，毫米级交给机械臂。**

机械臂的笛卡尔微调精度是毫米级，所以车不需要精确停在某个点，
只需要保证停车瞬间目标落在窗口内。

### 12.2 停车窗口（必须实测标定）

在 `base_link` 下定义（数值为占位，**待 §21 Q6 实测**）：

```
x ∈ [-0.10, +0.10] m     # 目标大致在臂的正侧方
y ∈ [ 0.25,  0.42] m     # 侧向距离
z ∈ [-0.15, +0.25] m     # 高度
```

标定方法：把真橘子放在不同位置，逐个调 `/arm/check_reachable`，
扫描出"可达 + 无碰撞"的三维边界，画一张俯视投影图贴在工位上。

### 12.3 两段式停车：发现即停 → 视觉闭环逼近（**已推翻原 TTC 方案**）

**为什么不能用"提前减速"**：按 §3.5 的核算，D435i 要到 `x_base ≈ +0.42 m` 才第一次看见目标。
也就是说，**看见的时候距离窗口只剩约 0.5 m 行程**，而且此时车还在匀速前进。
在没有速度反馈的情况下，用开环油门去"刹进 0.2 m 宽的窗口"必然不稳。

同时，§12.3 原来的分档 `x > 0.6 m → 巡航 15%` 这一档**永远不会触发**（看不见）。

**改为两段式**，第二段是闭环，不需要里程计：

```text
段 1  DETECT_STOP       D435i 首次捕获目标 → 立刻 0 油门 + brake
                        （宁可停早，不要开过；停了还能蠕进，开过了就得倒车）

段 2  CREEP_IN          /chassis/creep(+3 cm) → 等 300 ms 稳定 → 重新读目标 x_base
                        重复，直到 x_base ∈ [-0.08, +0.08]
                        最多 8 次；若 x_base 突然跳变 > 10 cm → 回段 1 重新停
                        若 8 次后仍未进窗 → 跳过该目标（黑名单 30 s）
```

**为什么这样更稳**

1. 第二段每步只有 3 cm，**误差不会累积超过一步**，天然免疫"没有里程计"。
2. 每步都用视觉重新测量，等于把 `creep` 做成了闭环伺服，`throttle→v` 标定不准也无所谓。
3. "停早了"是可恢复的错误，"开过了"要倒车——而倒车在这个底盘上更不可控（换向互锁 + 无反馈）。
   **所以策略要偏向"早停"。**

**creep 的实现要求**：`/chassis/creep(distance_m)` 必须是**有限步进**，内部按时间积分
（`distance / v_nominal`）到点即停，并且**必须有超时兜底**（比如 `max(distance/v, 2s) + 1s`）。
绝不允许"发了就一直走"。

### 12.4 brake 能否真的把车刹住？（待实测，Q17）

目前 `brake` 只是油门写 1400 µs（轻刹），**并不能保证静止**。而 `CUT` 全程约 34 s，
如果车在这 34 s 里缓慢蠕动，刀片会把果梗扯断、甚至把机械臂拽弯。

必须在 S1 做一次实测：

- 平地上 `brake` 后，用激光笔/贴纸标记车轮，观察 30 s 是否有位移
- 若有蠕动 → 需要（a）更强制动值（油门更低）、（b）给一个驱动轮机械止动，
  或（c）在 PICK 期间周期性发 `brake` 并用视觉监控位移

> **这条不解决，S3 的连续采摘不能验收。**

### 12.5 停车精度不够怎么办

- 停车后若 `/arm/check_reachable` 返回 false，**先尝试机械臂的补偿**（移动到窗口边缘）；
- 仍不可达 → 用 `/chassis/creep` 小步进（±30 mm）微调，最多 3 次；
- 3 次仍失败 → 跳过该目标（黑名单 30 s），回巡航。

**"跳过"必须是一等公民。** 卡在一个摘不到的果子上是这类系统最常见的死锁。

### 12.6 巡航时的横向控制（原文档遗漏）

无地图、无里程、无 IMU，`HazardReport` 只提供 `nearest_bearing_rad`。三档方案：

| 档 | 做法 | 代价 | 建议 |
|---|---|---|---|
| **a. 开环直行** | `steer = 0`，靠楼道较直 + 低速 | 会缓慢跑偏，需要人扶 | ✅ **第一版** |
| b. 走廊居中 | 用 Astra 的左右墙距差做 P 控制输出 `steer` | 需要可靠的分左右墙逻辑；交叉口会失效 | S4 可选 |
| c. 视觉循线 | 地面贴胶带 + 下视相机 | 需要第三台相机 | 不做 |

第一版明确接受：**巡航是开环直行，可能跑偏，必要时人工扶正**。
这不是优雅方案，但在"无编码器、无 IMU"的当前硬件上是最诚实的方案。

**并且第一版只处理车体左侧的目标**，不做掉头、不做右侧采摘、不做倒车巡航。
这个边界要写进 S3 验收，否则范围会失控。

---

## 13. 采摘时序（一停多采）

> **⚠️ 前提未确认（Q16）**：下面的 `PICK×N`（一次停车采多个）依赖一个假设——
> **果实剪断后会自己离开套筒**（落进锥形罩的导料通道 → 收集箱），不需要"放手"动作。
>
> - 若成立：`PICK×N` 按下面的时序走，S3 可验收"连续采 3 个"
> - 若不成立（套筒一次只能装一颗）：必须改成 **一采一停**，并增加一个
>   `/arm/release` 或"回到收集口"的步骤，S3 验收标准相应下调
>
> 现有 `pick_and_cut_pipeline.py` 到"后退"就结束了，**没有任何脱手/卸载动作**，
> 所以这条必须先确认，否则 S3 的核心流程是空中楼阁。

```text
SEARCH      巡航中持续收集目标 → 排序进队列（最多保留 5 个）
STOP        目标进入视野 → 立刻停车（§12.3 段 1）→ creep 逼近窗口（段 2）
PICK×N      对队列首个目标：
  ┌ PRE_APPROACH   /arm/move_to_pose 到 Pre-Grasp（果实前方 5–8 cm）
  ├ INSERT         沿进刀轴直进 4–6 cm，套筒包住果实
  ├ HOLD           ← 停在这里，等 /mission/approve_cut 或语音"剪切"
  ├ CUT            /cutter/cut(span=-60°, dwell=0.5) → 合刀 → 保压 → 张刀
  ├ RETREAT        退刀 2–5 cm（先沿进刀轴后退，再抬升）
  ├ 队列非空？ → 直接对下一个目标 PRE_APPROACH（不回 REST）
  └ 队列空   → 回 REST
RESUME      臂在 REST 确认 → 解除刹车 → 继续巡航
```

**四个要点**

1. **"张刀"必须是 `/cutter/cut` 的原子组成部分**，不能由任务层拼接。
   否则一旦任务层崩了，刀片会留在合刀状态。
2. **HOLD 必须可超时**：等 30 s 无确认 → 自动退刀 + 语音提示 + 回巡航（避免卡死）。
3. **只有 `cutter` 的 result 是 `success` 且 `final_angle == home` 才允许 RETREAT。**
   没张开就退刀会刮伤果实/扯断枝条。
4. **每次上电必须先人工把刀片摆到全开，并重测 J6 零点（Q19）。**
   J6 多圈位置不跨上电保存（减速比 50:1，上电读数只在 ±3.6° 内随机），
   且不能顶限位找零位（堵转会触发驱动器复位、再次丢圈）。
   ⇒ `/cutter/cut` 必须有一道**开机闸门**：`config/j6_blade.json` 里记录的
   `measured_at_boot` 与本次开机的 `boot_id` 不一致时，拒绝执行剪切，
   并语音提示"请先摆正刀片并重测零点"。
   **S3 的"一键启动"因此有一个前置人工步骤**，验收标准里要写明。

### 13.1 单次循环时间预算（按当前 J6 速度）

| 步骤 | 预计 |
|---|---|
| 发现 → 停车 → creep 逼近 | 4–8 s |
| PRE_APPROACH | 3–5 s |
| INSERT | 2–3 s |
| HOLD（人工确认） | 不定，典型 3–10 s |
| **CUT（60° @ 3.6°/s，含保压与张刀）** | **约 34 s**（合刀 17 s + 张刀 17 s） |
| RETREAT | 2–3 s |
| **单果合计** | **约 50–65 s** |

> ⚠️ 这是当前的**最大瓶颈**。若固件 `speed` 参数可提高（Q10），把剪切降到 8–10 s，
> 单果可压到 25 s 以内。**这件事的收益远大于任何软件优化。**

---

## 14. L3 任务编排决策：为什么不用行为树

### 14.1 备选方案

| 方案 | 说明 |
|---|---|
| A. 行为树（BehaviorTree.CPP v3/v4 + XML） | 行业标准，可视化，Sequence/Fallback/Retry 天然 |
| B. Python 顺序脚本（阻塞式 `await` / 直接调用） | 最直观，最快出 demo |
| C. **异步技能层 + tick 驱动显式状态机**（选定） | 技能层稳定，编排层可替换 |

### 14.2 决策：选 C

**理由**

1. **行为树的价值要等三个条件成熟才兑现**：多个任务、多人编辑流程、跨任务复用恢复子树。
   本项目当前一个都不满足——只有一个线性主流程 + 少量分支。
   现在引入 BT，等于**立刻支付**（Blackboard、tick 语义、节点注册、XML、
   跨 tick 状态难调试）而**收益延后**，并且可能永远用不上。
2. **纯 Python 顺序脚本的问题不是"顺序"，是"不可中断"**。用 `await` 写出的
   线性流程看似干净，但一旦要支持"任意时刻语音暂停""剪切失败重试""跳过卡死目标"，
   就会长出层层嵌套的 try/except 与全局标志，最后没人敢改。这是反目标 §2 里的第 4 条。
3. **C 把两者的优点拆开了**：
   - **技能层**（`pickorange_skills`）把每个动作封装成"非阻塞三件套"：
     `start()` / `poll()` / `cancel()`，内部是 Action client + 超时 + 错误码。
     它可被状态机、脚本、单元测试三种方式复用。
   - **编排层**（`pickorange_mission`）是一个 10 Hz tick 的状态机，
     **状态 × 事件 → 下一状态** 写成一张显式转移表，跳步由守卫函数拒绝。
     每 tick 只做"检查条件 + 推进子状态"，永不阻塞。
4. **关键性质：编排器可替换。** 因为 L2 的 Action 契约是稳定边界，
   将来若真需要 BT，只需写一层 BT 叶子节点去调同一批 skills，**L1/L2 一行不改**。
   这个性质本身就是"便于加模块"的兑现方式。

### 14.3 状态机设计（tick 驱动，10 Hz）

```text
状态：BOOT → IDLE → CRUISE ⇄ SLOW → STOP → PICK → RESUME → CRUISE
                   ↘ FAULT ↗        ↘ HOLD ↗（PICK 的子状态）
任意状态 → ESTOP
```

**转移表（节选，守卫列即"必须满足才能进入"）**

| 当前 | 事件 | 下一状态 | 守卫 |
|---|---|---|---|
| IDLE | `/mission/start` | CRUISE | 臂在 REST ∧ estop 未激活 ∧ 底盘 watchdog_ok |
| CRUISE | `hazards.suggestion == STOP` | FAULT | — |
| CRUISE | 目标进入减速区 | SLOW | `targets.selected_id != 0` |
| SLOW | `check_reachable == true` ∧ 已进窗口 | STOP | — |
| STOP | 稳定 500 ms | PICK | 臂在 REST |
| PICK | 子状态完成 | PICK（下一个目标） | 队列非空 |
| PICK | 子状态完成 | RESUME | 队列空 |
| PICK/HOLD | 30 s 超时 | RESUME | — |
| RESUME | 臂在 REST | CRUISE | — |
| 任意 | `/safety/estop` | ESTOP | — |
| 任意 | 心跳丢失 | FAULT | — |

**PICK 的子状态机（线性，同样显式）**

```text
PRE_APPROACH → INSERT → HOLD → CUT → RETREAT → (DONE)
                 ↑                 │
                 └── 重试 ≤2 次 ←──┘（仅 CUT 失败且刀片已张开时）
```

### 14.4 什么时候应该改回行为树（演进判据）

出现以下任一条，就应当把 L3 换成 BT（此时技能层已经就绪，改动是局部的）：

- 同时存在 **3 个以上任务**（采摘 / 巡逻 / 回充 / 巡检），且共享大量子流程
- 有 **第二个人** 需要独立修改作业流程而不碰代码
- 需要 **跨任务复用的恢复子树**（如"卡住→尝试脱困→重试原动作"）
- 需要**运行时热加载/远程编辑**作业流程

---

## 15. 安全设计

### 15.1 四级急停链

> ⚠️ **"断电急停"对本机是危险动作（Q18）**：`safe_park_and_exit.sh` 的存在本身就说明
> ——**悬空断电会让机械臂下坠**。24 V 总断电会同时切掉机械臂的保持力矩。
> 所以急停必须分回路，不能一把切总电。
>
> 需要确认的硬件事实：底盘动力回路与机械臂 24 V 是否可独立切断？机械臂驱动器是否有
> 抱闸或独立使能回路？在此之前，`/safety/estop` 的语义定为：**底盘立即停车 + 机械臂 `!STOP` 保持力矩**，
> **不失能**。

| 级别 | 触发 | 行为 | 恢复方式 |
|---|---|---|---|
| L0 物理 | 24 V 断电开关 | 全系统断电 | 人工（**PICK 期间的姿态下会下坠，慎用**） |
| L1 MCU | 500 ms 心跳丢失 / SW 看门狗 | 底盘六路回中位；臂 `!STOP` | 重新发心跳 |
| L2 软件 | `/safety/estop` 服务、语音"停止" | 取消所有 Action + brake + 臂 `!STOP`（保持力矩） | `/safety/reset` + `enable` |
| L3 任务 | 状态守卫拒绝、超时 | 只停当前子动作 | 状态机回退 |

### 15.2 互锁（Interlock）—— 本轮新增，必须有

| 规则 | 实现位置 |
|---|---|
| **底盘动 ⇒ 臂必须在 REST** | `chassis_node` 下发非零油门前置检查 |
| **臂动 ⇒ 底盘必须已 brake** | `mission` 在 PICK 入口强制 brake，`chassis_node` 在 L2 层兜底 |
| **剪切中 ⇒ 底盘绝对静止 + J1~J5 锁死** | `cutter` 执行期间硬件层锁定，拒绝其它运动请求 |
| **Astra 报 STOP ⇒ 禁止任何前进** | `mission` + `chassis_node` 双层 |

### 15.3 心跳链

```
voice_io ─┐
mission ──┼─→ safety_supervisor ─→ chassis_node ─→ STM32H5(500ms) ─→ 硬件
cutter ───┘
```

任一层丢失 → 逐级降级：`supervisor` 取消所有 Action → `chassis_node` brake →
MCU 超时回中位。**四重冗余，任何一层崩了车都会停。**

### 15.4 语音安全约束（最容易翻车的地方）

| 命令 | 允许状态 | 硬限制 |
|---|---|---|
| 剪切 | 仅 `PICK/HOLD` | 单次有效，不排队 |
| 进刀 | 仅 `PICK` | 单步 ≤ 30 mm，累计 ≤ 60 mm |
| 退刀 | 仅 `PICK` | 单步 ≤ 30 mm |
| **前进** | 仅 `CRUISE`/`IDLE` 且臂在 REST | **有限步进 0.2 m**，不是"一直走" |
| **后退** | 同上 | **有限步进 0.2 m** |
| 停止 | 任意 | 立即 |
| 暂停/继续 | 任意 | — |

> **绝对禁止**："开始前进直到我说停"这种开放指令。
> 这是语音控制移动机器人最常见的失控方式。所有移动指令必须是**有限步进 + 超时自动结束**。

---

## 16. 性能预算与实时性

### 16.1 CPU 预算（目标：满载 < 60%）

| 任务 | 核 | 估计占用 |
|---|---|---|
| ros2_control + 硬件插件 | A78C | 5% |
| chassis_node + supervisor | A78C | 3% |
| mission（10 Hz） | A78C | 1% |
| target_detector（YOLO11n-seg @15 Hz） | A55 | 30–40% 单核 |
| astra_guard（深度 ROI @30 Hz） | A55 | 8% |
| KWS（常开） | A55 | 3–8% |
| TTS（间歇） | A55 | 峰值 1 核 |

**剩余算力充足**，不需要为性能做激进优化。真正的瓶颈是 §13.1 的机械剪切时间。

### 16.2 抖动控制（配置项，见 §6.3）

- 控制线程 `chrt -f 80` + 绑 A78C
- 视觉/语音绑 A55
- 演示时 governor 设 `performance`
- 关深度 idle（可选）

### 16.3 固件优化（唯一值得动 RTOS 的地方）

当前 `ventuno_q_chassis_pwm.ino` 在 `loop()` 里用 `delayMicroseconds` **串行**生成 6 路脉冲，
一帧阻塞约 10–13 ms，期间 RPC 回调与看门狗检查都被推迟。

**建议改成硬件定时器 PWM**（Zephyr `pwm_set_cycles`，`pwm3`/`pwm4` 设备已存在）：

- 零抖动、六路真正并行、`loop()` 不再阻塞
- 需先验证 `~D3/D5/D6/D9/D10/D11` 六路能否全部映射到硬件 PWM 通道（Q12 前置实验）
- **若不能全映射，保持现状也可以**——RC 舵机对 ±1 ms 抖动不敏感，现状功能正确，
  这是优化而非缺陷。

---

## 17. 语音子系统

### 17.1 选型（全部离线、全部 CPU、不占 NPU）

| 用途 | 选型 | 大小 | 延迟 | 理由 |
|---|---|---|---|---|
| 常开监听 + 命令词 | **sherpa-onnx KWS**（流式 zipformer，中英） | 30–70 MB | < 100 ms | 6 个命令词，常开低功耗，误触发可控。**主力** |
| 自由语句（S4 可选） | **SenseVoice-small int8** | ~230 MB | 0.3–0.8 s/句 | 中文准确率高、离线、非流式 |
| 固定反馈语 | **预生成 WAV**（约 15 句） | ~2 MB | 0 | 零延迟零 CPU，最稳 |
| 动态 TTS（S4 可选） | **piper `zh_CN-huayan-medium`** 或 sherpa-onnx VITS | 60–160 MB | 实时 | ARM 上 CPU 就够 |

**为什么不用大模型 / 不占 NPU**：命令词表只有 6 条，KWS 的准确率与延迟都优于通用 ASR，
且 CPU 占用极低。把 NPU 留给将来的视觉加速更划算。

### 17.2 两个必须处理的坑

1. **回声自激**：喇叭与麦在同一 USB 声卡上，TTS 输出会被当成命令。
   第一版做**半双工**：播放期间停止 ASR（丢帧而非缓存），简单可靠。
2. **USB 声卡设备号漂移**：和串口一样，必须用 `/dev/snd/by-id/` 或 ALSA 名字绑死，
   不能用 `hw:1,0`。这是同类型坑的第二次出现，值得写进避坑清单。

### 17.3 命令词表（S3 实现范围）

| 命令词 | 意图 | 下游 |
|---|---|---|
| 剪切 / 剪 | 确认剪切 | `/mission/approve_cut` |
| 进刀 | 套筒前探 | `/arm/...` 笛卡尔 +25 mm |
| 退刀 | 套筒后探 | `/arm/...` 笛卡尔 −25 mm |
| 前进 | 小车前进一点 | `/chassis/creep(+0.2 m)` |
| 后退 | 小车后退一点 | `/chassis/creep(−0.2 m)` |
| 停止 / 停 | 软急停 | `/safety/estop` |
| 暂停 | 暂停任务 | `/mission/pause` |
| 继续 | 恢复任务 | `/mission/resume` |

### 17.4 反馈语表（预生成 WAV）

| 时机 | 语句 |
|---|---|
| 发现目标 | "发现目标" |
| 停车完成 | "已停车" |
| 套住完成 | "套住完成，等待剪切指令" |
| 开始剪切 | "开始剪切" |
| 剪切完成 | "剪切完成" |
| 未切断 | "未完全切断，准备重试" |
| 退刀 | "退刀" |
| 采摘成功 | "采摘完成，继续巡航" |
| 跳过目标 | "该目标无法到达，已跳过" |
| 停止 | "已停止" |
| 前进/后退 | "前进中" / "后退中" |
| 故障 | "发生故障，已停车" |

---

## 18. 视觉模型与数据策略

见 §11.4（接入方式）与 §11.5（500 张的评估方法）。补充策略：

### 18.1 训练/部署链路

```
标注 (X-AnyLabeling) → YOLO-seg 数据集 → 训练 (M4 Max, ultralytics)
   → best.pt → 导出 best.onnx（静态 640）→ 板上 onnxruntime CPU
   → （可选）x86 上 QAIRT 量化 → QNN context binary → 板上 QNN EP
```

### 18.2 数据策略

- **机位一致性优先于数量**：同机位 500 张 > 跨机位 2000 张
- 长尾类别（`pedicel` 70 个、`tree` 29 个）当前 Recall = 0，等于没有。
  这两类**不要**在 S2 阶段作为逻辑依赖（例如"必须找到果梗才允许切"），
  否则整条流水线会被一个 Recall=0 的类别卡死。
- 果梗定位的第一版替代方案：**用果实质心 + 相机光轴方向定义进刀轴**，
  果梗识别作为加分项，不作为门禁。

---

## 19. 数据、日志与存储

| 项 | 策略 |
|---|---|
| rosbag2 | 只录 `/perception/targets`、`/robot_state`、`/chassis/state`、`/tf`；**不录原始图像**（除非专门采集） |
| 图像采集 | 单独脚本按需开，采完即导出，不长期占盘 |
| 模型权重 | `dummyarm_AImodel/runs/` 已在 `.gitignore`，只把 ONNX 部署件放 `models/deploy/` |
| 磁盘治理 | 剩余 19 GB；建议加 M.2 NVMe（Q11） |
| 日志 | 每个节点 `rclcpp`/`rclpy` logger；关键状态转移写 `INFO` 并带 `seq` |

---

## 20. 落地里程碑

| 阶段 | 内容 | 验收标准 | 阻塞项 |
|---|---|---|---|
| **S0 地基** | `pickorange_interfaces` + TF 树 + `chassis_node`（§6.5 方案 A）+ `/robot_state` + 顶层 launch + 绑核配置 | 一条命令起来；RViz 显示 车+臂+两相机 TF；`ros2 topic pub /cmd_vel` 车能走；`/chassis/state` 有 watchdog_ok | **Q1 Q2 Q3 Q4 Q4b** |
| **S1 感知** | Astra 驱动 + `astra_guard`；ONNX 导出并接入 `target_detector`；`/perception/targets` + `check_reachable`；planning scene 加车体；brake 保压实测 | 画面出现橘子 → RViz 出 3D 球 + 可采/不可采标记；Astra 报出正确的前方距离；brake 30 s 无位移 | Q5 Q6 Q7 Q8 **Q17 Q20 Q21** |
| **S2 采摘** | `/cutter/cut` Action 打通 ros2_control（**废弃 `cut.py` 子进程路径**）+ 开机闸门；`/arm/move_to_pose`；技能层；单目标闭环 | 手动触发，完成 套→等确认→切→张刀→退刀，全程无阻塞、无超时强杀 | Q19 |
| **S3 编排** | 状态机 + 目标队列 + 巡航-停车-creep-采摘循环 + 语音 KWS/TTS | 楼道挂 3 个橘子，一键启动，连续采 3 个（前置：刀片摆全开 + 测零点），中途可用语音暂停/停止 | Q9 Q10 **Q16** |
| **S4 演进（可选）** | 行为树迁移 / NPU 加速 / 力感知 / VIO | — | — |

### 20.1 S2 的前置硬伤（必须先修）

1. **`cut.py` 与 `ros2_control` 抢串口**：`cut.py` 明确拒绝在 controller_manager 运行时执行，
   而 `pick_and_cut_pipeline.py` 用 `subprocess` 调它 ⇒ **只要 MoveIt 在跑，剪切必然失败**。
   必须把 J6 切削做成 `/cutter/cut`，走 `dummyarm_hardware` 里已有的 `cutter_target_pos_` 钩子
   （该成员目前是裸变量，**没有 ROS 接口**，需要补）。
2. **`orange_detector_node` 是 HSV 阈值，不是训练好的 YOLO 模型**，模型尚未接入系统。
3. **`harvest_fsm_node` 在 ROS 回调里 `time.sleep` + `subprocess`**，必须整体重写。

---

## 21. 风险登记表

| ID | 风险 | 影响 | 概率 | 缓解 |
|---|---|---|---|---|
| R1 | 串口双写（cut.py vs ros2_control） | 剪切完全不可用 | **已确定** | S2 前改 Action（§20.1） |
| R2 | 手眼外参失效 | 定位全错 | **已确定** | S0 重标（§9.4） |
| R3 | Astra Pro Plus 驱动版本不匹配 | 避障不可用 | 高 | 必须用 OrbbecSDK_ROS2 **main 分支（SDK v1.x）**；v2-main 明确"不支持 Astra Pro Plus"，apt 的 `ros-jazzy-orbbec-camera` 是 v2，**装不上** |
| R4 | 无编码器 ⇒ 停车位置不可控 | 目标出窗口 | 高 | 发现即停 + creep 闭环逼近（§12.3） |
| R5 | J6 剪切 34 s，循环太慢 | 演示节奏差 | **已确定** | Q10 提固件速度；否则降低期望 |
| R6 | eMMC 只剩 19 GB | 采集/训练中断 | 高 | 加 NVMe（Q11） |
| R7 | 语音回声自激 | 误触发 | 中 | 半双工 + 命令词阈值 |
| R8 | 500 张模型域偏移 | 检出率低 | 中 | §11.5 先评估，按结论补采 |
| R9 | 无速度反馈，电池压降导致变慢 | 巡航不稳 | 中 | 视觉闭环兜底，不用开环时间 |
| R10 | 转向连杆死点/顶死堵转 | 电机过流 | 低 | 标定时确认满舵余量（§10.2） |
| R11 | 臂在车上，重心高 | 倾覆 | 低 | 底盘加速度限制；臂动时停车 |
| **R12** | **`brake` 不能真正保压，34 s 剪切期间车蠕动** | **扯断果梗 / 扭伤机械臂** | **中高** | Q17 实测；必要时机械止动或周期性重刹 |
| **R13** | **`arduino-router` socket 权限不通** | **S0 第一天卡死** | **已确定** | §6.5 方案 A（加组 + msgpack 客户端） |
| **R14** | **剪断后套筒不自动脱果** | **"一停多采"不成立，S3 范围要缩** | **中** | Q16 确认；不成立则改一采一停 |
| **R15** | **每次上电 J6 零点丢失** | **一键启动不成立** | **已确定** | Q19；`/cutter` 加开机闸门 |
| **R16** | 规划器不知车体存在，解会撞车身 | 实际碰撞 | 中 | Q21：planning scene 加底盘包围盒 |
| **R17** | 训练权重不在板上 | S1 无法起跑 | **已确定** | Q20：补导出 + 传输步骤 |

---

## 22. 待确认问题

> 标记 **P0** 的会阻塞 S0 开工；P1 阻塞 S1；P2 阻塞 S2/S3。
> 每条都写了"不给答案时我采用的默认值"，以便不阻塞讨论。

### P0 — 阻塞 S0（TF 树、底盘接入）

| # | 问题 | 为什么需要 | 默认假设 |
|---|---|---|---|
| **Q1** | 机械臂基座法兰中心，相对**相机立杆顶**的**垂直高度差**是多少？ | 建 `base_link → astra_link`，缺这个 TF 树不闭合 | 假设臂基座比 Astra 低 60 mm（**很可能错，必须实测**） |
| **Q2** | D435i 相对 Astra 的**俯仰角差**（是否水平？向下多少度？） | 决定 `camera_color_optical_frame` 姿态 | 假设与 Astra 同俯仰、无 roll |
| **Q3** | 轴距 `L`、后轮距、前轮距、**车体外形长宽高**（mm） | 自行车模型 + 规划避障包围盒 | 假设轴距 0.45 m（**必须实测**） |
| **Q4** | `base_footprint` 取后轴中心地面投影，是否同意？ | 自行车模型旋转中心 | 同意（不同意请给替代点） |
| **Q4b** | **`arduino-router` 组权限怎么给？**（§6.5 方案 A/B/C） | 不给权限，chassis_node 根本连不上 MCU | 方案 A：`sudo usermod -aG arduino-router arduino`（**需要你执行一次 sudo**） |

### P1 — 阻塞 S1（感知）

| # | 问题 | 为什么需要 | 默认假设 |
|---|---|---|---|
| **Q5** | 楼道橘子**悬挂高度范围**？离墙/离车侧的期望距离？ | 定 `targets` 的 ROI 与停车窗口 | 0.35–0.75 m 高，离车侧 0.3–0.5 m |
| **Q6** | 能否配合做一次**停车窗口扫描标定**（车在场时）？ | §12.2 的窗口只能实测 | 先用占位盒子，S1 末补 |
| **Q7** | 那 500 张图的**采集机位**是否等于现在的侧装 D435i 视角？ | 决定是"先评估"还是"先补采" | 视为**不同机位**，先按 §11.5 评估 |
| **Q8** | Astra Pro Plus 的**固件版本**与是否已有可用驱动？ | R3 的直接依据 | 假设需要源码编译 main 分支 |
| **Q17** | **`brake` 能否让车真正保持静止？** 平地 brake 后 30 s 内车轮有无位移？（需实测） | 34 s 剪切期间车蠕动会扯断果梗 | 假设**不能**保持，必须加止动或周期重刹 |
| **Q20** | `best.pt` / `best.onnx` 在哪里？能否放到板上？ | 板上 `dummyarm_AImodel/runs/` 不存在 | 假设需要你在 M4 上导出后传过来 |
| **Q21** | 车体碰撞包围盒尺寸（长×宽×高），能否接受用简化盒代替 CAD？ | 规划器需要知道车体存在 | 简化盒，边距 +3 cm |

### P2 — 阻塞 S2/S3

| # | 问题 | 为什么需要 | 默认假设 |
|---|---|---|---|
| **Q9** | USB 声卡型号、麦克风数量（单麦/麦阵）、喇叭接在何处（USB 声卡输出 / 板载 speaker） | 决定 ALSA 设备绑定与半双工实现 | 单麦 + USB 声卡输出 |
| **Q10** | J6 剪切速度能否提高？固件 `speed` 参数是硬限制还是可调？ | R5，最大性能瓶颈 | 按 3.6°/s，单果 34 s |
| **Q11** | 是否愿意加 M.2 NVMe SSD（256 GB，约百元）？ | 只剩 19 GB，采集+bag 很快撞墙 | 假设愿意 |
| **Q12** | 是否要做"底盘固件硬件 PWM 化"？（需先验证 6 路能否全映射到硬件 PWM） | §16.3 优化项 | 假设 S0 之后再说 |
| **Q16** | **果实剪断后怎么离开套筒？** 锥形罩有导料通道直通收集箱，还是套筒一次只能装一颗、需要人工取果？ | **决定 `PICK×N`（一停多采）是否成立**，直接决定 S3 验收标准 | 假设**能自动脱手**（现有注释提到"导料收纳通道"，需确认） |
| **Q18** | 底盘动力与机械臂 24 V 是否可**独立切断**？机械臂驱动器有抱闸或独立使能回路吗？ | 急停语义与安全硬件清单；PICK 姿态下断电会下坠 | 假设不可独立切断 ⇒ 急停只停底盘、臂保持力矩 |
| **Q19** | 每次上电必须人工摆刀片 + 重测 J6 零点，这个前置步骤**能否接受**？ | 决定"一键启动"的边界与 `/cutter` 开机闸门 | 接受；S3 验收里写明此前置步骤 |

### 22.1 需要你拍板的架构选择（非阻塞，但影响后续）

| # | 选择 | 我的建议 |
|---|---|---|
| **Q13** | L3 用行为树 or 技能层+状态机 | **技能层 + 状态机**（§14），BT 留作 S4 演进 |
| **Q14** | 新包命名前缀用 `pickorange_*` | 建议采用（机械臂专属包保留 `dummyarm_*`） |
| **Q15** | `docs/architecture.md`（v1）如何处理 | 保留原文 + 顶部加指向本文的说明（已执行） |

---

## 23. 附录

### A. 避坑清单（累积）

1. 串口设备号漂移 → 必须用 `/dev/serial/by-id/*REF*`
2. 串口关闭复位 → `c_cflag &= ~HUPCL`，open 时拉低 DTR
3. MoveIt Jazzy：OMPL 参数要包在 `ompl:` 下；时间参数化插件配在 `response_adapters`
4. `ros2_control` 接口声明必须与插件导出一一对应
5. 实时线程禁止阻塞 `readLine`
6. 步进电机低速齿槽抖动 → 速度缩放保持 1.0（0.60 rad/s 基准）
7. J6 轨迹杂流隔离（`lock_j6_during_trajectory_`）
8. 逆解航向用 `atan2(X, -Y)`（URDF 大臂朝 −Y）
9. 禁用粗劣两连杆近似，用真实 CAD IK
10. 收折用两段式 Lift & Fold + 编码器闭环
11. **新增**：USB 声卡设备号也会漂移 → 用 `/dev/snd/by-id/`
12. **新增**：Astra Pro Plus 只支持 OrbbecSDK v1.x 分支
13. **新增**：底盘串口 `/dev/ttySTM0` 被 `arduino-router` 独占，不能直接开；宿主机没有 `arduino.app_utils` 模块
14. **新增**：J6 零点不跨上电，且不能顶限位找零（堵转触发驱动器复位）

### B. 术语

| 术语 | 含义 |
|---|---|
| 停车窗口 | `base_link` 下"机械臂可达 ∩ 相机可见"的三维区域 |
| 有效转角 δ | 自行车模型里的前轮等效转角 |
| 单一写者 | 每个执行器只有一个进程可发指令 |
| 指令闸门 | L4 对 L2 Action 请求的准入检查 |
| HOLD | 已套住果实、等待剪切确认的稳定状态 |

### C. 参考文档

- `docs/architecture.md` — v1.0 机械臂阶段架构（历史）
- `docs/HANDOFF_交接文档.md` — 机械臂真值与避坑（**必读**）
- `docs/safety_and_incidents.md` — 事故史与硬性安全约束（**必读**）
- `docs/chassis_control_verified.md` — 底盘接线实测（优先于 wiring 文档）
- `docs/chassis_wiring_and_control_spec.md` — 底盘信号规范（历史）
- `docs/M2_spec_j6_blade_home.md` — J6 刀片零点与剪切行程
- `docs/柑橘实例分割模型训练与验证记录.md` — 模型实验台账
- `docs/柑橘数据集处理与清洗记录.md` / `docs/柑橘数据集标注与处理指南_AnyLabeling.md`

---

## 变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-09-08 | 机械臂阶段 3 架构批准（混合架构 C） |
| **v2.0** | **2026-10-30** | 扩展为整机架构：五层分层、进程模型、TF 树、双相机分工、停车策略、一停多采时序、L3 编排决策（不用 BT）、语音选型、安全互锁、实时性预算、里程碑 S0–S4、待确认问题 |
| **v2.0.1** | **2026-10-30** | builder-check 修订：① 停车策略由"TTC 分级减速"改为"发现即停 + creep 视觉闭环"（几何核算证明远距离预判不可行）；② 新增 §6.5 底盘 RouterBridge 接入方案（实测 socket/权限/模块均不通）；③ 关节环频率 100 Hz → 20/50 Hz（串口带宽限制）；④ 新增巡航横向控制边界（第一版开环直行、只采左侧）；⑤ 新增 Q16–Q21（脱果方式、brake 保压、急停回路、J6 上电闸门、权重文件、车体碰撞体） |
