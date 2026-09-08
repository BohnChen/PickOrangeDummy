# PickOrangeDummy · 架构决策文档（v1.0）

> 状态：已批准（2026-09-08）。审阅人：陈泌丞。
> 变更记录：v0.1 讨论稿 → v1.0 批准。关键决定见 §11。

---

## 1. 范围与目标

本阶段（阶段 3）只做「真机可用的 ROS2 控制与可视化」：

- 机械臂以 ROS2 标准接口暴露：`/joint_states`、轨迹执行、系统管理服务
- RViz2 实时显示真机状态（M1 数字样机 → 真机联动）
- 为后续（MoveIt 规划 / 视觉 / 旋切任务层 / 移动底盘）预留挂载点

**明确不做**（后续里程碑）：MoveIt 规划、视觉伺服、J6 旋切任务流、底盘接入。

## 2. 硬件与链路基线（已实测）

- VENTUNO Q（Ubuntu 24.04 / Jazzy / arm64）＝ ROS2 主机 + 控制机
- 机械臂 REF 主控（STM32F405）经 `/dev/ttyACM1` USB-CDC @115200
- ASCII 协议（`dummyarm_link` 库已封装，阶段 1 自测通过）
- 末端：J6 上仅空谐波减速器壳；自研旋切末端执行器后续装入
- 6 关节软件限位/减速比/FK 参数已从固件提取（M0 文档后续成稿）

## 3. 架构方案对比

| 维度 | A 桥接节点（Python） | B ros2_control（C++） | C 混合（推荐） |
|---|---|---|---|
| 实现速度 | 快（~1 天可联动 RViz） | 慢（串口异步/生命周期适配） | A 先行见效，B 后置工程化 |
| 与 MoveIt 2 对接 | 需自写 follow 桥 | 原生（trajectory controller） | 由 B 承担 |
| 面试/工程价值 | 一般 | 高（标准硬件抽象） | 最高 |
| 系统管理（急停/回零） | 直接 Service | ros2_control 不原生提供 | 管理节点承担 |
| 串口实时性 | 无硬实时约束，够用 | 需独立读写线程+快照 | 同 B |
| 代码复用 | 直接调 `dummyarm_link` | 可调 `dummyarm_link` | 共用 `dummyarm_link` |

**推荐 C（混合），分两步落地：**
- **阶段 3a（桥接）**：Python 节点发布 `/joint_states`，提供管理服务，RViz 真机同步——先让链路在 ROS2 里「看得见、管得住」。
- **阶段 3b（工程化）**：C++ `SystemInterface` 硬件插件替换桥接的关节通道（读写复用 `dummyarm_link` 思路，串口读线程 + 快照），轨迹走 `joint_trajectory_controller`，为 MoveIt 铺路；系统管理职责留在管理节点。

## 4. 节点图（目标架构，阶段 3b）

```text
                ┌─────────────────────────────────────────────┐
                │            dummyarm_bringup (launch)        │
                └─────────────────────────────────────────────┘
  /joint_states ◄── joint_state_broadcaster ◄──┐
                                                 │ read()   ┌──────────────────────┐
  RViz2 / robot_state_publisher ◄── TF ◄─────────┼──────────┤ dummyarm_hardware     │
                                                 │          │ (SystemInterface C++) │
  joint_trajectory_controller ◄── position ──────┤ write()  │   └─ 串口读写线程 ──►  │
                                                 │          │   /dev/ttyACM1 (REF)  │
                                                 └──────────┤                      │
  dummyarm_system（管理节点, Python/C++）                   └──────────────────────┘
    ├─ std_srvs/Trigger: start / disable / estop / home / reset
    ├─ 心跳与断连检测（读超时 → 状态 UNKNOWN → 自动 !STOP）
    └─（预留）后续高级动作 Action 与旋切任务
```

**说明**：`estop` 语义 = REF `!STOP`（锁位+清 FIFO+失能）＋管理节点状态迁移；
运动使能/回零不做进 ros2_control（它是轨迹层，不是安全/业务层）。

## 5. 接口清单（草案）

| 接口 | 类型 | 内容 | 提供方 |
|---|---|---|---|
| `/joint_states` | Topic | 6 关节角（rad），~100Hz | joint_state_broadcaster |
| `/joint_trajectory_controller/follow_joint_trajectory` | Action(标准) | 关节轨迹执行 | 阶段 3b |
| `/dummyarm_system/start` `disable` `estop` `home` `reset` | Service | `std_srvs/Trigger`（标准，不自定义） | 管理节点 |
| `diagnostic`（预留） | Topic | 链路/固件诊断 | 阶段 3b |
| `/cmd_vel`、感知话题 | — | 阶段 4 预留，不在 3 实现 | — |

命名约定：关节一律 `Joint1..Joint6`（大写，对齐既有 gitee URDF 资产与 MoveIt 教程风格；
固件 CAN 节点号一致 1..6，映射表放 M0 文档），URDF/控制器/代码统一。

## 6. TF 树与坐标系

```text
world ── base_link ── Joint1_link ── … ── Joint6_link ── tool0 ── tcp
                            └── camera_link（阶段4 手眼标定后启用）
```

- REP-103：米/弧度/右手系；`base_link` 原点＝机械臂基座法兰中心投影点（M0 测定）
- `tcp` 待自研末端执行器 CAD 定稿后给出偏移；当前仅 `tool0`
- 零位语义：URDF 零位＝REF 标定零位（`!HOME` 后各关节角为 0 的构型）→ 需要一次
  L-pose 标定对齐（M0/M1 逐轴校准子任务），真机/仿真共用同一模型

## 7. J6 双角色处理（关节 vs 旋切执行器）

- URDF/控制层：`Joint6` 始终是普通 revolute（±720°）
- 任务层语义拆分（未来旋切任务包内表达，不污染控制层）：
  - 定位模式：J6 参与腕姿 IK（现状）
  - 旋切模式：J6 独立多圈/定角运动 + 力矩上限（需先实测驱动板 current limit 行为）
- 现阶段动作演示只走 `Joint6` 角度，不做旋切专用流

## 8. 安全设计（阶段 3 验收必须项）

> ⚠️ 先读 `docs/safety_and_incidents.md`（J3 损坏史 + 硬性约束，任何代码/会话必守）。

1. 上电（进程启动）**不**自动使能、不自动运动
2. `estop` 服务 + 键盘/脚本一键 `!STOP` 路径
3. 串口读超时/断连 → 管理节点报 UNKNOWN 并发 `!STOP`，拒绝新运动指令
4. 所有写命令经 `dummyarm_link` 限位检查（软件限位双保险）
5. 演示/验收用低速（默认 speed≤10）+ 单轴微动起步
6. 与固件 `!STOP`、24V 物理断电形成三级急停

## 9. 里程碑与验收

| 步骤 | 内容 | 验收标准 |
|---|---|---|
| 3a.1 | workspace + description 包（URDF 修正版） | RViz 无 TF 报错，滑块驱动与真机同向 |
| 3a.2 | bridge 管理节点 + joint_states | RViz 实时跟随真机（同步演示） |
| 3b.1 | hardware_interface C++ 插件 | `ros2 control list_hardware_interfaces` 正常 |
| 3b.2 | joint_state_broadcaster + trajectory controller | 服务端轨迹真机执行（低速小角度） |
| 3b.3 | 管理节点整合 + estop 演练 | 断连自动停机/estop 全链路 |
| 3b.4 | rosbag + 文档 + 演示录制 | 交付记录 |

## 10. 待你拍板的开放项

1. ✅ 关节命名统一：采用 `Joint1..Joint6`（大写，与既有 URDF 资产一致）
2. ✅ 混合架构 C（桥接先行 3a，ros2_control 后置 3b）
3. ✅ 系统管理服务用标准 `std_srvs/Trigger` 起步
4. ⏳ URDF 模型几何源：桌面 DH 参数图由视觉模型读取后转交（进行中），
   与固件 DH 表（已验证）交叉比对后定稿
5. ✅ workspace git init（本地已建；GitHub 远端待仓库名确认后推送）

## 11. 已批准决定汇总（v1.0）

| # | 决定 | 备注 |
|---|---|---|
| D1 | 混合架构 C：3a Python 桥接 → 3b C++ SystemInterface | §3 |
| D2 | 关节命名 `Joint1..Joint6` | 与固件节点号 1:1 |
| D3 | 管理服务用 `std_srvs/Trigger`（start/disable/estop/home/reset） | §5 |
| D4 | 自定义接口延迟到任务层需要时再引入 | 蓝图约束 |
| D5 | URDF 几何：固件 DH + 图纸 DH 交叉验证 | §10-4 |
| D6 | workspace 根：`~/PickOrangeDummy`，git 管理 | |
