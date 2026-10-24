# DummyArm 项目 · 会话交接文档（新会话先读我）

> 生成：2026-09-14（最新实测验收版本）。目的：让新会话在最短时间内恢复上下文、知道真值表与控制架构、
> 能直接续接主线，避免重踩已知坑。主人：陈泌丞（BohnChen）。
> 工作约定：中文、简洁、直接；任何动臂动作前先讲风险并获同意。

---

## 0. 一句话当前状态

1. **MoveIt 2 真机控制闭环已完全打通并实测通过**：
   - 仿真与真实机械臂（Qualcomm Monaco Monza / VENTUNO Q aarch64 + STM32F405 REF 单片机）全闭环运行通过；
   - 实现了在 RViz 中 6-DOF 全空间大范围极致灵巧交互拖拽、OMPL 规划与平滑真实物理执行；
   - 在空间位姿调整运动中，第 6 轴（J6 旋切刀具轴）被硬件驱动层有效隔离锁定，绝对零空转，仅在需要切果时通过专门工具独立驱动；
   - 运动速度健康流畅（基准速度 0.60 rad/s，消除微步抖动区间），手腕 J4/J5/J6 与各轴动作连贯平稳。
2. **安全停机与一键收纳闭环已就绪**：
   - `./scripts/safe_park_and_exit.sh` 实现智能双模自适应，确保物理各轴 100% 确认折叠到位后才断电失能，彻底杜绝悬空断电下坠。

---

## 1. 环境与工具（VENTUNO Q 开发板）

- **系统**：Ubuntu 24.04 noble / ROS 2 Jazzy (aarch64)；
- **工作区路径**：`~/01_MyProject/PickOrangeDummy`（git 仓库，GitHub 远端 `BohnChen/PickOrangeDummy` 已全量同步）；
- **核心包清单**：
  1. `dummyarm_cad_description`：CAD 导出高质量高精度红白外观网格模型（带完整碰撞模型）；
  2. `dummyarm_description`：原版真值链模型；
  3. `dummyarm_hardware`：C++ 实现的 ros2_control SystemInterface 插件（负责串口通信、角度映射、安全限位拦截）；
  4. `dummyarm_moveit_config`：完整 MoveIt 2 配置包（含 SRDF、规划管道、控制器、launch、rviz 配置）；
  5. `dummyarm_bridge`：原厂基础桥接管理服务包；
  6. `dummyarm_bringup`：整机顶层控制启动包。
- **真机硬件串口**：**必须直接使用永久硬件软链接**：
  `/dev/serial/by-id/usb-Robot_Embedded_Framework_REF_1.0_CDC_Interface_365D357B3334-if00`
  （彻底免疫 USB 插拔后在 `ACM0`、`ACM1`、`ACM2` 之间的动态漂移）。

---

## 2. 机械臂物理真值与核心映射

- **构型**：Dummy V2 6-DOF 铝合金 CNC 复刻版；
- **固件**：auk0217 ASCII 协议固件（115200 8N1，响应 `!`、`#`、`>`、`@`）；
- **末端设计**：J6 为独立旋转动力轴，驱动自研“套果-旋切”采摘末端执行器。

### 核心物理角 ↔ CAD 模型映射表（真机实测定案）

| 关节 | 固件原始角 ($q_{\text{fw}}$, 度) | CAD 模型角 ($q_{\text{cad}}$, 弧度) | 运动学与物理朝向事实 |
|---|---|---|---|
| **Joint1** | $q_{\text{fw}}[0]$ | $q_{\text{cad}}[0] = q_{\text{fw}}[0]$ | 1:1 直通。底座正前方由电机硬件零位决定 |
| **Joint2** | $q_{\text{fw}}[1]$ | $q_{\text{cad}}[1] = q_{\text{fw}}[1] + 73.0^\circ$ | 物理 REST(-73°) 对应 CAD 0°，物理立姿(0°) 对应 CAD 73° |
| **Joint3** | $q_{\text{fw}}[2]$ | $q_{\text{cad}}[2] = q_{\text{fw}}[2] - 180.0^\circ$ | 物理 REST(180°) 对应 CAD 0°，物理立姿(90°) 对应 CAD -90° |
| **Joint4** | $q_{\text{fw}}[3]$ | $q_{\text{cad}}[3] = q_{\text{fw}}[3]$ | 1:1 同向，从后向前看均为逆时针 |
| **Joint5** | $q_{\text{fw}}[4]$ | **$q_{\text{cad}}[4] = -q_{\text{fw}}[4]$** | **实测镜像取反！** 固件增加为低头，模型增加为抬头 |
| **Joint6** | $q_{\text{fw}}[5]$ | $q_{\text{cad}}[5] = q_{\text{fw}}[5]$ | 1:1 同向。轨迹运动期间驱动层锁定隔离，仅切削时转动 |

### 安全硬限位红线（驱动层逐帧硬拦截，违规立即丢弃）
- **J3 物理防断臂红线**：固件角度严格拦截 $< 35^\circ$（模型角度严格拦截 $< -145^\circ$），**历史机械断裂红线，绝不可逾越**；
- **J5 防壳体碰撞红线**：物理角度严格拦截超过 $\pm 95^\circ$（超过 105° 壳体机械硬干涉碰撞）。

---

## 3. 标准操作命令速查

### 3.1 启动真机 MoveIt 控制系统
```bash
cd ~/01_MyProject/PickOrangeDummy
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dummyarm_moveit_config real_moveit.launch.py
```
> **启动特性**：
> - 串口自动锁定，`spawner` 激活控制器，驱动自动发送 `!START` 闭环锁轴就绪；
> - RViz 自动加载，速度/加速度滑动条默认自动锁定在 `1.0` 正常平稳速度；
> - 交互小球可全向任意大范围自由拖拽，点击 `Plan & Execute` 机械臂平稳受控到达目标，J6 全程静止锁定。

### 3.2 独立果梗切削命令
```bash
./scripts/cut.py
```
> 单独驱动第 6 轴（J6）：刀片全开 `0°` → 合刀 `-60°` → 保压 → 回位 `0°`。
> 行程限制在剪切层（`[-60°, 0°]`），J1~J5 锁死不动，靠 `#GETJPOS` 闭环确认到位。
> 装刀后 **禁止** 使用已退役的整圈旋转脚本（`j6_spin.py` 默认拒绝执行）。
> 详见 `docs/M2_spec_j6_blade_home.md`。

### 3.3 优雅回位收纳并失能退出
```bash
./scripts/safe_park_and_exit.sh
```
> 智能双模停机：若 ROS 2 正在运行，通过 Action 驱动平稳回 REST，**100% 确认到位后**自动发送 `!STOP` / `!DISABLE` 释放力矩并清理进程，绝不悬空断电。

### 3.4 纯硬件底层零位标定（若底座物理对齐微调）
```bash
# 在机械臂断电失能状态下，亲手把底座摆正，然后运行：
./scripts/calibrate_j1.py
```
> 向电机发送原厂 `#OFFSET_J 1` 命令，将当前物理位置烧录固化为单片机电机的绝对硬件零位。

---

## 4. 架构避坑指南速查（不要再踩）

1. **串口设备号漂移**：
   开发板本身带有串口，机械臂拔插后可能分配为 `ACM0`、`ACM1`、`ACM2`。**严禁在代码里使用 `os.path.realpath` 解析出死板的绝对节点**，必须直传 `/dev/serial/by-id/*REF*` 软链接路径，由内核动态重定向。
2. **串口关闭复位问题**：
   Linux 串口必须显式清除 `tty.c_cflag &= ~HUPCL;` 并在 open 时拉低 DTR，否则进程关闭串口或退出时，内核拉动 DTR 信号会触发单片机硬件冷复位（表现为驱动板单灯闪烁重启）。
3. **MoveIt 2 在 ROS 2 Jazzy 中的参数命名空间**：
   - OMPL 参数字典必须包裹在 `{'ompl': ompl_planning_yaml}` 命名空间下，否则报红字 `NO PLANNING LIBRARY LOADED`；
   - 时间参数化插件必须配置在 **`response_adapters`** 中（而不是 request_adapters），否则轨迹时间戳全为 0，控制器报 `Time between points is not strictly increasing` 拒收。
4. **硬件接口声明严格匹配**：
   URDF 中 `<ros2_control>` 声明的接口必须与 C++ 插件导出的接口一一对应。不能声明虚假的 `velocity` 命令接口或 `effort` 状态接口，否则 `resource_manager` 会直接判定硬件初始化失败并挂起 controller_manager。
5. **控制器实时线程严禁阻塞**：
   在 C++ 硬件驱动的 `send_commands()` 中**绝不允许调用阻塞式 `readLine`**，必须是纯异步写（<0.05ms）。否则会饿死非实时的 ROS 2 Action Server 事件循环，导致动作到位后迟迟不汇报 Success、耗尽 19s 超时强杀、随后无法进行下一次动作。
6. **低速齿槽抖动与速度配置**：
   步进电机在低于 2°/s 时会掉入微步齿槽力矩死区产生电磁抖动。不要在 RViz 中将速度/加速度缩放设为 0.1 极限低速，正常运行保持默认 **`1.0`**（匹配底层 0.60 rad/s 安全基准速度），转动惯量自然平滑运动。
7. **J6 轨迹杂流隔离**：
   6-DOF 逆解为了满足末端姿态经常算出多圈角度，在驱动层 `send_commands()` 中拦截 J6 变化量（`target_pos = cutter_target_pos_`），实现上位机大范围灵巧逆解、物理 J6 静止锁定的完美解耦。
8. **视觉反投影与机械臂底座坐标轴定义**：
   Dummy 机械臂 CAD URDF 中，大臂自然舒展朝向为 **`-Y` 轴**！计算对准航向角时应使用 `atan2(X, -Y)`，严禁直接套用标准 `+X` 轴的 `atan2(Y, X)`，否则底座 J1 会对称偏转 90°。
9. **CAD 严格闭式逆解与真实触碰**：
   在目标附近规划或触碰时，严禁使用粗糙的两连杆经验近似（会导致 7~8cm 悬空和几何失真），必须直接调用真实 CAD 运动学链严格优化逆解（残差 < 0.1 毫米），确保 RViz 空间末端与真实物理小球 100% 贴死！
10. **两段式安全收折（Lift & Fold）**：
   从前探位收回 REST 必须采用“先抬臂收肘回正、再平稳折叠收拢”两段式轨迹，并做编码器闭环到位判定，杜绝盲目 sleep 导致未到位截断停在半路。

---

## 5. 视觉闭环与高精轻点定案 (2026-09-15 重大突破定案)

1. **视觉环境与驱动就绪**：
   - 官方 RealSense 驱动包 `ros-jazzy-realsense2-camera` (4.58.4) 与 `ros-jazzy-librealsense2` (2.58.4) 实测稳定工作；
   - RealSense D435i 深度相机硬件链路正常（USB 3.2 握手，640x480 @ 30FPS RGB + 深度点云稳定输出）。
2. **感知功能包 `dummyarm_perception`**：
   - `orange_detector_node`：升级为一阶图像矩（Image Moments）物理质心计算，抗反光偏心；采用轮廓自适应掩码双边中位数深度滤波，消灭空洞跳变；
   - 流水线引入 1/2 空间下采样优化与零内存拷贝，调试图像发布开销暴降 75%，`rqt_image_view` 恢复 25~30 FPS 丝滑帧率；
   - 输出标准话题：`/dummyarm/target_orange_pose` (PoseStamped) 与 `/dummyarm/target_orange_marker` (Marker)。
3. **高精度手眼外参终极定案（SVD 4 点实测）**：
   - 相机物理摆放为**机械臂右前方高位俯视**，与现实物理直觉 100% 契合；
   - 终极实测定案参数写入 `perception_bringup.launch.py`：
     - `cam_x = -0.1959`
     - `cam_y = -0.2986`
     - `cam_z = 0.4678` (高度正 46.8cm)
     - `cam_roll = 0.5252`, `cam_pitch = 1.2661`, `cam_yaw = 0.4923`
   - 闭环自检验证：机械臂末端 tool0 与相机反投影目标在基座系重合误差仅 **0.1cm (1.0 毫米)**！
4. **运动学与仿生轻触执行器 `instant_tap.py`**：
   - 彻底植入机械臂真实 CAD URDF 正反向运动学链优化逆解器（残差 < 0.1 毫米）；
   - 采用非阻塞 Direct Trajectory Stream 架构，绝无 Action 超时挂起，退出 100% 优雅释放真机控制权给 RViz；
   - 动作流程：阶段 1 直达目标正前方 2.5cm 处定住展示 1 秒 -> 阶段 2 沿进刀轴向前顶进 2.5cm 轻碰物体表面 3 次 (Tap x3) -> 阶段 3 自动两段式平稳收折回到 REST 原点。实测定位与触达精度极佳！

---

## 6. 下一步推进建议

1. **接入机械旋切采摘末端闭环**：
   - 在 `instant_tap.py` 精准触碰验证的基础上，将阶段 2 的轻碰替换为自研双层锥形外罩进给套入；
   - 触发 `./scripts/cut.py` 驱动 J6 由 `0°` 合刀到 `-60°` 再回位完成果梗剪切作业（行程见 `docs/M2_spec_j6_blade_home.md`）！
2. **采摘后视觉脱落校验与多目标连续轮巡**：
   - 将状态机接入后撤视场抽检，若柑橘脱落则计数累加并寻找下一目标，若未脱落则执行补偿重试。

---

## 7. 2026-10-24 现场记录（装刀后）

1. **J6 刀片零位已固化**：以刀片最大打开姿态为 J6 零点，`#OFFSET_J 6` 写入电机 EEPROM。
   零位前读数 `-1.22°`，零位后 `-0.20°`，回位后 `+0.01°`；规范见 `docs/M2_spec_j6_blade_home.md`。
   - 语义：`J6 = 0°` 刀片全开（剪切起始），`J6 = -60°` 合刀到底。
   - **以后任何全轴重标定（`#CALIBRATION`）都会覆盖该零位，必须重新执行 `scripts/zero_j6.py`。**
2. **J5 存在装配预应力**：末端执行器装配状态下 J5 被轻微压弯（当时读数 `J5 = 8.36°`），
   未触及 `±95°` 限位红线。后续动作前需确认 J5 无卡滞、无异常电流噪声；
   若压弯加剧，应拆检末端执行器与 J5 输出轴的同心度。
