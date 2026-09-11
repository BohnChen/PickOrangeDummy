# DummyArm 项目 · 会话交接文档（新会话先读我）

> 生成：2026-09-09（本会话结束）。目的：让新会话在最短时间内恢复上下文、知道真值表、
> 能直接续接主线，避免重踩已知坑。主人：陈泌丞（BohnChen）。
> 工作约定：中文、简洁、直接；任何动臂动作前先讲风险并获同意。

---

## 0. 一句话状态

机械臂**运动学/控制链已全部打通并实测**（ros2_control + 桥接 + 映射）；
**视觉 CAD 装配已完成**：补丁版导出器直接产出可用包，项目内 `dummyarm_cad_description`
（2026-09-10 直接采用 Fusion 导出：坐标/限位/滑块+RViz 全验证；关节值=CAD 约定、设计姿态=0）。
2026-09-10 另按 DH 修正真值链几何 J3/J4，J3 零位约定随之平移 90°，TH[2]: 90→0。
Fusion→URDF 导出全流程（含三个坑与设计规则）见 **`docs/fusion_export_workflow.md`**。
剩余：CAD 包并入 `dummyarm_description`（可选，改名 Joint1..6/link_1..6/tool0 + 换 ros2_control）；
真机只读核对新映射（TH[2]: 90→0）。

## 1. 环境与工具（都在这台 VENTUNO Q 上）

- Ubuntu 24.04 / ROS2 Jazzy；源码工作区：`~/01_MyProject/PickOrangeDummy`（2026-09-10 从 `~/PickOrangeDummy` 移入；git, GitHub 远端 `BohnChen/PickOrangeDummy`，已推）
- 5 个包：`dummyarm_description`（真值链）`dummyarm_cad_description`（Fusion CAD 视觉包）`dummyarm_hardware`(C++ SystemInterface) `dummyarm_bridge`(管理服务) `dummyarm_bringup`
- 构建：`scripts/build.sh`（记得它补 lib symlink；仓库根 = 本目录）
- 真机串口：`/dev/ttyACM1`（Type-C 控制面 = REF 板 USB-CDC；另一面是 ESP32）
- OCC venv：`~/occvenv/bin/python`（已装 cadquery-ocp=OCP）；pip 源已切清华镜像
- 独立库：`~/dummyarm_tools/dummyarm_link`（ASCII 协议 Python 库+`dummyarm` CLI，J5 限位已改 ±95）
- 关键文档：`docs/`：`architecture.md`（v1.0 批准）`safety_and_incidents.md`（事故史+硬约束）`M0_spec.md`（规格冻结）`fusion_export_workflow.md`（Fusion 导出流程）

## 2. 机械臂事实（实测/图纸真值，勿再推翻）

- 结构：Dummy V2 国产复刻（auk/晓文链），CNC 铝件"视频款/改版"（J1/J23/J4/J5A/J5B/J6+基座）
- 固件：**auk0217 系**（reademe.txt: 发货前刷 auk0217）；FK 实测与 auk 参数一致
- 末端：无夹爪；J6 空减速器壳；未来 = 自研"套果-旋切"执行器（J6 驱动）
- 用户实测：桌面→J1 底座开孔高 ≈92.6mm（图纸 55 基座 + 37.5 J1件 = 92.5）
- J5 物理：**±95°**（105° 必碰结构）→ 全链已按 ±95 收紧
- 接线安全：REF↔J1 反向线（错了烧 CAN 全家）；供电建议 12V

### 关节真值表（度；模型轴：J1=Z, J2/J3/J5=−Y, J4/J6=+X）

| 关节 | 减速比 | 限位 | 备注 |
|---|---|---|---|
| J1 | 30 | ±170 | 无硬停，线束约束 |
| J2 | 30 | −73…90 | 下限=安全标定 |
| J3 | 30 | 35…180 | **<35 历史崩坏红线** |
| J4 | 24 | ±180 | |
| J5 | 30 | **±95** | 105° 碰结构 |
| J6 | 50 | ±720 | 多圈(旋切动力) |

### 几何/映射（关键常数）
- 图纸真值链参数(m)：L_BASE 0.0925(实测92.6) D_BASE 0.035 L_ARM 0.146 D_ELBOW 0.052(图50) L_FA 0.115 腕=公共点 tool0=+0.072 沿 J6 轴
- FW→模型：`q_model = SIGNS·(q_fw + TH_HOME)`，SIGNS=[1,-1,-1,1,1,1]，TH=[0,-90,0,0,0,0]
  （2026-09-10 修正 J3 零位约定；物理 REST (0,-73,180) ↔ 模型 (0,163,-180)）
- 关系：p_fw ≈ p_model + (60.85, 0, 171.14)mm（诊断用）
- 固件 DH(0.109…)与物理(92.5)差 ~16mm = 预期（固件近似）
- REST=(0,-73,180,0,0,0) 休憩位；!HOME→(0,0,90,0,0,0)（7 字姿态）
- **操作规定：动作前先回 !HOME（7 字）再执行；结束回 REST + 失能**（主人规定 2026-09-10）
- 桥接读模型角实测 [0,163,−180,0,0,0]=REST（2026-09-10 约定修正后；旧记 −270）

## 3. 运行命令（速查）

```bash
# RViz 纯模型（滑块演示, 不连臂）
source /opt/ros/jazzy/setup.bash && source install/setup.bash   # 在仓库根执行
ros2 launch dummyarm_description display.launch.py
# 真机只读控制链 (不使能)
ros2 launch dummyarm_bringup control.launch.py          # spawn_trajectory:=true 才加载轨迹控制器
# 管理服务(单独进程, 与上面串口互斥, 别同开!)
ros2 run dummyarm_bridge dummyarm_system               # 服务: /dummyarm_system/{start,disable,stop,home,reset}
# ASCII 直控
dummyarm query   # 使能: dummyarm start; 动: >j1..j6,speed
```

## 4. 视觉装配问题的完整来龙去脉（新会话必读，省 3 小时）

> **2026-09-10 更新（结论已闭环）**：实际导出器为 **Adriaeik/fusion2URDF v3.1.0**（非 dheena2k2）。
> 根因 = `core/robot_model.py::_compute_joint_global_origin` 对根组件关节 **double-lift**
> （把已是世界坐标的 `geometryOrOrigin*` 又叠加了一次组件变换）→ 米级帧距。
> 修复 = 17 行补丁（存 `~/dummyurdf/fusion2URDF-root-joint-fix.patch`）+ `tools/reframe.py --snapshot`
> 离线重生成 → 关节帧与 CAD 对齐（轴残差 <1.6mm）。
> CAD↔模型映射已解出并应用：`q_model = [+1,-1,-1,+1,+1,+1]·q_cad + (0,164.95°,-179.95°,0,0,0)`，
> 限位按模型约定（J2 [0,163°]、J3 [-180°,-35°]），RViz 验证通过。
> 另：真值链几何按 DH 修正（Joint3/Joint4，见 M0_spec §3），J3 零位约定平移 90°。
> 以下为当时的分析记录（部分结论已被上面的根因取代，保留作历史）。

症状：Fusion 导出 URDF（dheena2k2/fusion2urdf-ros2 工具）在 RViz 中坐标系/部件米级错位、散开。
结论（已查明）：
1. 工具"Translations are intentionally unsupported"：**Fusion 联结若带任何平移分量会被原样写成 joint origin**（本例 0.5~1.1m）→ 帧距 1.14m。
2. 设计原点不在基座（Fusion 装配原点偏离 (−55cm,0,11cm) 级）→ 烘焙进帧。
3. 工具只修旋转（frame_overrides.csv/reframe.py），平移不可救。
4. 导出 debug JSON（frame_model/fusion_transforms）**位置数据同样被污染**（source_world_rotation 残差巨大），不能直接反解。
5. STEP 整机（`~/Dummy.step`，Fusion 导出 23MB，OCC 读入 OK）为 compound+Location 实例式（无 XCAF 结构、无 per-part occurrence 分组、顶层 6 组各横跨 1-2m、无 scale），**不能靠简单遍历还原主件位姿**——需专门调试（暂定 TODO：换成熟 pythonocc 装配遍历或 Fusion 归零原点重导）。
6. 因此：**视觉最终解的两条路**：
   - A（推荐）：Fusion 重导：①根组件命名 `base_link`+取消接地 ②每 link 独立组件只含实体 ③联结=选**两圆孔轴线**的纯旋转、不拖平移 ④装配原点放基座中心 ⑤导出时 post_* 清 0。→ URDF 到达即可 10 分钟装库。
   - B：STEP 专项（pythonocc 装配遍历，时间不可控）。
- 当前过渡视觉：真值链 + 打印件 STL（PCA 启发式挂载、弧度 bug 已修），`display.launch.py` 可演示（件有平移/朝向误差，后续用 CAD 替换）

## 5. 下一步建议（新会话按序）

1. **真机只读核对新映射**：`ros2 run dummyarm_bridge dummyarm_system` + `ros2 topic echo /joint_states --once`
   → 期望 REST = `(0, 2.844887, -3.141593, 0,0,0)`（模型 −180°，不是旧的 −270°）。全程失能、无运动。
2. 【可选】CAD 包并入 `dummyarm_description`：命名统一 Joint1..6/link_1..6/tool0、
   限位按 M0 表、J6 revolute±720、ros2_control 换 dummyarm_hardware（思路见 `scripts/import_export_urdf.py`）
3. 真机同步验收 `control.launch.py`(include_bridge:=false 单开) + 低速单轴动作（需用户在场；大动作先报风险）
4. 末端执行器（套果-旋切）+ 视觉；收尾文档/演示视频/公开仓库（面试用）

## 6. 已知坑速查（不要再踩）

- `pkill -f` 会匹配到自己的命令行 → 用 `[x]` 技巧或按 PID
- launch 里多行 robot_description 必须 `ParameterValue(..., value_type=str)`
- xacro 宏调用写法 `<xacro:name .../>`，不是 `<xacro:call name .../>`（会 XML 错）
- colcon ament_python 可执行文件在 bin/，ros2 run 只找 lib/<pkg>/ → build.sh 补 symlink
- 打开串口必须 DTR 拉低（否则复位 REF）
- 不要同时开 bridge 与 ros2_control 占同一串口
- 双发 /joint_states 会造成 RViz"画面跳"（残留进程清理）
- /tmp 会被清（kdlcheck.cpp 等工具要重编或入库）
- 本机 GUI 会话里别用我后台起的 GUI 进程占显示
- **ROS 2 Jazzy MoveIt 2 规划库未加载（NO PLANNING LIBRARY LOADED）**：OMPL 配置文件内容在传给 `move_group` 和 `rviz2` 节点参数时，必须嵌套在 `{'ompl': ompl_planning_yaml}` 命名空间下，否则因缺少 `ompl.` 前缀被识别为空规划器。
- **ROS 2 Jazzy MoveIt 2 轨迹时间戳全为0被控制器拒收（Time between points 0 and 1 is not strictly increasing）**：`AddTimeOptimalParameterization` 插件在 Jazzy 中必须配置在 `response_adapters`（规划后适配器）中，不能放在 `request_adapters`，否则规划出的轨迹不带时间戳生成，控制器因首两点时间差为0直接报错 `Goal request rejected` 导致超时 abort。
- **joint_trajectory_controller 容差配置**：`goal_time` 建议设置为 `0.0`，让控制器根据轨迹自带的 duration 和 MoveIt 裕量协同动态判定，避免硬编码超时引发误判。
- **机械臂真机安全使能机制**：启动时 `dummyarm_hardware` 默认失能（无力矩），提供 `/dummyarm/enable` 与 `/dummyarm/disable` 两个标准服务。在 RViz 中点击执行前，必须先在终端通过 `ros2 service call /dummyarm/enable std_srvs/srv/Trigger` 完成人工确认与电机闭环锁轴。
- **controller_manager 启动卡死在等待 robot_description**：切勿在 launch 中使用 `ParameterValue(Command(['xacro', ...]))` 给 `ros2_control_node`，必须使用 Python 的 `xacro.process_file(...).toxml()` 直接生成纯 XML 字符串传递，避免因 launch substitution 延迟导致控制器无限等待 topic。
- **ROS 2 硬件接口声明严格匹配（Discrepancy between robot description file and actually exported HW interfaces）**：URDF 中 `<ros2_control>` 块声明的 `command_interface` 与 `state_interface` 必须与 C++ 硬件插件 `export_command_interfaces()` / `export_state_interfaces()` 完全一一对应。若 URDF 中多声明了硬件未支持的接口（如虚假的 `velocity` 或 `effort`），`resource_manager` 会直接判定硬件初始化失败并挂起 `controller_manager`，导致控制器无法启动。
- **ROS 2 控制器并发加载冲突（Can't accept new action goals. Controller is not running）**：切勿在 launch 中并列启动多个独立 `spawner` 节点分别加载 broadcaster 和 trajectory controller，否则会引发 controller_manager 内部竞争死锁导致轨迹控制器未激活（inactive）；应在一个 `spawner` 命令中按序传入所有控制器名称并附带 `--param-file`。

## 7. 仓库/工具清单速查

scripts/: build.sh generate_true_urdf.py(真值链+PCA视觉) import_export_urdf.py fix_export_frames.py
  align_parts.py align_precise.py assemble_stack.py render_check.py extract_step_joints.py
  parse_step_assembly.py step_statement_parse.py step_world_compound.py step_solids_world.py
  step2world_xcaf.py(OCP, ~/occvenv 运行)
资料: ~/Dummy相关资料(厂商包) ~/Dummy_work(解压: ren20250527 任同学资料/fw20260222 固件) ~/Dummy-Robot(原版) ~/dummy(auk gitee) ~/DH参数(图)
