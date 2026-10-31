# pickorange_interfaces

整机接口定义（msg / action / srv）。对应 `docs/architecture_v2.md` §8。

**设计约束**

- 只依赖 ROS 标准消息，不依赖任何驱动包
- 所有位姿一律注明坐标系；车体几何用 `base_footprint`，MoveIt 目标用 `base_link`
- 「有完成语义」的操作一律用 Action，不用 Topic + 猜

## 内容

| 类型 | 名称 | 用途 |
|---|---|---|
| msg | `Target` | 单个候选目标 + 可采性判定 |
| msg | `TargetArray` | 一次感知周期的目标列表（已排序） |
| msg | `ChassisState` | 底盘状态（租约 / 锁存 / 电池） |
| msg | `HazardReport` | 前方通行安全 |
| msg | `RobotState` | 全局状态黑板 |
| action | `MoveToPose` | 机械臂笛卡尔移动（MoveIt MoveGroup） |
| action | `CutStem` | 果梗剪切（原子：合刀→保压→张刀） |
| action | `Creep` | 底盘有限步进（绝不"一直走"） |
| srv | `CheckReachable` | 可达性判定（compute_ik + 碰撞） |

## 编译

```bash
cd ~/01_MyProject/PickOrangeDummy
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pickorange_interfaces --symlink-install
source install/setup.bash
ros2 interface list | grep pickorange
```
