# PickOrangeDummy

Dummy 机械臂（auk/国产化版）ROS2 真机控制与操作实验平台。

目标（详见 `docs/`）：把一台基于 peng-zhihui Dummy-Robot 复刻的超紧凑六轴机械臂
建设成 **ROS2 机械臂任务执行与安全监控系统**——经典 pick-and-place 闭环起步，
远期面向低位柑橘采摘（RGB-D 视觉 + 旋转剪切末端执行器 + 移动底盘）。

## 仓库布局

```
docs/
  architecture.md           # 架构决策（混合架构 C：桥接 → ros2_control）
  safety_and_incidents.md   # 事故史 + 硬性安全约束（先读）
src/
  dummyarm_description/     # URDF/Xacro 数字样机（CAD 网格 + 真机固件限位）
  dummyarm_bridge/          # [开发中] 真机桥接节点（joint_states + 管理服务）
  dummyarm_hardware/        # [规划] ros2_control SystemInterface (C++)
  dummyarm_bringup/         # [规划] launch 聚合
```

## 硬件/环境基线

- 机械臂：Dummy V2 复刻（STM32F405 REF 主控，USB-CDC ASCII 协议 @115200）
- 控制主机：VENTUNO Q（Ubuntu 24.04 / ROS 2 Jazzy / arm64）
- 独立控制库：`dummyarm_link`（见个人工具目录，pyserial 封装已验证协议）

## 状态

- [x] 阶段 0 环境就绪（Jazzy + xacro + ros2_control）
- [x] 阶段 1 真机链路正规化（dummyarm_link，自测通过）
- [x] 阶段 2 架构批准（v1.0）
- [x] M1 数字样机骨架（description 包，TF 验证通过）
- [ ] 3a 真机桥接节点（进行中）
- [ ] 3b ros2_control 工程化
