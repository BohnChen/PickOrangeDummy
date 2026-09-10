# Dummy_URDF — Robot Description

> **DummyArm 项目内用法**（2026-09-10 更新：直接采用 Fusion 导出，补丁版导出器）：
> - `ros2 launch dummyarm_cad_description display_jsp.launch.py` — 滑块版（初始=设计姿态/REST），看模型/演示用
> - `ros2 launch dummyarm_cad_description display.launch.py` — 导出器原版（mock ros2_control + 3 控制器），可用 `/position_controller/commands` 驱动
> - 关节值 = **直接导出的 CAD 约定**（设计姿态/REST = 0）；限位按"相对 REST"（J1 ±170° / J2 [0,163°] / J3 [-145,0°] / J4 ±180° / J5 ±95° / J6 continuous）
> - 关节名保持 CAD 的 `_1.._6`（真值链 `dummyarm_description` 用 `Joint1..6` + 模型约定）
> - 导出/修复流程与坑见 `docs/fusion_export_workflow.md`

![Dummy_URDF](images/robot.png)

## Overview

| Property | Value |
|----------|-------|
| Total mass | 9.325 kg |
| Links | 7 |
| Joints | 6 (6 movable) |
| Assemblies | 17 |
| Root link | `base_link` |

## Table of Contents

- [Kinematic Tree](#kinematic-tree)
- [Link Properties](#link-properties)
- [Joint Properties](#joint-properties)
- [Assembly Breakdown](#assembly-breakdown)
- [Quick Start (ROS 2)](#quick-start-ros-2)
- [Files](#files)

## Kinematic Tree

```
base_link [BAKE]
  └─ _1 [revolute]
    J1 [BAKE]
      └─ _2 [revolute]
        J2J3 [BAKE]
          └─ _3 [revolute]
            J4 [BAKE]
              └─ _4 [revolute]
                J5 [BAKE]
                  └─ _5 [revolute]
                    J6 [BAKE]
                      └─ _6 [continuous]
                        tool0 [BAKE]
```

## Link Properties

| Link | Mass (kg) | Material | Collision | Bodies |
|------|-----------|----------|-----------|--------|
| `J1` | 0.4287 | material | box | 1 |
| `J2J3` | 2.6121 | material | box | 1 |
| `J4` | 1.0217 | material | box | 1 |
| `J5` | 0.9671 | material | box | 3 |
| `J6` | 0.4100 | material | box | 3 |
| `base_link` | 3.8115 | material | sphere | 1 |
| `tool0` | 0.0738 | material | box | 1 |

## Joint Properties

| Joint | Type | Parent → Child | Axis | Limits |
|-------|------|---------------|------|--------|
| `_1` | revolute | `base_link` → `J1` | (0,0,1) | [-170.0°, 170.0°] |
| `_2` | revolute | `J1` → `J2J3` | (0,0,1) | [0.0°, 163.0°] |
| `_3` | revolute | `J2J3` → `J4` | (0,0,1) | [-145.0°, 0.0°] |
| `_4` | revolute | `J4` → `J5` | (0,0,1) | [-180.0°, 180.0°] |
| `_5` | revolute | `J5` → `J6` | (0,0,1) | [-95.0°, 95.0°] |
| `_6` | continuous | `J6` → `tool0` | (0,0,1) | — |

## Assembly Breakdown

### 

- **Links**: 
- **Total mass**: 0.000 kg

### 35Motor_Driver

- **Links**: 
- **Total mass**: 0.000 kg

### 35Motor_Driver_1

- **Links**: 
- **Total mass**: 0.000 kg

### 35Motor_Driver_2

- **Links**: 
- **Total mass**: 0.000 kg

### 35_20Motor_Assembly

- **Links**: J5
- **Total mass**: 0.967 kg

### 35_20Motor_Assembly_1

- **Links**: J6
- **Total mass**: 0.410 kg

### 35_28Motor_Assembly

- **Links**: 
- **Total mass**: 0.000 kg

### 42_34Motor_Assembly

- **Links**: 
- **Total mass**: 0.000 kg

### 42_34Motor_Assembly_1

- **Links**: 
- **Total mass**: 0.000 kg

### 42_40Motor_Assembly

- **Links**: 
- **Total mass**: 0.000 kg

### J1

- **Links**: J1
- **Total mass**: 0.429 kg

### J2J3

- **Links**: J2J3
- **Total mass**: 2.612 kg

### J4

- **Links**: J4
- **Total mass**: 1.022 kg

### J5

- **Links**: 
- **Total mass**: 0.000 kg

### J6

- **Links**: 
- **Total mass**: 0.000 kg

### base_link

- **Links**: base_link
- **Total mass**: 3.812 kg

### tool0

- **Links**: tool0
- **Total mass**: 0.074 kg

## Quick Start (ROS 2)

```bash
# 1. Copy package to your ROS 2 workspace
cp -r dummyarm_cad_description ~/ros2_ws/src/

# 2. Build
cd ~/ros2_ws
colcon build --packages-select dummyarm_cad_description
source install/setup.bash

# 3. Visualize in RViz2
ros2 launch dummyarm_cad_description display.launch.py

# 4. Validate URDF structure
check_urdf install/dummyarm_cad_description/share/dummyarm_cad_description/urdf/Dummy_URDF.urdf

# 5. Print kinematic tree
urdf_to_graphviz install/dummyarm_cad_description/share/dummyarm_cad_description/urdf/Dummy_URDF.urdf
```

**Joint control**: The launch file includes `joint_state_publisher_gui` —
use the sliders to move revolute/prismatic joints in RViz2.

**Topic inspection**:
```bash
# See published joint states
ros2 topic echo /joint_states

# See robot description parameter
ros2 param get /robot_state_publisher robot_description
```

## Files

| Path | Description |
|------|-------------|
| `urdf/Dummy_URDF.urdf.xacro` | Top-level xacro (entry point) |
| `urdf/Dummy_URDF.urdf` | Flat URDF (for validation) |
| `urdf/assemblies/` | Per-assembly xacro macros |
| `meshes/` | Visual (OBJ) and collision (STL) meshes |
| `launch/display.launch.py` | Launch robot_state_publisher, RViz, and generated controllers |
| `config/joint_state.yaml` | Joint state publisher config |
| `config/ros2_controllers.yaml` | Generated ros2_control controller manager config |
| `robot_data.yaml` | Supplementary data (beyond URDF) |
| `docs/transforms.md` | Transformation matrices (KaTeX) |

## Customizing

Assemblies tagged `!dummy_` are designed to be swapped out. To replace one:

1. Create your replacement as a xacro macro with the same interface
2. Place it in `urdf/assemblies/`
3. Update the `<xacro:include>` in `urdf/Dummy_URDF.urdf.xacro`
4. Update meshes in `meshes/<your_assembly>/`

The xacro prefix system (`${prefix}`) ensures link names stay unique
when multiple instances of the same assembly are used.

---
*Generated by Fusion URDF/XACRO Exporter v3.1.0*