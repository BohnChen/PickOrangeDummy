# 会话交接：S0 收尾 → S1 起步（2026-10-31）

> ⚠️ **本文已部分过时（2026-11-01）**：S1 的手眼标定**已完成**
> （ArUco + PnP + 6 点 SVD，留出验证 7.6mm），不再卡在这一步。
> 最新全貌请看 **`docs/PROJECT_REVIEW_2026-11-01.md`**。
> 本文的硬件/固件/环境部分仍然有效。

## 一句话现状

**S0（硬件地基）已完成**：双相机独立出图、底盘固件 v3、整车 TF、安全保护全部就绪。
**S1 卡在一步**：相机↔机械臂手眼标定未做，因此还不能把视觉坐标交给机械臂。

## 硬件实况（今天验证过）

| 设备 | 连接 | 状态 |
|---|---|---|
| Astra Pro Plus | 板载 USB2（经 Genesys hub） | 深度/彩色/IR 正常，**近距 <0.6m 无返回** |
| RealSense D435i | **直插板子 USB3.0 Type-A，5 Gbps** | 彩色+对齐深度 ~15Hz，IMU 可读（IIO） |
| 语音 USB 声卡 | 扩展坞（现在拔了） | 录音正常，只支持 44.1kHz |
| HC-SR04 + 9G 舵机 | D10/D11/D9，3.3V 供电 | 扫描正常，**已改为静止断电防抖** |

> ⚠️ 扩展坞带不动全部外设（RealSense 一开流就整树重枚举）。RealSense 必须直插板子。

## 固件

- `firmware/chassis_v3_sonar/` 是当前主固件（已烧录）
  - 底盘 3 路 PWM（D3/D5/D6）+ 300ms 租约 + 固件锁存急停
  - 舵机改**硬件 PWM**（TIM4_CH2/PB7），静止 1.5s 断脉冲 → **不再抖动**
  - 车速自适应扫描：停车全扫 / 慢速窄扫 / >30% 锁定正前方
  - MCU 内近距刹车：`250mm + 5mm×油门%`
  - **默认 `auto_scan=0`**：上电后舵机静止，等系统统一起动再唤醒
- 烧录：`./scripts/flash_chassis.sh chassis_v3_sonar`（默认就是它）

## 怎么把系统跑起来

```bash
# 1) 底盘 RPC 中继（免 sudo）
./scripts/router_proxy.sh start

# 2) Astra
source ~/ros2_ws_orbbec/install/setup.bash
ros2 launch orbbec_camera astra_pro_plus.launch.py

# 3) D435i（必须带命名空间，否则和 Astra 撞在 /camera）
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=d435i camera_namespace:=d435i \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30 align_depth.enable:=true

# 4) 双相机预览
./scripts/web_dual_cam.sh        # 浏览器开 http://<板子IP>:8888

# 5) 底盘（只读模式，默认不动车）
ros2 run pickorange_chassis chassis_node --ros-args -p rpc_target:=tcp:127.0.0.1:7501

# 6) 超声手动检查
./scripts/sonar.py info
./scripts/sonar.py park 90
./scripts/sonar.py sweep
```

## 明天的第一件事：手眼标定

**为什么必须做**：旧外参是"相机在臂右前上方 46.8cm 俯视"时测的，
现在是"车头 310mm 高、朝左 100°"，**完全作废**。`transforms.yaml` 里只是机械估计。

**方法（末端贴标记，全自动）**

1. 在末端执行器尖端贴一小块亮色胶带（5mm 左右，黄/红）
2. 让机械臂在 D435i 视野内摆 6 个不同位姿
3. 每个位姿采集：相机检测标记的 3D 点 + FK 给出的 `tool0` 坐标
4. SVD 解 `d435i_color_optical_frame → base_link` 的刚体变换
5. 写回 `src/pickorange_bringup/config/transforms.yaml` 的 `d435i` 段
6. 复验：末端与相机反投影目标误差目标 **< 1 cm**

## S1 剩下的活

- [ ] `pickorange_perception` 包 + `target_detector`（HSV→3D，接口保持可换 ONNX）
- [ ] `astra_guard`（Astra + 超声融合 → `/perception/hazards`）
- [ ] `srv/CheckReachable`（MoveIt compute_ik + 车体碰撞盒）
- [ ] 模型权重仍不在板上（`dummyarm_AImodel/runs/` 为空）→ 今天用 HSV

## 踩过的坑（别再踩）

1. 两台相机默认都占 `/camera`，D435i 必须加 `camera_name/camera_namespace`
2. 扩展坞供电不足 → 整树反复重枚举，RealSense 报 `VIDIOC_QBUF: No such device`
3. HC-SR04 的 ECHO 上升沿延迟 **~2.2ms**（不是几百 µs），等待窗口给 3ms
4. 舵机角度变量不能用 `int8_t`（>127° 溢出）
5. 舵机抖动 = 控制环 limit cycle：硬件 PWM 治电、静止断脉冲治机械
6. `pkill -f xxx` 会连自己所在的 shell 一起杀（用 `pgrep -f "[x]xx"` 的括号技巧）
