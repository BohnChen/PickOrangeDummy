# pickorange_chassis

底盘的**唯一写者**：`/cmd_vel` ⇄ Arduino RouterBridge RPC。

对应 `docs/architecture_v2.md` §6.5 / §10 / §15。

## 为什么需要这一层

`/dev/ttySTM0` 被 `arduino-router` 独占，宿主机 Python 没有 `arduino` 包。
更要紧的是：**底盘是"租约"模型**——不持续发指令就硬停，而且档位信号异常时
车可能继续前进。所以必须有一个**专职、独立线程、永不阻塞**的节点来当这个写者。

## 安全默认：默认不动车

```bash
# 只读模式（默认）：连上去、读 status、发布 /chassis/state，绝不发运动指令
ros2 run pickorange_chassis chassis_node \
  --ros-args -p rpc_target:=tcp:127.0.0.1:7501

# 真正驱动（确认车架空或周围无人之后）
ros2 run pickorange_chassis chassis_node \
  --ros-args -p rpc_target:=tcp:127.0.0.1:7501 -p allow_motion:=true
```

`allow_motion=false` 时：

- `/cmd_vel` 会被换算并显示在 `/chassis/state` 上，但**不会下发**
- `chassis/creep` 直接 ABORTED（`error_code=DRY_RUN`）
- `chassis/estop` / `arm` / `stop` 仍然有效（它们不会让车动）

## 连接方式

宿主机用户不在 `arduino-router` 组时，用中继：

```bash
./scripts/router_proxy.sh start      # 起一个容器做 unix→TCP 中继（免 sudo）
ros2 run pickorange_chassis chassis_node --ros-args -p rpc_target:=tcp:127.0.0.1:7501
```

拿到权限后（`sudo usermod -aG arduino-router $USER` + 重新登录）改用直连，代码不变：

```bash
... -p rpc_target:=unix:/var/run/arduino-router.sock
```

## 接口

| 类型 | 名称 | 说明 |
|---|---|---|
| Sub | `/cmd_vel` (`TwistStamped`) | 超过 `cmd_timeout_s`（默认 0.3 s）没更新就收油 |
| Pub | `/chassis/state` (`ChassisState`) | 10 Hz，含租约剩余、锁存状态、看门狗计数 |
| Srv | `/chassis/estop` | **固件锁存**急停，必须 `arm` 才解除 |
| Srv | `/chassis/arm` | 解除锁存（输出保持中位） |
| Srv | `/chassis/stop` | 立刻中位（滑行） |
| Act | `/chassis/creep` | 有限步进，到点必停 |

### 符号映射（写错的后果很严重）

```python
thr   =  k_linear  * linear.x          # 前进为正
steer = -k_angular * angular.z         # 底盘 steer>0=右，ROS angular.z>0=左 ⇒ 取负
```

写反了整条视觉伺服会镜像修正，而且**调试时看不出来**——车会朝着错误的方向调整。

## 两个实测出来的硬约束

1. **油门有最小有效值。** 实测 15% 车几乎不动、30% 才明显起步。
   所以 `creep` 会把 `max_speed` 抬到 `THR_MIN_EFFECTIVE = 30`。
2. **斜率限制决定最小步长。** 固件每帧最多变 1%，约 50%/s，
   从 0 升到 30% 要 0.6 s，这段时间已经在走了。
   所以 `creep` 的 `distance_m` 小于 `CREEP_MIN_STEP_M = 0.10 m` 会被钳制并告警——
   **"走 3 cm"这种指令在物理上做不到**，它只会变成一段加速-减速的蠕动。

   → 停车策略因此依赖"窗口"而不是"精确位置"：车负责把目标送进窗口，
   毫米级交给机械臂（见架构 §12）。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `rpc_target` | `unix:/var/run/arduino-router.sock` | `unix:` 或 `tcp:` |
| `allow_motion` | **false** | 安全默认 |
| `max_speed_pct` | 50 | 软限速 |
| `k_linear` | 100.0 | %/(m/s) |
| `k_angular` | 100.0 | %/(rad/s) |
| `cmd_timeout_s` | 0.30 | `/cmd_vel` 过期即收油 |
| `v_per_pct` | 0.008 | m/s per 1%（**占位，待标定**） |
| `rate_hz` | 20.0 | 续租约频率 |

## 已验证（2026-10-31）

```
/chassis/state      → throttle 30%, steer -50（angular.z=+0.5 → 左）✓ 符号正确
/chassis/estop      → 固件 latched=1，mode=FAULT ✓
/chassis/arm        → latched=0，mode=STOP ✓
/chassis/creep      → allow_motion=false 时 ABORTED/DRY_RUN ✓
mc.sh status        → stops 计数 4→5，确认指令真的到了 MCU ✓
```
