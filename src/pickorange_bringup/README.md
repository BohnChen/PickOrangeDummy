# pickorange_bringup

整机顶层 launch 与标定配置。

**核心约定：所有几何参数集中在 [`config/transforms.yaml`](config/transforms.yaml)，launch 里不写死任何数字。**
（v1 把 6 个手眼外参写死在 `perception_bringup.launch.py`，改一次要翻代码——这次不再重复。）

## tf.launch.py

发布整车静态 TF + 可选机械臂 URDF。

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch pickorange_bringup tf.launch.py                    # 整车 TF + 机械臂
ros2 launch pickorange_bringup tf.launch.py with_arm:=false    # 只看车和相机
```

验证：

```bash
ros2 run tf2_ros tf2_echo base_footprint base_link   # 期望 (0.600, 0.040, 0.202)
ros2 run tf2_ros tf2_echo base_footprint astra_link  # 期望 (0.670, 0.000, 0.450)
```

## transforms.yaml 的 `accuracy` 字段

| 值 | 含义 |
|---|---|
| `MEASURED` | 卷尺实测 |
| `ASSUMED` | 按推算填的，**可能有几 cm 误差，用前确认** |
| `PLACEHOLDER` | 占位（如无里程计时的 `odom → base_footprint`） |

`ASSUMED` 的项不影响抓取精度（相机 4 点标定会吸收），但影响标定初值和 RViz 观感。
