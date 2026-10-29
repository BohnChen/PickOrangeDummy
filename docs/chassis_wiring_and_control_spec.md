> ⚠️ **已修订（2026-10-29）**：本文部分内容已被实车验证结果取代——5 号通道应接 **D6**（非 D9），
> 且必须打开小车板 **SW2 拨码第 8 位**。最新结论见 [`chassis_control_verified.md`](chassis_control_verified.md)。

# 移动底盘硬件接线与信号控制技术规范
# Mobile Chassis Wiring & Signal Control Specification

> **项目名称**：PickOrangeDummy  
> **适用机型**：Arduino VENTUNO Q（高通 Dragonwing IQ-8275 + STM32H5F5 双脑架构）  
> **执行机构**：低位柑橘采摘移动底盘（3 通道 50Hz PWM 控制器）  
> **文档版本**：v1.0  
> **更新日期**：2026-10-27  

---

## 1. 架构总览与电气安全

在 PickOrangeDummy 项目规划中，移动底盘作为自主采摘的移动基座，与 Dummy 机械臂形成“感知-移动-采摘”协同。VENTUNO Q 采用分工架构：
- **大脑（高通 IQ-8275 / Ubuntu 24.04 / ROS 2 Jazzy）**：运行导航堆栈（Nav2）、手眼视觉伺服，发布 `/cmd_vel` 移动指令。
- **小脑（STM32H5F5 MCU / 实时固件）**：负责底层硬件定时、换向互锁、急停保护及 50Hz 舵机脉冲生成。

### 1.1 电气安全硬性约束（红线）
1. **电源严禁反灌**：底盘电池或动力板输出的 `+5V` 供电线**严禁插进 VENTUNO Q 任何引脚**（包括 `5V`、`VIN` 或 GPIO）。该线必须剪齐并用绝缘胶布单独包裹。
2. **共地连接（Common Ground）**：底盘控制器 GND 必须与 VENTUNO Q 的数字 GND 紧密互连，确保电平参考面一致。
3. **信号隔离**：接线前必须拔除小车原装无线接收机，防止接收机推挽输出与 VENTUNO Q 的 GPIO 发生总线冲突。
4. **架空测试**：任何涉及动力输出的首次通电或固件更新，驱动轮必须全程离地悬空。

---

## 2. 物理接口与引脚接线定义

VENTUNO Q 的微控制器（STM32H5F5）引脚引出在板载标准的 **Arduino UNO Shield 黑色排母**上（紧邻高通 SoC 散热片）。杜邦线需加装公头插针转接后插入。

### 2.1 接线对应表

| 信号类型 | 小车控制器测试点 | VENTUNO Q 接口与孔位 | MCU 硬件定时器通道 | 逻辑电平 | 说明 |
|---|---|---|---|---|---|
| **地线** | **GND**（黑/棕线） | **GND**（上段 10 孔排母，第 4 孔） | — | 0V | 电平参考地 |
| **通道 1** | **3 号点：油门** | **~D3**（下段 8 孔排母，第 5 孔） | `TIM3_CH4`（PB1） | 3.3V | 前进速度与刹车控制 |
| **通道 2** | **4 号点：转向** | **~D5**（下段 8 孔排母，第 3 孔） | `TIM3_CH2`（PB5） | 3.3V | 转向打角控制 |
| **通道 3** | **5 号点：倒车/方向**| **~D9**（上段 10 孔排母，第 9 孔） | `TIM4_CH2`（PB7） | 3.3V | 倒车驱动/方向选择 |
| **电源** | **+5V**（红线） | **悬空不接（绝缘隔离）** | — | — | **严禁接板** |

### 2.2 实物孔位布局（顶视图）

```
                     VENTUNO Q 板子上方（M.2 卡槽方向）
                                  │
                  ┌───────────────┴───────────────┐
                  │   【上段 10 孔排母 (DIGITAL)】  │
                  │   [ 1 ] D21 (SCL)             │
                  │   [ 2 ] D20 (SDA)             │
                  │   [ 3 ] AREF                  │
【底盘 GND】 ───> │   [ 4 ] GND                   │  <-- 接底盘参考地
                  │   [ 5 ] D13 (SCK)             │
                  │   [ 6 ] D12 (MISO)            │
                  │   [ 7 ] ~D11 (MOSI)           │
                  │   [ 8 ] ~D10 (SS)             │
【底盘 5 号脚】───> │   [ 9 ] ~D9                   │  <-- 接 5 号倒车
                  │   [10 ] D8                    │
                  └───────────────┬───────────────┘
                                  │ （物理间隙约 2mm）
                  ┌───────────────┴───────────────┐
                  │   【下段 8 孔排母 (DIGITAL)】   │
                  │   [ 1 ] D7                    │
                  │   [ 2 ] ~D6                   │
【底盘 4 号脚】───> │   [ 3 ] ~D5                   │  <-- 接 4 号转向
                  │   [ 4 ] D4                    │
【底盘 3 号脚】───> │   [ 5 ] ~D3                   │  <-- 接 3 号油门
                  │   [ 6 ] D2                    │
                  │   [ 7 ] D1 (TX)               │
                  │   [ 8 ] D0 (RX)               │
                  └───────────────┬───────────────┘
                                  │
                     VENTUNO Q 板子下方（网口 / USB 接口方向）
```

---

## 3. 信号控制逻辑规格

三路控制信号遵循标准模型舵机 PWM 时序：
- **载波频率**：`50 Hz`（脉冲重复周期 $T = 20\text{ ms}$）
- **高电平脉宽范围**：`1000 µs ~ 2000 µs`
- **中位（停止/回正）**：`1500 µs`（死区范围：`1475 µs ~ 1525 µs`）

### 3.1 各通道逻辑定义

#### 通道 1：3 号引脚（油门 / 刹车）
- **中位（1500 µs）**：电机不供电，处于自然滑行/怠速状态。
- **前进驱动区间（> 1525 µs ~ 2000 µs）**：脉宽大于中位死区时驱动底盘前进，脉宽越长前进速度越快。
- **主动制动区间（1000 µs ~ < 1475 µs）**：脉宽低于中位死区时启动制动（短接电机绕组抑制滑行），脉宽越低制动力度越大。

#### 通道 2：4 号引脚（转向舵机）
- **中位（1500 µs）**：转向轮居中直线行驶。
- **左转区间（1000 µs ~ 1475 µs）**：`1000 µs` 为物理左打死。
- **右转区间（1525 µs ~ 2000 µs）**：`2000 µs` 为物理右打死。
- **软件限幅约束**：为防止转向连杆机械过冲卡死，常规工作范围限幅在 `1100 µs ~ 1900 µs`。

#### 通道 3：5 号引脚（倒车 / 方向选择）
- **倒车激活（> 1525 µs ~ 2000 µs）**：触发底盘倒车驱动。
- **中位与前进释放（1000 µs ~ 1500 µs）**：保持非倒车状态。

---

## 4. 关键保护算法：换向互锁与安全看门狗

由于小车控制器包含独立的前进（3 号）与倒车（5 号）驱动通路，**绝对禁止两通道同时输出驱动脉宽**，否则会造成内部 H 桥直通短路或损坏传动齿轮。

### 4.1 状态转移与 250ms 换向死区
当底盘由“前进”切换至“倒车”或反之，必须执行强制降速归零并保持中位：
```
[状态 1: 前进中]
  3号脚: >1525us, 5号脚: 1500us
       │
       ▼ (收到倒车请求)
[状态 2: 归中等待]
  3号脚: 1500us, 5号脚: 1500us
  保持等待 >= 250ms (消除反电动势与机械转动惯量)
       │
       ▼ (等待超时完成)
[状态 3: 倒车执行]
  3号脚: 1500us, 5号脚: >1525us
```

### 4.2 通信丢失与失控保护（Watchdog Failsafe）
- 上层 Linux Python / ROS 2 节点必须以 $\ge 10\text{ Hz}$（间隔 $\le 100\text{ ms}$）的心跳周期向 MCU 发送目标指令。
- STM32 底层维护看门狗计数器，若超过 `300 ms` 未收到有效心跳包，**硬件底层立即无条件切断动力，所有通道脉宽复位至 1500 µs 并开启制动**。

---

## 5. 控制代码与 ROS 2 适配实现

### 5.1 STM32 实时控制层（Zephyr PWM 50Hz 驱动核心）
在 VENTUNO Q 的 Arduino Core for Zephyr 框架下，通过修改硬件定时器重载值保证真实的 50Hz 周期输出：

```cpp
#include <Arduino.h>
#include <zephyr/drivers/pwm.h>

static const struct device *pwm3 = DEVICE_DT_GET(DT_NODELABEL(pwm3));
static const struct device *pwm4 = DEVICE_DT_GET(DT_NODELABEL(pwm4));

struct PwmChannel {
  const struct device *dev;
  uint32_t channel;
  uint64_t cycles_per_sec;
};

// 映射引脚
static PwmChannel ch_throttle = {pwm3, 4, 0}; // ~D3 (3号脚: 油门)
static PwmChannel ch_steer    = {pwm3, 2, 0}; // ~D5 (4号脚: 转向)
static PwmChannel ch_reverse  = {pwm4, 2, 0}; // ~D9 (5号脚: 倒车)

void init_pwm_ch(PwmChannel &ch, uint8_t pin) {
  analogWrite(pin, 0); // 触发底层引脚复用映射
  pwm_get_cycles_per_sec(ch.dev, ch.channel, &ch.cycles_per_sec);
}

void write_us(PwmChannel &ch, uint16_t pulse_us) {
  uint32_t period_cycles = ch.cycles_per_sec / 50; // 50Hz -> 20ms
  uint32_t pulse_cycles  = (uint64_t)ch.cycles_per_sec * pulse_us / 1000000ULL;
  pwm_set_cycles(ch.dev, ch.channel, period_cycles, pulse_cycles, PWM_POLARITY_NORMAL);
}

void setup() {
  init_pwm_ch(ch_throttle, D3);
  init_pwm_ch(ch_steer, D5);
  init_pwm_ch(ch_reverse, D9);

  // 上电默认归位
  write_us(ch_throttle, 1500);
  write_us(ch_steer, 1500);
  write_us(ch_reverse, 1500);
}
```

### 5.2 上层 ROS 2 `/cmd_vel` 到微秒脉宽的转换关系

在 Linux Python 节点中，将 `Twist` 线速度 $v$（m/s）与角速度 $\omega$（rad/s）映射至通道脉宽：

$$\text{pulse}_{\text{steer}} = \text{clamp}\left(1500 + \text{gain}_{\text{steer}} \cdot \omega,\; 1100,\; 1900\right)$$

$$\begin{cases} 
v > 0: & \text{pulse}_{\text{throttle}} = \text{clamp}\left(1525 + k_{\text{throttle}} \cdot v,\; 1525,\; 2000\right), & \text{pulse}_{\text{reverse}} = 1500 \\
v < 0: & \text{pulse}_{\text{reverse}} = \text{clamp}\left(1525 + k_{\text{reverse}} \cdot |v|,\; 1525,\; 2000\right), & \text{pulse}_{\text{throttle}} = 1500 \\
v = 0: & \text{pulse}_{\text{throttle}} = 1400 \text{ (制动)}, & \text{pulse}_{\text{reverse}} = 1500
\end{cases}$$

---

## 6. 架空联调验收基线（Checklist）

完成硬件转接和固件烧录后，执行以下标准验证步骤：

1. **静态中位检查**：
   - 上电初始，万用表测量 D3、D5、D9 输出，占空比应恒定为 $7.5\%$（有效高电平约 $1.5\text{ ms}$）。底盘两驱动轮无异响、无爬行。
2. **转向行程校准**：
   - 注入 $1100\text{ µs}$ 观察是否向左打死且舵机无持续啸叫堵转；注入 $1900\text{ µs}$ 观察是否向右打死。
3. **单向油门爬升**：
   - 阶梯式提升 3 号脚（1550 -> 1600 -> 1700 -> 1800 µs），观察轮速是否阶梯式平稳提升。
4. **换向互锁验证**：
   - 从前进全速突发下发倒车全速，抓取示波器或逻辑分析仪波形，确认两引脚交替过程中存在 $\ge 250\text{ ms}$ 的全中位死区，严禁重叠脉冲。
5. **拔线失控保护**：
   - 底盘全速旋转中，拔除 VENTUNO Q 的上位机通信线，底盘驱动轮必须在 $300\text{ ms}$ 内彻底停转。
