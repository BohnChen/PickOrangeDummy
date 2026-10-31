# 底盘状态（chassis_node 发布，20 Hz）
#
# 语义对齐 firmware/chassis_v2_atomic：
#   - 固件是「租约」模型：任何指令把租约续到 now + 300 ms，过期立即硬停
#   - estop 是固件内「锁存」：置位后一切运动指令返回 false，必须显式 arm()
#   - brake() 不存在；STOP = 三路 1500 µs 中位 = 滑行

int8 MODE_STOP  = 0
int8 MODE_FWD   = 1
int8 MODE_REV   = 2
int8 MODE_FAULT = 3

int8 mode

float32 throttle_pct         # -100..100（负数 = 倒车）
float32 steer_pct            # -100..100（**正 = 右**，与 ROS angular.z 相反）

bool leased                  # 租约当前是否有效
float32 lease_remaining_s    # 距硬停还有多久（> 0.3 说明指令没跟上）

bool estop_latched           # 固件锁存急停是否置位
uint32 wd_stops              # 看门狗强制停车累计次数（判断链路是否在抖）
uint32 stop_calls            # 显式 stop() 累计次数

float32 battery_v            # 0.0 = 未接电压采样（需硬件 ADC 分压，见架构 §3.8）

uint32 cmd_seq               # 已接受的指令序号（用于丢弃过期指令）
