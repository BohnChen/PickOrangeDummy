# 前方通行安全（astra_guard 发布，30 Hz）
#
# 只回答一个问题：正前方会不会撞上。
# 不做语义、不识别橘子（那是 D435i + target_detector 的事）。

int8 SUGGEST_GO   = 0
int8 SUGGEST_SLOW = 1
int8 SUGGEST_STOP = 2

float32 nearest_distance_m   # 走廊 ROI（|y| < 0.35 m, z ∈ [0.3, 1.5]）内的 5% 分位数
float32 nearest_bearing_rad  # 相对车头方位，左正右负
int8 suggestion

bool valid                   # false = 数据不可信（丢帧 / 遮挡 / 全无效深度），此时必须按 STOP 处理
