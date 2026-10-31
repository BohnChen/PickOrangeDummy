# 一个候选采摘目标（base_link 下的三维点 + 可采性判定结果）
#
# 契约：
#   - position 一律在 header.frame_id 指定的坐标系下（当前固定 base_link）
#   - reachable 由 /arm/check_reachable 判定，不是检测器自己猜的
#   - reject_reason 仅用于调试与语音提示，逻辑不得依赖它的具体文字

uint32 id
string class_name            # orange | occluded_orange | pedicel | tree
float32 confidence

geometry_msgs/Point position # base_link 下，单位米
float32 radius_m             # 果实半径估计（用于套筒预对准）

bool reachable
string reject_reason         # 不可采原因（超出包络 / IK 无解 / 碰撞 / 置信度低）

builtin_interfaces/Time stamp
