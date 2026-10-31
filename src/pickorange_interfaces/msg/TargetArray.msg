# 一次感知周期的目标列表
#
# 契约：
#   - targets 已按优先级排序（可达 > 距离近 > 置信度高）
#   - selected_id 是任务层当前选中的目标 id；0 表示未选中
#   - 目标列表最多保留 5 个，避免下游对着一堆不可采目标打转

std_msgs/Header header       # header.frame_id = base_link

Target[] targets
uint32 selected_id
