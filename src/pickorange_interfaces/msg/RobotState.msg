# 全局状态黑板（mission 发布，10 Hz）
#
# 唯一真相：任何界面 / 语音 / 日志都从这里读，不要各自推断。

string mode                # IDLE | CRUISE | SLOW | STOP | PICK | RESUME | FAULT | ESTOP
string task                # harvest | idle | ...
string substate            # PICK 内部：PRE_APPROACH | INSERT | HOLD | CUT | RETREAT

uint32 error_code
string error_text

uint32 target_id           # 当前作业目标；0 = 无
uint32 queue_len           # 当前停车位剩余的待采目标数
uint32 picked_total        # 本次任务累计采摘数

bool voice_listening       # 语音是否在听（半双工时为 false）
bool estop_active

builtin_interfaces/Time stamp
