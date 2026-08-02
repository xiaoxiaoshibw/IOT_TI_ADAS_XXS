# 命令门控状态（继承旧栈 GateDecision 思想）
uint8 SOURCE_FOLLOWER=0
uint8 SOURCE_AEB=1
uint8 SOURCE_BUILTIN_STOP=2

std_msgs/Header header
uint8 selected_source
bool limited                    # 本帧是否被限幅滤波修剪过
string reason
