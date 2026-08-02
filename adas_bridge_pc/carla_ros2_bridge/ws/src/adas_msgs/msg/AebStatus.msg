# AEB 结构化状态（继承旧栈 aeb_overlay 结构化诊断思想）
uint8 STATE_INACTIVE=0
uint8 STATE_MONITORING=1
uint8 STATE_WARNING=2
uint8 STATE_EMERGENCY=3

std_msgs/Header header
uint8 state
float32 ttc_s                   # 最近碰撞点 TTC；无风险时为 +inf 语义（发布方填大值）
float32 required_decel_mps2     # 避碰所需减速度（正值）
string reason
