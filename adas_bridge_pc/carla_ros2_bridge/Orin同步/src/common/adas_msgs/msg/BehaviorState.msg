# 行为层输出：当前行为状态 + 目标车道/速度
uint8 STATE_LANE_FOLLOW=0
uint8 STATE_FOLLOW_LEAD=1
uint8 STATE_OVERTAKE_WAIT=2
uint8 STATE_OVERTAKE_ACTIVE=3
uint8 STATE_OVERTAKE_RETURN=4
uint8 STATE_STOPPING=5
uint8 STATE_EMERGENCY=6

std_msgs/Header header
uint8 state
float32 target_speed_mps
int8 target_lane 0              # 0=当前车道，-1=左邻道，+1=右邻道
