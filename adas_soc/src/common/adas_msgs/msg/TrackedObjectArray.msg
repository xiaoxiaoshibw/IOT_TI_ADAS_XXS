# 跟踪目标数组 + 主前车选举结果
std_msgs/Header header
TrackedObject[] objects
int32 primary_lead_id -1          # 主前车 id；-1 = 无主前车
float32 primary_lead_gap_m 0      # 主前车纵向间距（沿路径，自车头基准）；无主前车时无意义
float32 primary_lead_speed_mps 0  # 主前车滤波后速度；无主前车时无意义
