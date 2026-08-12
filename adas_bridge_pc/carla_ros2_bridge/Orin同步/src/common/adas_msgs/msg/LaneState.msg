# 车道相对状态（v1 由仿真桥直通真值；日后可换真感知）
# 符号约定：左正右负（横向偏移、航向误差）；左转曲率为正
std_msgs/Header header
bool valid
float64 lateral_offset          # 自车相对车道中心线横向偏移 [m]
float64 heading_error           # 自车航向 - 车道切向 [rad]
float64 curvature               # 车道中心线曲率 [1/m]
float64 lane_width              # 车道宽 [m]
bool left_lane_available        # 左侧存在同向 Driving 车道且当前允许变道
bool right_lane_available       # 右侧存在同向 Driving 车道且当前允许变道
