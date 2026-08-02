# 单个跟踪目标（对齐 Autoware PredictedObject 精简版）
uint8 CLASS_UNKNOWN=0
uint8 CLASS_CAR=1
uint8 CLASS_TRUCK=2
uint8 CLASS_PEDESTRIAN=3
uint8 CLASS_BICYCLE=4

uint32 id
uint8 classification
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist        # 目标自身坐标系纵向速度在 twist.linear.x
geometry_msgs/Vector3 dimensions               # 长(x)宽(y)高(z) [m]

# 路径坐标系投影（object_tracker 填充；原始目标发布方置 0）
float32 path_longitudinal_m 0                  # 沿参考路径纵向（自车前方为正）
float32 path_lateral_m 0                       # 相对车道中心线横向（左正）
