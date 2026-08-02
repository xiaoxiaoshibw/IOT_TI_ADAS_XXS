# 语义化全局路线点：几何 + 地图拓扑 + 道路约束
uint8 MANEUVER_UNKNOWN=0
uint8 MANEUVER_STRAIGHT=1
uint8 MANEUVER_LEFT=2
uint8 MANEUVER_RIGHT=3
uint8 MANEUVER_LANE_CHANGE_LEFT=4
uint8 MANEUVER_LANE_CHANGE_RIGHT=5
uint8 MANEUVER_STOP=6

float64 x
float64 y
float64 yaw
int64 lane_id                    # 与 MapLane.id 同宽，禁止截断 CARLA 编码 lane id
int32 road_id
float32 speed_limit
uint8 maneuver
