# 地图交通标志/路口提示
uint8 TYPE_STOP_SIGN=0
uint8 TYPE_TRAFFIC_LIGHT=1
uint8 TYPE_JUNCTION=2

uint8 LIGHT_UNKNOWN=0
uint8 LIGHT_RED=1
uint8 LIGHT_GREEN=2

uint8 type
uint8 traffic_light_state
geometry_msgs/Point position
int64 lane_id
