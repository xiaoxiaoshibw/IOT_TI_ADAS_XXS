std_msgs/Header header
uint8 IDLE=0
uint8 WAITING_FOR_MAP=1
uint8 PLANNING=2
uint8 DRIVING=3
uint8 ARRIVED=4
uint8 FAILED=5
uint8 CANCELED=6
uint8 state
# P0.C 会话隔离：本次运行的 run_id。空代表启动握手尚未建立；消费端必须
# 仅接受与本地 run_id 一致的消息，禁止将空 run_id 当作通配符。
string run_id
string map_id
string goal_id
string route_id
string detail
float32 remaining_distance_m
