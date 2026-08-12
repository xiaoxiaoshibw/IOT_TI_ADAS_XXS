std_msgs/Header header
# P0.C 会话隔离：LaneGraph 必须携带与本机会话一致的规范 UUID v4。
# 空 run_id 永不构成握手：上游 producer（CARLA 桥 / SIL map_provider）必须
# 由 launch 注入同一个 UUID v4；消费端（planner / adapter / GUI）一律拒
# 绝空 / 错配 / 非规范 ID 的消息。
string run_id
string map_id
string map_hash
adas_msgs/MapLane[] lanes
