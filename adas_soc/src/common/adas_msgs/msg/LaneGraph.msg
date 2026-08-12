std_msgs/Header header
# P0.C 会话隔离：LaneGraph 是会话启动握手（首次可接受 run_id==""，随后锁定），
# 后续 LaneGraph 必须携带相同 run_id，否则消费端视为旧会话残留并丢弃。
string run_id
string map_id
string map_hash
adas_msgs/MapLane[] lanes
