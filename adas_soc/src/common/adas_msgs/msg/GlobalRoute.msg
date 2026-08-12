# 地图语义增强的全局路线；无效状态必须 points=[]，用于清除 transient-local 旧路线
uint8 STATUS_INVALID=0
uint8 STATUS_PLANNING=1
uint8 STATUS_VALID=2
uint8 STATUS_CANCELLED=3
uint8 STATUS_FAILED=4

std_msgs/Header header
uint32 route_id
# P0.C 会话隔离：消费端只在 run_id 一致时采纳，否则视为旧会话残留。
string run_id
string frame_id                  # 与 header.frame_id 一致，供非 Header 消费端显式校验
string map_id
string map_hash
string goal_id
adas_msgs/RoutePoint[] points
float32 length
uint8 status
