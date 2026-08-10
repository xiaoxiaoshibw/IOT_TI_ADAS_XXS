# 车辆接口输出：归一化执行量（对标 Autoware raw_vehicle_cmd_converter 输出）
std_msgs/Header header
float32 throttle        # 0..1
float32 brake           # 0..1
float32 steer           # -1..1，左正
