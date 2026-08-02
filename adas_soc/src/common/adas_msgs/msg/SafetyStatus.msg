# 整机安全态聚合（safety_monitor 输出）
uint8 LEVEL_OK=0
uint8 LEVEL_WARN=1
uint8 LEVEL_MRM_COMFORT=2       # 请求舒适停车（gate 切 builtin_stop）
uint8 LEVEL_MRM_EMERGENCY=3     # 请求紧急停车

std_msgs/Header header
uint8 overall
string[] failed_components
