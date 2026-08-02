# 双机冗余心跳（M5）。AEB 标志必选随心跳——继承旧栈实测教训：
# 主控在紧急制动中死亡时，备机接管需要用不同的限速策略。
uint8 ROLE_PRIMARY=0
uint8 ROLE_BACKUP=1
uint8 HEALTH_OK=0
uint8 HEALTH_WARN=1
uint8 HEALTH_ERROR=2

std_msgs/Header header
uint64 seq                      # 循环序号：socket 活着不等于循环活着，seq 停滞是第二判据
uint8 role
uint8 health
Control last_control            # 接管 seed
bool aeb_active
