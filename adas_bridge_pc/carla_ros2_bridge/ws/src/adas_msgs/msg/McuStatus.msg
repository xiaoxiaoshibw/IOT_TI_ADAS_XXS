# MCU 安全状态遥测：CAN 网关解码 0x202 心跳 / 0x203 诊断 / 0x204 E2E 诊断后发布。
# 取值与 MCU/include/adas_can_protocol.h、MCU/include/safety.h 保持一致；
# GUI/遥测据此显示控制源、安全状态、故障码、最后有效命令年龄和降级原因。

std_msgs/Header header

# system_state（SYS_MODE_*）
uint8 SYS_INIT=0
uint8 SYS_STANDBY=1
uint8 SYS_ACTIVE=2
uint8 SYS_DEGRADED=3
uint8 SYS_MRM=4
uint8 SYS_EMERGENCY_BRAKE=5
uint8 SYS_FAILSAFE=6
uint8 SYS_FAULT_LOCK=7

# active_source（SRC_*）
uint8 SRC_NONE=0
uint8 SRC_PRIMARY=1
uint8 SRC_BACKUP=2
uint8 SRC_WATCHDOG=9

uint8 system_state
uint8 active_source
uint8 fault_level
uint8 loop_load_pct

# 0x202 安全标志位
bool primary_fresh
bool backup_fresh
bool aeb_floor_active
bool degraded
bool manual_override
bool estop

# 0x203 诊断
uint16 fault_code            # FC_* 位图
uint8 reset_reason
uint8 primary_timeout_count
uint8 backup_timeout_count
uint8 crc_error_count
uint8 loop_overrun_count

# 0x204 E2E 诊断
uint8 protocol_version
bool protocol_version_ok
bool test_build
uint8 self_test_mask
uint8 can_recovery_count

# 新鲜度证据（秒；负值表示从未收到）
float32 heartbeat_age_s      # 最近有效 0x202
float32 diag_age_s           # 最近有效 0x203
float32 feedback_age_s       # 最近有效 0x201 执行反馈
float32 command_age_s        # 上游 gate 控制命令最近被网关接受的年龄

# 由 fault_code/state 派生的可读降级原因，供 GUI 直接显示
string degrade_reason
