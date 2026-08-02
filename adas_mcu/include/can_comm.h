/*
 * can_comm.h — CANA 收发 + 帧解码/编码（协议见 adas_can_protocol.h）
 *
 * 职责：初始化 CANA、按 ID 布置收/发邮箱；每 tick 轮询收邮箱，对 Orin
 *       主/备两路做 CRC+序列校验并解码为 SocCommand_t；提供 MCU 三帧回传。
 * 不做控制/仲裁决策——那是 control.c / safety.c 的事。
 */
#ifndef CAN_COMM_H_
#define CAN_COMM_H_

#include <stdint.h>
#include <stdbool.h>
#include "adas_types.h"

/* 源索引 */
#define SRC_IDX_PRIMARY   0U
#define SRC_IDX_BACKUP    1U
#define SRC_IDX_COUNT     2U

void CanComm_init(void);
void CanComm_service(uint32_t now_ms);
bool CanComm_isBusHealthy(void);
uint16_t CanComm_recoveryCount(void);
bool CanComm_recoveryExhausted(void);
/* 操作员强制恢复（长按清故障 / 0x301 CLEAR 触发，由 main 调用）：
 * 清零快速重试预算并立即重新初始化 CANA 尝试入网。物理故障已排除时可一键
 * 恢复而不必复位整机；若总线仍故障，下一 service 会重新判 bus-off 并再锁。 */
void CanComm_forceRecovery(uint32_t now_ms);

/* 每 1ms tick 调用：读所有收邮箱，校验+解码，刷新 link 时间戳/序列。
 * 返回本 tick 是否收到过至少一帧有效数据。crc_err_out 累加 CRC 失败次数。*/
bool CanComm_pollRx(uint32_t now_ms, uint16_t *crc_err_out);

/* 取解码结果（idx: SRC_IDX_PRIMARY/BACKUP） */
const SocCommand_t *CanComm_getCommand(uint16_t idx);
/* Clear decoded safety requests when status expires or a source is dropped.
 * Invalid source indices are ignored. */
void                CanComm_clearSafetyRequests(uint16_t idx);
LinkMonitor_t      *CanComm_getLink(uint16_t idx);

/* 故障注入：取最近一条 0x301 命令；has_new 为是否有未读命令 */
bool CanComm_getInjectCommand(uint16_t *cmd_out, uint16_t *param_out);
bool CanComm_getSessionRequest(uint16_t *request_out, uint32_t *session_id_out,
                               uint16_t *sequence_out);

/* MCU 回传 */
void CanComm_sendControl(const ControlOutput_t *out, const McuStatus_t *st,
                         uint16_t mcu_seq);
void CanComm_sendHeartbeat(const McuStatus_t *st, uint16_t mcu_seq,
                           uint16_t alive, bool buzzer_on);
void CanComm_sendDiag(const McuStatus_t *st);
void CanComm_sendE2eDiag(const McuStatus_t *st);
void CanComm_sendLinkStats(void);           /* 0x205 链路间隔诊断 */
void CanComm_sendSessionStatus(uint16_t ack, uint32_t session_id,
                               uint16_t sequence);
/* 0x302 注入响应：MCU 收到 0x301 命令并应用 Safety_applyInject 后回送。
 * 帧内容 = cmd/param 回显 + 当前 system_state/fault_level/fault_code + 自增 seq。
 * 测试脚本据此验证"注入生效"闭环。 */
void CanComm_sendInjectResponse(uint16_t cmd, uint16_t param,
                                const McuStatus_t *st);

/* 主源 ctrl 帧(0x101/0x102)链路存活期最大到达间隔 ms（饱和 255，诊断用） */
uint16_t CanComm_priCtrlMaxGap(void);

#endif /* CAN_COMM_H_ */
