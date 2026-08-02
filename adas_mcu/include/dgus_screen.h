/*
 * dgus_screen.h — 迪文(DWIN) DGUS III 触摸串口屏收发（DMG80480C043_02WTC）
 *
 * 走 SCIA（IO.29=TX，IO.28=RX，见 board.h），协议为迪文标准帧：
 *   写 VP：5A A5 LEN 82 VPH VPL DATA...
 *   读/回传：5A A5 LEN 83 VPH VPL DLEN DATA...（触摸控件配置"数据主动上传"后，
 *            屏在被触摸时会主动发送与读响应相同格式的帧，无需先发读命令）
 * 具体页面/控件的 VP 地址由 DGUS Tool 建工程时分配，不在本文件写死。
 */
#ifndef DGUS_SCREEN_H_
#define DGUS_SCREEN_H_

#include <stdint.h>
#include <stdbool.h>

/* 上电初始化 SCIA + 收发状态机复位 */
void Dgus_init(void);

/* 非阻塞轮询：推进一帧在途的发送、消费到达的字节。放在空闲背景任务里调用
 * （与 Oled2_update 同一路径），不占用 1kHz 控制 tick 预算。 */
void Dgus_service(uint32_t now);

/* 写单个 16bit VP。上一帧仍在发送时返回 false（不排队，调用方按需重试）。 */
bool Dgus_writeVpWord(uint16_t vp_addr, uint16_t value);

/* 写连续多个 16bit VP（count<=DGUS_MAX_WORDS，见 .c）。 */
bool Dgus_writeVp(uint16_t vp_addr, const uint16_t *words, uint16_t count);

/* 取走最近一次触摸/数据回传（先到先得，取走前不会被新帧覆盖）。
 * 返回 true 表示 vp_addr 与 value 已写入有效值且已取走；无待处理回传则返回 false。 */
bool Dgus_pollTouch(uint16_t *vp_addr, uint16_t *value);

/* 是否收到过合法回传帧（用于状态展示；上电前尚无回传时为 false）。 */
bool Dgus_isPresent(void);

#endif /* DGUS_SCREEN_H_ */
