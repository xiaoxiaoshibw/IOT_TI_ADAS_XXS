/*
 * asr_pro.c — ASR-PRO 语音识别模块 UART 驱动
 *
 * 借用 LINA 外设的 SCI 兼容异步模式(8N1)充当第二路 UART。
 * F280025C 只有一个硬件 SCI(SCIA，已分给串口屏)，故 ASR-PRO 走 LINA。
 *
 * 波特率计算（VCLK = SYSCLK/LINCLKDIV）：
 *   默认 LINCLKDIV=1 → VCLK=100MHz
 *   BaudRate = VCLK / (16 * (P + 1 + M/16))
 *   目标 115200：P = 53, M = 7 → 100000000/(16*(54+7/16)) = 115319 (0.10%)
 *
 * 协议帧（ASR-PRO → MCU）：
 *   0x0C 0x0D CMD [ARG...]   （帧头 + 命令 + 可选附加字节）
 */
#include "asr_pro.h"
#include "board.h"
#include "driverlib.h"
#include "device.h"

#define ASR_BAUD                115200UL

/* 波特率预分频值：VCLK=100MHz, 目标 115200
 * P = floor(100000000 / (115200 * 16)) - 1 = 53
 * M = round((100000000 / (115200 * 16) - 54) * 16) = 7 */
#define ASR_BAUD_PRESCALER      53U
#define ASR_BAUD_DIVIDER        7U

/* ASR-PRO 帧解析状态机 */
#define ASR_HDR0                0x0CU
#define ASR_HDR1                0x0DU
#define ASR_MAX_PAYLOAD         6U      /* 命令+附加字节最大长度 */

typedef enum
{
    ASR_WAIT_HDR0 = 0,
    ASR_WAIT_HDR1,
    ASR_WAIT_CMD,
    ASR_WAIT_ARG
} AsrRxState_t;

static AsrRxState_t s_rx_state;
static uint16_t s_rx_cnt;              /* 已收到的附加字节数 */
static uint16_t s_rx_cmd;
static uint16_t s_rx_arg;
static bool s_pending;
static bool s_present;
/* 调试计数：ASR-PRO 模块没响应/响应错时帮助定位是 UART 没收到、还是帧格式错、
 * 还是命令码不在 MCU 表里。s_rx_state 暴露当前解析状态机位置。 */
static uint16_t s_rx_byte_count;       /* UART 累计收到的字节数 */
static uint16_t s_rx_frame_count;      /* 累计收到的合法帧（CMD 字节到位） */
static uint16_t s_rx_hdr0_count;       /* 累计见到的 0x0C 帧头字节 */

/* ------------------------------------------------------------------ */
void AsrPro_init(void)
{
    /* GPIO22/23 pinmux 为 LINA TX/RX */
    GPIO_setPinConfig(BOARD_ASRPRO_TX_PINCFG);
    GPIO_setPinConfig(BOARD_ASRPRO_RX_PINCFG);
    GPIO_setPadConfig(BOARD_ASRPRO_TX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(BOARD_ASRPRO_RX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(BOARD_ASRPRO_RX_GPIO, GPIO_QUAL_ASYNC);

    /* LINA 模块使能（置位 SCIGCR0.RESET + TX/RX 引脚功能） */
    LIN_enableModule(LINA_BASE);

    /* 进入复态以配置寄存器 */
    LIN_enterSoftwareReset(LINA_BASE);

    /* SCI 模式（非 LIN 报文模式） */
    LIN_enableSCIMode(LINA_BASE);

    /* 8N1 */
    LIN_setSCICharLength(LINA_BASE, 8U);
    LIN_setSCIStopBits(LINA_BASE, LIN_SCI_STOP_ONE);
    LIN_disableSCIParity(LINA_BASE);

    /* 波特率 */
    LIN_setBaudRatePrescaler(LINA_BASE, ASR_BAUD_PRESCALER, ASR_BAUD_DIVIDER);

    /* 使能收发 */
    LIN_enableDataTransmitter(LINA_BASE);
    LIN_enableDataReceiver(LINA_BASE);

    /* 退出复态，开始工作 */
    LIN_exitSoftwareReset(LINA_BASE);

    /* 状态机初始化 */
    s_rx_state = ASR_WAIT_HDR0;
    s_rx_cnt = 0U;
    s_rx_cmd = 0U;
    s_rx_arg = 0U;
    s_pending = false;
    s_present = false;
    s_rx_byte_count = 0U;
    s_rx_frame_count = 0U;
    s_rx_hdr0_count = 0U;
}

/* ------------------------------------------------------------------ */
/* 暴露给 host 测试（tests/test_asr_parser_host.c）。 */
void AsrPro_resetForTest(void)
{
    s_rx_state = ASR_WAIT_HDR0;
    s_rx_cnt = 0U;
    s_rx_cmd = 0U;
    s_rx_arg = 0U;
    s_pending = false;
    s_present = false;
    s_rx_byte_count = 0U;
    s_rx_frame_count = 0U;
    s_rx_hdr0_count = 0U;
}
/* 暴露给 host 测试。非 static 让 host 单元测试可以单步灌帧验证状态机。 */
void asr_rx_feed_for_test(uint16_t b);
/* ------------------------------------------------------------------ */
static void asr_rx_feed(uint8_t b)
{
    s_rx_byte_count++;
    if (b == ASR_HDR0) { s_rx_hdr0_count++; }

    switch (s_rx_state)
    {
    case ASR_WAIT_HDR0:
        if (b == ASR_HDR0) { s_rx_state = ASR_WAIT_HDR1; }
        break;

    case ASR_WAIT_HDR1:
        s_rx_state = (b == ASR_HDR1) ? ASR_WAIT_CMD : ASR_WAIT_HDR0;
        break;

    case ASR_WAIT_CMD:
        s_rx_cmd = b;
        s_rx_arg = 0U;
        s_rx_cnt = 0U;
        s_pending = true;   /* 命令字节到达即视为一帧有效 */
        s_present = true;
        s_rx_frame_count++;
        s_rx_state = ASR_WAIT_ARG;
        break;

    case ASR_WAIT_ARG:
        /* 帧边界：捕获到 0~MAX_PAYLOAD 个附加字节后的 0x0C 头立即转入 HDR1。
         * 这是修复 P0#2 帧边界 bug 的关键——之前的二级 if-else 会在 cnt>0
         * 时吞掉所有后续字节，导致下一帧的 0x0C 不被识别、命令被吃掉成 arg。
         * 协议规定帧要么是 3 字节 (头头命令)，要么是 4+ 字节 (头头命令 arg...)，
         * 下一帧的 0x0C 头就是当前帧的最自然结束边界。 */
        if (b == ASR_HDR0)
        {
            s_rx_state = ASR_WAIT_HDR1;
            break;
        }
        if (s_rx_cnt == 0U) { s_rx_arg = b; }
        s_rx_cnt++;
        if (s_rx_cnt >= ASR_MAX_PAYLOAD)
        {
            s_rx_state = ASR_WAIT_HDR0;
        }
        break;

    default:
        s_rx_state = ASR_WAIT_HDR0;
        break;
    }
}

/* ------------------------------------------------------------------ */
void asr_rx_feed_for_test(uint16_t b)
{
    asr_rx_feed((uint8_t)b);
}

/* ------------------------------------------------------------------ */
void AsrPro_service(void)
{
    while (LIN_isSCIDataAvailable(LINA_BASE))
    {
        uint16_t raw = LIN_readSCICharNonBlocking(LINA_BASE, false);
        asr_rx_feed((uint8_t)(raw & 0xFFU));
    }
}

/* ------------------------------------------------------------------ */
bool AsrPro_pollCommand(uint16_t *cmd, uint16_t *arg)
{
    if (!s_pending) { return false; }
    *cmd = s_rx_cmd;
    *arg = s_rx_arg;
    s_pending = false;
    return true;
}

/* ------------------------------------------------------------------ */
void AsrPro_sendByte(uint16_t data)
{
    LIN_writeSCICharBlocking(LINA_BASE, data & 0xFFU);
}

/* ------------------------------------------------------------------ */
bool AsrPro_isPresent(void)
{
    return s_present;
}

/* 调试 accessor：暴露 ASR-PRO 解析器内部状态。OLED2 DBG 页用这几项定位
 * "完全没声音"（byte=0）/ "声音到了但帧头错"（byte>0 hdr0=0）/ "帧收到了
 * 但命令码不在 MCU 表里"（frame>0 last_cmd 不在 STOP/STATUS/SPEED/FLT） */
uint16_t AsrPro_byteCount(void)    { return s_rx_byte_count; }
uint16_t AsrPro_hdr0Count(void)    { return s_rx_hdr0_count; }
uint16_t AsrPro_frameCount(void)   { return s_rx_frame_count; }
uint16_t AsrPro_lastCmd(void)      { return s_rx_cmd; }
uint16_t AsrPro_rxState(void)      { return (uint16_t)s_rx_state; }
