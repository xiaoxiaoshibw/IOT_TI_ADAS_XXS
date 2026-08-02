/* 迪文 DGUS III 触摸屏（DMG80480C043_02WTC）收发，走 SCIA。协议细节见 dgus_screen.h。
 *
 * §5 死代码门控：当 DGUS_ENABLE=0（默认；阶段 1 HIL 台架未配屏幕），
 * 仅暴露函数名 as no-op，源代码主体不进编译，所有相关 SCI/HW 资源
 * 不初始化、不占用 RAM。开启时按 adas_config.h 定义。 */
#include "dgus_screen.h"
#include "adas_config.h"
#include "board.h"
#include "driverlib.h"
#include "device.h"

#define DGUS_BAUD           115200UL
#define DGUS_HDR0           0x5AU
#define DGUS_HDR1           0xA5U
#define DGUS_CMD_WRITE      0x82U
#define DGUS_CMD_READ       0x83U
#define DGUS_MAX_WORDS      8U      /* 单帧最多写 8 个 16bit 字，够用且帧不至过长 */
#define DGUS_TX_BUF_LEN     (6U + 2U * DGUS_MAX_WORDS)

typedef enum
{
    RX_WAIT_HDR0 = 0,
    RX_WAIT_HDR1,
    RX_LEN,
    RX_CMD,
    RX_VP_HI,
    RX_VP_LO,
    RX_DLEN,
    RX_DATA_HI,
    RX_DATA_LO,
    RX_SKIP
} DgusRxState_t;

#if DGUS_ENABLE
static uint16_t s_tx_buf[DGUS_TX_BUF_LEN];
static uint16_t s_tx_len, s_tx_pos;
static bool s_tx_busy;

static DgusRxState_t s_rx_state;
static uint16_t s_rx_remain;        /* LEN 字段后仍待消费的字节数 */
static uint16_t s_rx_vp;
static uint16_t s_rx_dwords_remain;
static uint8_t  s_rx_hi_byte;

static bool s_pending;
static uint16_t s_pending_vp;
static uint16_t s_pending_val;
static bool s_present;
#endif /* DGUS_ENABLE */

#if DGUS_ENABLE
static void rx_feed(uint8_t b)
{
    switch (s_rx_state)
    {
    case RX_WAIT_HDR0:
        s_rx_state = (b == DGUS_HDR0) ? RX_WAIT_HDR1 : RX_WAIT_HDR0;
        break;
    case RX_WAIT_HDR1:
        s_rx_state = (b == DGUS_HDR1) ? RX_LEN : RX_WAIT_HDR0;
        break;
    case RX_LEN:
        s_rx_remain = b;
        s_rx_state = (s_rx_remain > 0U) ? RX_CMD : RX_WAIT_HDR0;
        break;
    case RX_CMD:
        s_rx_remain--;
        if ((b == DGUS_CMD_READ) && (s_rx_remain >= 4U))
        {
            s_rx_state = RX_VP_HI;
        }
        else
        {
            s_rx_state = (s_rx_remain > 0U) ? RX_SKIP : RX_WAIT_HDR0;
        }
        break;
    case RX_VP_HI:
        s_rx_vp = (uint16_t)((uint16_t)b << 8);
        s_rx_remain--;
        s_rx_state = RX_VP_LO;
        break;
    case RX_VP_LO:
        s_rx_vp = (uint16_t)(s_rx_vp | b);
        s_rx_remain--;
        s_rx_state = RX_DLEN;
        break;
    case RX_DLEN:
        s_rx_dwords_remain = b;
        s_rx_remain--;
        s_rx_state = ((s_rx_dwords_remain > 0U) && (s_rx_remain > 0U))
                     ? RX_DATA_HI : RX_WAIT_HDR0;
        break;
    case RX_DATA_HI:
        s_rx_hi_byte = b;
        s_rx_remain--;
        s_rx_state = RX_DATA_LO;
        break;
    case RX_DATA_LO:
        s_rx_remain--;
        if (!s_pending)   /* 只保留最早一帧，取走前不被新帧覆盖 */
        {
            s_pending_vp = s_rx_vp;
            s_pending_val = (uint16_t)(((uint16_t)s_rx_hi_byte << 8) | b);
            s_pending = true;
        }
        s_present = true;
        s_rx_dwords_remain--;
        s_rx_state = ((s_rx_dwords_remain > 0U) && (s_rx_remain > 0U))
                     ? RX_DATA_HI : RX_WAIT_HDR0;
        break;
    case RX_SKIP:
    default:
        if (s_rx_remain > 0U) { s_rx_remain--; }
        s_rx_state = (s_rx_remain > 0U) ? RX_SKIP : RX_WAIT_HDR0;
        break;
    }
}
#endif /* DGUS_ENABLE */

void Dgus_init(void)
{
#if DGUS_ENABLE
    GPIO_setPinConfig(BOARD_SCREEN_TX_PINCFG);
    GPIO_setPinConfig(BOARD_SCREEN_RX_PINCFG);
    GPIO_setPadConfig(BOARD_SCREEN_TX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(BOARD_SCREEN_RX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(BOARD_SCREEN_RX_GPIO, GPIO_QUAL_ASYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SCI_performSoftwareReset(SCIA_BASE);
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, DGUS_BAUD,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);

    s_tx_busy = false;
    s_tx_len = 0U;
    s_tx_pos = 0U;
    s_rx_state = RX_WAIT_HDR0;
    s_rx_remain = 0U;
    s_pending = false;
    s_present = false;
#else
    (void)0;
#endif
}

void Dgus_service(uint32_t now)
{
#if DGUS_ENABLE
    (void)now;
    if (s_tx_busy)
    {
        while ((s_tx_pos < s_tx_len) &&
               (SCI_getTxFIFOStatus(SCIA_BASE) != SCI_FIFO_TX16))
        {
            SCI_writeCharNonBlocking(SCIA_BASE, s_tx_buf[s_tx_pos]);
            s_tx_pos++;
        }
        if (s_tx_pos >= s_tx_len) { s_tx_busy = false; }
    }
    while (SCI_getRxFIFOStatus(SCIA_BASE) != SCI_FIFO_RX0)
    {
        rx_feed((uint8_t)SCI_readCharNonBlocking(SCIA_BASE));
    }
#else
    (void)now;
#endif
}

bool Dgus_writeVp(uint16_t vp_addr, const uint16_t *words, uint16_t count)
{
#if DGUS_ENABLE
    uint16_t i;
    if (s_tx_busy || (count == 0U) || (count > DGUS_MAX_WORDS)) { return false; }

    s_tx_buf[0] = DGUS_HDR0;
    s_tx_buf[1] = DGUS_HDR1;
    s_tx_buf[2] = (uint16_t)(3U + (2U * count));   /* LEN = cmd(1)+vp(2)+data */
    s_tx_buf[3] = DGUS_CMD_WRITE;
    s_tx_buf[4] = (uint16_t)(vp_addr >> 8);
    s_tx_buf[5] = (uint16_t)(vp_addr & 0xFFU);
    for (i = 0U; i < count; i++)
    {
        s_tx_buf[6U + (2U * i)] = (uint16_t)(words[i] >> 8);
        s_tx_buf[7U + (2U * i)] = (uint16_t)(words[i] & 0xFFU);
    }
    s_tx_len = (uint16_t)(6U + (2U * count));
    s_tx_pos = 0U;
    s_tx_busy = true;
    return true;
#else
    (void)vp_addr; (void)words; (void)count;
    return false;
#endif
}

bool Dgus_writeVpWord(uint16_t vp_addr, uint16_t value)
{
    return Dgus_writeVp(vp_addr, &value, 1U);
}

bool Dgus_pollTouch(uint16_t *vp_addr, uint16_t *value)
{
#if DGUS_ENABLE
    if (!s_pending) { return false; }
    *vp_addr = s_pending_vp;
    *value = s_pending_val;
    s_pending = false;
    return true;
#else
    (void)vp_addr; (void)value;
    return false;
#endif
}

bool Dgus_isPresent(void)
{
#if DGUS_ENABLE
    return s_present;
#else
    return false;
#endif
}
