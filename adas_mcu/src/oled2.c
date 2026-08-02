/*
 * oled2.c — GPIO43/26 上的第二块 SSD1306 车辆数据屏。
 *
 * GPIO 开漏软件 I2C 采用短事务发送：每次 Oled2_update() 最多发送
 * 0x40 + 3 个像素，且调用只发生在 1 kHz 控制 tick 的空闲背景路径。
 * OLED 缺席、NACK、仲裁丢失或超时会退避后重试，不影响安全控制。
 */
#include "oled2.h"
#include "board.h"
#include "adas_can_protocol.h"
#include "adas_config.h"
#include "asr_pro.h"
#include "driverlib.h"
#include "device.h"
#include <string.h>

#define OLED2_ADDR_PRIMARY     0x3CU
#define OLED2_ADDR_ALTERNATE   0x3DU
#define OLED2_SCL_GPIO         43U
#define OLED2_SDA_GPIO         26U
#define OLED2_WIDTH            128U
#define OLED2_LINE_COUNT       9U
#define OLED2_REFRESH_MS       500U
#define OLED2_RETRY_MS         1000U
#define OLED2_TX_MAX           (OLED2_WIDTH + 1U)
#define OLED2_DATA_CHUNK       3U
#define OLED2_ASR_DISPLAY_MS   3000U   /* ASR 命令覆盖显示时长 */

typedef enum
{
    OLED2_PHASE_INIT_START = 0,
    OLED2_PHASE_CLEAR_CMD_START,
    OLED2_PHASE_CLEAR_DATA_START,
    OLED2_PHASE_REFRESH_WAIT,
    OLED2_PHASE_WRITE_CMD_START,
    OLED2_PHASE_WRITE_DATA_START,
    OLED2_PHASE_RETRY_WAIT,
    OLED2_PHASE_DISABLED
} Oled2Phase_t;

static Oled2Phase_t s_phase;
static uint16_t s_tx[OLED2_TX_MAX];
static uint16_t s_tx_len;
static uint16_t s_page;
static uint16_t s_column;
static uint16_t s_init_step;
static uint32_t s_refresh_ms;
static uint32_t s_retry_start_ms;
static uint16_t s_target_addr;
static uint16_t s_probe_index;
static bool s_swapped;
static bool s_present;
static char s_text[22];
/* 最近一次 ASR-PRO 命令：3 秒内覆盖显示 "ASR:STOP/STATUS/SPEED/FAULT"，过期
 * 后自动恢复车辆数据翻页。 */
static uint16_t s_asr_cmd;
static uint32_t s_asr_until_ms;

static uint16_t oled2_scl_gpio(void)
{
    return s_swapped ? OLED2_SDA_GPIO : OLED2_SCL_GPIO;
}

static uint16_t oled2_sda_gpio(void)
{
    return s_swapped ? OLED2_SCL_GPIO : OLED2_SDA_GPIO;
}

static void oled2_select_probe(uint16_t index)
{
    s_probe_index = index & 3U;
    s_swapped = (s_probe_index >= 2U);
    s_target_addr = ((s_probe_index & 1U) == 0U) ?
                    OLED2_ADDR_PRIMARY : OLED2_ADDR_ALTERNATE;
}

/* JTAG 烧录或 MCU 复位可能打断一笔 I2C 事务，而 OLED 没有同步断电。
 * 先以开漏 GPIO 送 9 个 SCL 和一个 STOP，释放控制器可能卡住的 SDA。 */
static void oled2_recover_bus(void)
{
    uint16_t i;
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setPadConfig(OLED2_SCL_GPIO,
                      GPIO_PIN_TYPE_OD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(OLED2_SDA_GPIO,
                      GPIO_PIN_TYPE_OD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(OLED2_SCL_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(OLED2_SDA_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_writePin(oled2_sda_gpio(), 1U);
    GPIO_writePin(oled2_scl_gpio(), 1U);
    DEVICE_DELAY_US(5U);
    for (i = 0U; i < 9U; i++)
    {
        GPIO_writePin(oled2_scl_gpio(), 0U);
        DEVICE_DELAY_US(5U);
        GPIO_writePin(oled2_scl_gpio(), 1U);
        DEVICE_DELAY_US(5U);
    }
    GPIO_writePin(oled2_scl_gpio(), 0U);
    GPIO_writePin(oled2_sda_gpio(), 0U);
    DEVICE_DELAY_US(5U);
    GPIO_writePin(oled2_scl_gpio(), 1U);
    DEVICE_DELAY_US(5U);
    GPIO_writePin(oled2_sda_gpio(), 1U);
    DEVICE_DELAY_US(5U);
}

static void oled2_panel_powerup_delay(void)
{
    uint16_t i;
    for (i = 0U; i < 10U; i++)
    {
        DEVICE_DELAY_US(10000U);
        Board_serviceWatchdog();
    }
}

static void oled2_configure_bus(void)
{
    I2C_disableModule(I2CA_BASE);
    oled2_recover_bus();
    oled2_panel_powerup_delay();
}

static const uint8_t *oled2_glyph(char c)
{
    static const uint8_t zero[5] = {0U, 0U, 0U, 0U, 0U};
    static const uint8_t digit[10][5] =
    {
        {0x3EU,0x51U,0x49U,0x45U,0x3EU},{0U,0x42U,0x7FU,0x40U,0U},
        {0x42U,0x61U,0x51U,0x49U,0x46U},{0x21U,0x41U,0x45U,0x4BU,0x31U},
        {0x18U,0x14U,0x12U,0x7FU,0x10U},{0x27U,0x45U,0x45U,0x45U,0x39U},
        {0x3CU,0x4AU,0x49U,0x49U,0x30U},{0x01U,0x71U,0x09U,0x05U,0x03U},
        {0x36U,0x49U,0x49U,0x49U,0x36U},{0x06U,0x49U,0x49U,0x29U,0x1EU}
    };
    static const uint8_t alpha[26][5] =
    {
        {0x7EU,0x11U,0x11U,0x11U,0x7EU},{0x7FU,0x49U,0x49U,0x49U,0x36U},
        {0x3EU,0x41U,0x41U,0x41U,0x22U},{0x7FU,0x41U,0x41U,0x22U,0x1CU},
        {0x7FU,0x49U,0x49U,0x49U,0x41U},{0x7FU,0x09U,0x09U,0x09U,0x01U},
        {0x3EU,0x41U,0x49U,0x49U,0x7AU},{0x7FU,0x08U,0x08U,0x08U,0x7FU},
        {0U,0x41U,0x7FU,0x41U,0U},{0x20U,0x40U,0x41U,0x3FU,0x01U},
        {0x7FU,0x08U,0x14U,0x22U,0x41U},{0x7FU,0x40U,0x40U,0x40U,0x40U},
        {0x7FU,0x02U,0x0CU,0x02U,0x7FU},{0x7FU,0x04U,0x08U,0x10U,0x7FU},
        {0x3EU,0x41U,0x41U,0x41U,0x3EU},{0x7FU,0x09U,0x09U,0x09U,0x06U},
        {0x3EU,0x41U,0x51U,0x21U,0x5EU},{0x7FU,0x09U,0x19U,0x29U,0x46U},
        {0x46U,0x49U,0x49U,0x49U,0x31U},{0x01U,0x01U,0x7FU,0x01U,0x01U},
        {0x3FU,0x40U,0x40U,0x40U,0x3FU},{0x1FU,0x20U,0x40U,0x20U,0x1FU},
        {0x7FU,0x20U,0x18U,0x20U,0x7FU},{0x63U,0x14U,0x08U,0x14U,0x63U},
        {0x03U,0x04U,0x78U,0x04U,0x03U},{0x61U,0x51U,0x49U,0x45U,0x43U}
    };
    static const uint8_t colon[5] = {0U,0x36U,0x36U,0U,0U};
    static const uint8_t dash[5]  = {0x08U,0x08U,0x08U,0x08U,0x08U};
    static const uint8_t dot[5]   = {0U,0x60U,0x60U,0U,0U};
    static const uint8_t pct[5]   = {0x23U,0x13U,0x08U,0x64U,0x62U};
    static const uint8_t equal[5] = {0x14U,0x14U,0x14U,0x14U,0x14U};

    if (c >= '0' && c <= '9') { return digit[(uint16_t)(c - '0')]; }
    if (c >= 'A' && c <= 'Z') { return alpha[(uint16_t)(c - 'A')]; }
    if (c == ':') { return colon; }
    if (c == '-') { return dash; }
    if (c == '.') { return dot; }
    if (c == '%') { return pct; }
    if (c == '=') { return equal; }
    return zero;
}

static char *oled2_put_str(char *p, const char *s)
{
    while (*s != '\0') { *p++ = *s++; }
    return p;
}

static char *oled2_put_u32(char *p, uint32_t value)
{
    char tmp[10];
    uint16_t n = 0U;
    do
    {
        tmp[n++] = (char)('0' + (uint16_t)(value % 10UL));
        value /= 10UL;
    } while (value != 0UL);
    while (n > 0U) { *p++ = tmp[--n]; }
    return p;
}

static char *oled2_put_hex2(char *p, uint16_t value)
{
    /* 固定两位十六进制：方便看命令码（0x01/0x0A/0x0E/0x0F）。 */
    static const char hex[] = "0123456789ABCDEF";
    *p++ = hex[(value >> 4) & 0x0FU];
    *p++ = hex[value & 0x0FU];
    return p;
}

static char *oled2_put_f1(char *p, float value)
{
    int32_t scaled;
    if (value < 0.0f)
    {
        *p++ = '-';
        value = -value;
    }
    scaled = (int32_t)(value * 10.0f + 0.5f);
    p = oled2_put_u32(p, (uint32_t)(scaled / 10L));
    *p++ = '.';
    *p++ = (char)('0' + (uint16_t)(scaled % 10L));
    return p;
}

static const char *oled2_dir_name(uint16_t direction)
{
    if (direction == DIR_DRIVE) { return "FWD"; }
    if (direction == DIR_REVERSE) { return "REV"; }
    return "NEU";
}

static void oled2_build_text(uint32_t now_ms, const ControlOutput_t *out,
                             const SocCommand_t *primary,
                             const McuStatus_t *status, bool can_healthy)
{
    char *p = s_text;
    uint32_t uptime_s = now_ms / 1000UL;
    if (uptime_s > 999999UL) { uptime_s = 999999UL; }
    memset(s_text, 0, sizeof(s_text));

    /* ASR-PRO 最近 3 秒有命令到达 → 覆盖翻页内容，让现场操作员/乘客立刻看到
     * 语音指令识别结果。过期自动回到正常车辆数据页。 */
    if ((s_asr_cmd != 0U) && ((now_ms - s_asr_until_ms) < OLED2_ASR_DISPLAY_MS))
    {
        p = oled2_put_str(p, "ASR:");
        switch (s_asr_cmd)
        {
        case ASR_CMD_STOP:
            p = oled2_put_str(p, "STOP -> MRM");
            break;
        case ASR_CMD_QUERY_STATUS:
            p = oled2_put_str(p, "STATUS -> CHIRP");
            break;
        case ASR_CMD_QUERY_SPEED:
            p = oled2_put_str(p, "SPEED -> TX 0x201");
            break;
        case ASR_CMD_FAULT_INJECT:
            p = oled2_put_str(p, "FAULT INJECT");
            break;
        default:
            p = oled2_put_str(p, "?");
            break;
        }
        while ((p - s_text) < 21) { *p++ = ' '; }
        *p = '\0';
        return;
    }

    switch (s_page)
    {
    case 0U:
        p = oled2_put_str(p, "== DRIVE DATA ==");
        break;
    case 1U:
        p = oled2_put_str(p, "TSPD:");
        p = oled2_put_f1(p, primary->target_speed_ms);
        p = oled2_put_str(p, " M/S");
        break;
    case 2U:
        p = oled2_put_str(p, "STR:");
        p = oled2_put_f1(p, out->final_steer_deg);
        p = oled2_put_str(p, " DEG");
        break;
    case 3U:
        p = oled2_put_str(p, "ACC:");
        p = oled2_put_f1(p, out->final_accel_ms2);
        p = oled2_put_str(p, " M/S2");
        break;
    case 4U:
        p = oled2_put_str(p, "THR:");
        p = oled2_put_u32(p, (uint32_t)(out->final_throttle * 100.0f + 0.5f));
        *p++ = '%';
        p = oled2_put_str(p, " BRK:");
        p = oled2_put_u32(p, (uint32_t)(out->final_brake * 100.0f + 0.5f));
        *p++ = '%';
        break;
    case 5U:
        p = oled2_put_str(p, "DIR:");
        p = oled2_put_str(p, oled2_dir_name(out->drive_dir));
        p = oled2_put_str(p, out->hazard ? " HAZ:ON" : " HAZ:OFF");
        break;
    case 6U:
        p = oled2_put_str(p, "UP:");
        p = oled2_put_u32(p, uptime_s);
        p = oled2_put_str(p, "S LD:");
        p = oled2_put_u32(p, status->loop_load_pct);
        *p++ = '%';
        break;
    case 7U:
        p = oled2_put_str(p, can_healthy ? "CAN:OK" : "CAN:OFF");
        p = oled2_put_str(p, " CRC:");
        p = oled2_put_u32(p, status->crc_err_cnt);
        break;
    default:
        /* 调试页：ASR-PRO 状态。byte=0=模块没发任何字节；hdr0=0 但 byte>0
         * = UART 收到了非帧头字节（说明波特率/接线有问题或 ASR-PRO 在发错格式）；
         * frame=0 but hdr0>0 = 帧头到位但 0x0D 没跟上；last_cmd 非 0x01/0x0A/
         * 0x0E/0x0F = MCU 命令表没收录这个码。 */
        p = oled2_put_str(p, "ASR B:");
        p = oled2_put_u32(p, AsrPro_byteCount());
        p = oled2_put_str(p, " H:");
        p = oled2_put_u32(p, AsrPro_hdr0Count());
        p = oled2_put_str(p, " F:");
        p = oled2_put_u32(p, AsrPro_frameCount());
        p = oled2_put_str(p, " C:");
        p = oled2_put_hex2(p, AsrPro_lastCmd());
        p = oled2_put_str(p, " S:");
        p = oled2_put_u32(p, AsrPro_rxState());
        break;
    }
    while ((p - s_text) < 21) { *p++ = ' '; }
    *p = '\0';
}

static void oled2_fill_page_command(uint16_t page)
{
    s_tx[0] = 0x00U;
    s_tx[1] = 0xB0U + page;
    s_tx[2] = 0x00U;
    s_tx[3] = 0x10U;
    s_tx_len = 4U;
}

static void oled2_delay(void)
{
    DEVICE_DELAY_US(3U);
}

static bool oled2_raise_scl(void)
{
    uint16_t guard = 20U;
    GPIO_writePin(oled2_scl_gpio(), 1U);
    while (GPIO_readPin(oled2_scl_gpio()) == 0U && guard > 0U)
    {
        guard--;
        oled2_delay();
    }
    return (guard > 0U);
}

static bool oled2_write_byte(uint16_t value)
{
    uint16_t bit;
    bool ack;
    for (bit = 0U; bit < 8U; bit++)
    {
        GPIO_writePin(oled2_sda_gpio(),
                      ((value & (uint16_t)(0x80U >> bit)) != 0U) ? 1U : 0U);
        oled2_delay();
        if (!oled2_raise_scl()) { return false; }
        oled2_delay();
        GPIO_writePin(oled2_scl_gpio(), 0U);
    }
    GPIO_writePin(oled2_sda_gpio(), 1U);
    oled2_delay();
    if (!oled2_raise_scl()) { return false; }
    oled2_delay();
    ack = (GPIO_readPin(oled2_sda_gpio()) == 0U);
    GPIO_writePin(oled2_scl_gpio(), 0U);
    return ack;
}

static void oled2_stop(void)
{
    GPIO_writePin(oled2_sda_gpio(), 0U);
    oled2_delay();
    (void)oled2_raise_scl();
    oled2_delay();
    GPIO_writePin(oled2_sda_gpio(), 1U);
    oled2_delay();
}

static bool oled2_tx(void)
{
    uint16_t i;
    GPIO_writePin(oled2_sda_gpio(), 1U);
    if (!oled2_raise_scl()) { return false; }
    oled2_delay();
    GPIO_writePin(oled2_sda_gpio(), 0U);
    oled2_delay();
    GPIO_writePin(oled2_scl_gpio(), 0U);
    if (!oled2_write_byte((uint16_t)(s_target_addr << 1U)))
    {
        oled2_stop();
        return false;
    }
    for (i = 0U; i < s_tx_len; i++)
    {
        if (!oled2_write_byte(s_tx[i]))
        {
            oled2_stop();
            return false;
        }
    }
    oled2_stop();
    return true;
}

typedef struct
{
    uint16_t length;
    uint16_t data[3U];
} Oled2InitStep_t;

static const Oled2InitStep_t s_init_sequence[] =
{
    {2U,{0x00U,0xAEU,0U}}, {3U,{0x00U,0xD5U,0x80U}},
    {3U,{0x00U,0xA8U,0x3FU}}, {3U,{0x00U,0xD3U,0x00U}},
    {2U,{0x00U,0x40U,0U}}, {3U,{0x00U,0x8DU,0x14U}},
    {3U,{0x00U,0x20U,0x02U}}, {2U,{0x00U,0xA1U,0U}},
    {2U,{0x00U,0xC8U,0U}}, {3U,{0x00U,0xDAU,0x12U}},
    {3U,{0x00U,0x81U,0x7FU}}, {3U,{0x00U,0xD9U,0xF1U}},
    {3U,{0x00U,0xDBU,0x40U}}, {2U,{0x00U,0xA4U,0U}},
    {2U,{0x00U,0xA6U,0U}}, {2U,{0x00U,0x2EU,0U}},
    {2U,{0x00U,0xAFU,0U}}
};

static bool oled2_tx_init_step(void)
{
    uint16_t i;
    const Oled2InitStep_t *step = &s_init_sequence[s_init_step];
    s_tx_len = step->length;
    for (i = 0U; i < s_tx_len; i++) { s_tx[i] = step->data[i]; }
    return oled2_tx();
}

static void oled2_fill_data_chunk(bool blank)
{
    uint16_t i;
    uint16_t count = OLED2_WIDTH - s_column;
    if (count > OLED2_DATA_CHUNK) { count = OLED2_DATA_CHUNK; }
    s_tx[0] = 0x40U;
    for (i = 0U; i < count; i++)
    {
        uint16_t column = s_column + i;
        uint16_t value = 0U;
        if (!blank && s_text[column / 6U] != '\0' && (column % 6U) < 5U)
        {
            value = oled2_glyph(s_text[column / 6U])[column % 6U];
        }
        s_tx[i + 1U] = value;
    }
    s_tx_len = count + 1U;
}

static void oled2_schedule_retry(uint32_t now_ms)
{
    if (!s_present)
    {
        oled2_select_probe(s_probe_index + 1U);
    }
    s_present = false;
    s_retry_start_ms = now_ms;
    s_phase = OLED2_PHASE_RETRY_WAIT;
}

static bool oled2_blocking_init(void)
{
    s_init_step = 0U;
    while (s_init_step <
           (uint16_t)(sizeof(s_init_sequence) / sizeof(s_init_sequence[0])))
    {
        if (!oled2_tx_init_step()) { return false; }
        s_init_step++;
        Board_serviceWatchdog();
    }
    return true;
}

void Oled2_init(void)
{
    uint16_t orientation;
    uint16_t address;
    oled2_select_probe(0U);
    s_tx_len = 0U;
    s_page = 0U;
    s_column = 0U;
    s_init_step = 0U;
    s_refresh_ms = 0U;
    s_retry_start_ms = 0U;
    s_present = false;
    oled2_configure_bus();

    s_asr_cmd = 0U;
    s_asr_until_ms = 0U;

    /* 同型号模块通常是 0x3C/0x3D，但现场无人时不能把非标准焊盘地址
     * 当作硬件故障。启动阶段在两种可能线序下扫描完整合法 7 位地址，
     * 每次尝试都喂狗；找到 ACK 后才发送 SSD1306 初始化。 */
    for (orientation = 0U; orientation < 2U && !s_present; orientation++)
    {
        s_swapped = (orientation != 0U);
        oled2_recover_bus();
        for (address = 0x08U; address <= 0x77U; address++)
        {
            s_target_addr = address;
            s_tx_len = 0U;
            if (oled2_tx())
            {
                s_present = oled2_blocking_init();
                if (s_present)
                {
                    if (address == OLED2_ADDR_PRIMARY)
                    {
                        s_probe_index = s_swapped ? 2U : 0U;
                    }
                    else if (address == OLED2_ADDR_ALTERNATE)
                    {
                        s_probe_index = s_swapped ? 3U : 1U;
                    }
                    break;
                }
            }
            Board_serviceWatchdog();
        }
    }
    if (!s_present) { oled2_select_probe(0U); }
    s_page = 0U;
    s_column = 0U;
    s_phase = s_present ? OLED2_PHASE_CLEAR_CMD_START :
                          OLED2_PHASE_RETRY_WAIT;
}

void Oled2_update(uint32_t now_ms, const ControlOutput_t *out,
                  const SocCommand_t *primary, const McuStatus_t *status,
                  bool can_healthy)
{
    if (s_phase == OLED2_PHASE_DISABLED) { return; }
    if (s_phase == OLED2_PHASE_RETRY_WAIT)
    {
        if ((now_ms - s_retry_start_ms) >= OLED2_RETRY_MS)
        {
            oled2_recover_bus();
            s_init_step = 0U;
            s_phase = OLED2_PHASE_INIT_START;
        }
        return;
    }

    switch (s_phase)
    {
    case OLED2_PHASE_INIT_START:
        if (!oled2_tx_init_step())
        {
            oled2_schedule_retry(now_ms);
            break;
        }
        s_init_step++;
        if (s_init_step >=
            (uint16_t)(sizeof(s_init_sequence) / sizeof(s_init_sequence[0])))
        {
            s_present = true;
            s_page = 0U;
            s_column = 0U;
            s_phase = OLED2_PHASE_CLEAR_CMD_START;
        }
        break;
    case OLED2_PHASE_CLEAR_CMD_START:
        oled2_fill_page_command(s_page);
        if (!oled2_tx()) { oled2_schedule_retry(now_ms); break; }
        s_column = 0U;
        s_phase = OLED2_PHASE_CLEAR_DATA_START;
        break;
    case OLED2_PHASE_CLEAR_DATA_START:
        oled2_fill_data_chunk(true);
        if (!oled2_tx()) { oled2_schedule_retry(now_ms); break; }
        s_column += s_tx_len - 1U;
        if (s_column >= OLED2_WIDTH)
        {
            s_page++;
            if (s_page >= OLED2_LINE_COUNT)
            {
                s_page = 0U;
                s_refresh_ms = now_ms;
                s_phase = OLED2_PHASE_REFRESH_WAIT;
            }
            else { s_phase = OLED2_PHASE_CLEAR_CMD_START; }
        }
        break;
    case OLED2_PHASE_REFRESH_WAIT:
        if ((now_ms - s_refresh_ms) >= OLED2_REFRESH_MS)
        {
            s_page = 0U;
            s_phase = OLED2_PHASE_WRITE_CMD_START;
        }
        break;
    case OLED2_PHASE_WRITE_CMD_START:
        oled2_fill_page_command(s_page);
        if (!oled2_tx()) { oled2_schedule_retry(now_ms); break; }
        s_column = 0U;
        oled2_build_text(now_ms, out, primary, status, can_healthy);
        s_phase = OLED2_PHASE_WRITE_DATA_START;
        break;
    case OLED2_PHASE_WRITE_DATA_START:
        oled2_fill_data_chunk(false);
        if (!oled2_tx()) { oled2_schedule_retry(now_ms); break; }
        s_column += s_tx_len - 1U;
        if (s_column >= OLED2_WIDTH)
        {
            s_page++;
            if (s_page >= OLED2_LINE_COUNT)
            {
                s_page = 0U;
                s_refresh_ms = now_ms;
                s_phase = OLED2_PHASE_REFRESH_WAIT;
            }
            else { s_phase = OLED2_PHASE_WRITE_CMD_START; }
        }
        break;
    default:
        oled2_schedule_retry(now_ms);
        break;
    }
}

bool Oled2_isPresent(void)
{
    return s_present;
}

void Oled2_setAsrCommand(uint16_t cmd, uint32_t now_ms)
{
    if (cmd == 0U) { return; }
    s_asr_cmd = cmd;
    /* 用 now_ms 自身作起点，相对差判定过期：避免 wrap（控制路径 now 来自
     * 同一个 1kHz tick，不会回退）+ 支持同一 ASR 命令连续触发自动续期。 */
    s_asr_until_ms = now_ms;
}
