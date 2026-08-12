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
#define OLED2_LINE_COUNT       8U
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
    OLED2_PHASE_REFRESH_CLEAR_CMD_START,
    OLED2_PHASE_REFRESH_CLEAR_DATA_START,
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
/* C28x 的 char 是 16 位；显式用 uint16_t 保存 ASCII 和中文字模 token，避免
 * UTF-8 多字节解析，也只为现场需要的少量汉字付出 Flash 空间。 */
static uint16_t s_text[24];
/* 最近一次 ASR-PRO 命令：3 秒内覆盖显示 "ASR:STOP/STATUS/SPEED/FAULT"，过期
 * 后自动恢复车辆数据翻页。 */
static uint16_t s_asr_cmd;
static uint32_t s_asr_until_ms;

enum
{
    ZH_ZHUANG = 0x100U, ZH_TAI, ZH_DAI, ZH_JI, ZH_YUN, ZH_XING,
    ZH_JIANG, ZH_LEVEL, ZH_ZUI, ZH_XIAO, ZH_FENG, ZH_XIAN,
    ZH_JIN, ZH_JI_URGENT, ZH_ZHI, ZH_DONG, ZH_SHI, ZH_XIAO_EFFECT,
    ZH_GU, ZH_ZHANG, ZH_SUO, ZH_DING, ZH_SU, ZH_DU, ZH_ZHUAN,
    ZH_XIANG, ZH_ZONG, ZH_XIAN_LINE, ZH_DUAN, ZH_KAI, ZH_QI,
    ZH_MI, ZH_MIAO, ZH_YU, ZH_YIN, ZH_TING, ZH_CHE
};

/* Noto Sans CJK SC 16 px 的受限中文字库。每字 16 列；前 16 字节是上半页，
 * 后 16 字节是下半页。顺序必须与上面的 token 连续一致。 */
static const uint16_t s_zh_glyphs[][32] =
{
    /* 状 */ {0x00U,0x0CU,0x18U,0x00U,0xFFU,0x00U,0x20U,0x20U,0x20U,0xFFU,0xFFU,0x20U,0x22U,0x2CU,0x28U,0x00U,0x00U,0x0CU,0x06U,0x03U,0xFFU,0x00U,0xC0U,0x30U,0x1CU,0x07U,0x01U,0x0EU,0x38U,0x60U,0xC0U,0x00U},
    /* 态 */ {0x00U,0x08U,0x08U,0x08U,0x88U,0xC8U,0xE8U,0x9FU,0x1BU,0x78U,0xC8U,0x88U,0x08U,0x08U,0x08U,0x00U,0x00U,0x62U,0x3DU,0x01U,0x3DU,0x7CU,0x40U,0x47U,0x4DU,0x48U,0x40U,0x70U,0x05U,0x39U,0x63U,0x00U},
    /* 待 */ {0x00U,0x18U,0x8CU,0xE6U,0x31U,0x40U,0x44U,0x44U,0x44U,0x44U,0x7FU,0x44U,0xC4U,0x44U,0x44U,0x00U,0x00U,0x03U,0x01U,0xFFU,0x00U,0x02U,0x02U,0x0AU,0x32U,0x22U,0x82U,0x82U,0xFFU,0x02U,0x02U,0x00U},
    /* 机 */ {0x00U,0x10U,0x10U,0xFFU,0xFFU,0x10U,0x10U,0x00U,0xFEU,0x02U,0x02U,0x02U,0xFEU,0x00U,0x00U,0x00U,0x00U,0x0CU,0x03U,0xFFU,0xFFU,0x01U,0xC2U,0x78U,0x0FU,0x00U,0x00U,0x00U,0xFFU,0x80U,0x80U,0x70U},
    /* 运 */ {0x00U,0x10U,0x33U,0x26U,0x04U,0x20U,0x22U,0x22U,0xE2U,0x62U,0x22U,0x22U,0x22U,0x22U,0x20U,0x00U,0x00U,0x61U,0x21U,0x1FU,0x20U,0x48U,0x48U,0x4EU,0x49U,0x48U,0x48U,0x4CU,0x4DU,0x4EU,0x58U,0x00U},
    /* 行 */ {0x00U,0x18U,0x8CU,0xC6U,0x73U,0x10U,0x40U,0x42U,0x42U,0x42U,0x42U,0x42U,0xC2U,0x42U,0x42U,0x00U,0x00U,0x03U,0x01U,0xFFU,0x00U,0x00U,0x00U,0x00U,0x00U,0x80U,0x80U,0x80U,0xFFU,0x00U,0x00U,0x00U},
    /* 降 */ {0x00U,0xFEU,0x02U,0x62U,0xDEU,0x02U,0x10U,0x98U,0x8CU,0xF7U,0x64U,0x64U,0xDCU,0x8CU,0x80U,0x00U,0x00U,0xFFU,0x00U,0x08U,0x0DU,0x07U,0x11U,0x12U,0x1EU,0x12U,0x12U,0xFFU,0x12U,0x12U,0x12U,0x00U},
    /* 级 */ {0x00U,0x60U,0x58U,0xC6U,0x61U,0x30U,0x02U,0x02U,0xFEU,0x82U,0x02U,0x72U,0x5EU,0xC0U,0xC0U,0x00U,0x00U,0x64U,0x27U,0x33U,0x12U,0x12U,0xE0U,0x3CU,0xC3U,0x43U,0x2EU,0x18U,0x3CU,0x63U,0xC0U,0x00U},
    /* 最 */ {0x00U,0x40U,0xC0U,0x5FU,0x55U,0x55U,0xD5U,0x55U,0x55U,0x55U,0x55U,0x55U,0x5FU,0x40U,0x40U,0x00U,0x00U,0x20U,0x3FU,0x35U,0x35U,0x15U,0x7FU,0x40U,0x41U,0x27U,0x3DU,0x19U,0x2DU,0x67U,0x40U,0x00U},
    /* 小 */ {0x00U,0x00U,0x80U,0xE0U,0x00U,0x00U,0x00U,0xFFU,0xFFU,0x00U,0x00U,0x00U,0x60U,0xC0U,0x00U,0x00U,0x00U,0x0CU,0x07U,0x01U,0x00U,0x80U,0x80U,0xFFU,0x7FU,0x00U,0x00U,0x00U,0x00U,0x03U,0x1EU,0x00U},
    /* 风 */ {0x00U,0x00U,0x80U,0xFFU,0x01U,0x19U,0x31U,0xC1U,0xC1U,0x39U,0x09U,0x01U,0xFFU,0x00U,0x00U,0x00U,0x00U,0x30U,0x1FU,0x01U,0x08U,0x0CU,0x06U,0x03U,0x01U,0x03U,0x0CU,0x08U,0x07U,0x3CU,0x30U,0x08U},
    /* 险 */ {0x00U,0xFEU,0x02U,0x62U,0xFEU,0x42U,0x60U,0x30U,0x58U,0x4CU,0x43U,0x4CU,0x58U,0x30U,0x60U,0x00U,0x00U,0xFFU,0x00U,0x08U,0x09U,0x07U,0x40U,0x46U,0x58U,0x40U,0x4FU,0x40U,0x78U,0x4EU,0x41U,0x00U},
    /* 紧 */ {0x00U,0x00U,0x1FU,0x80U,0xFFU,0xFFU,0xA0U,0xB1U,0x93U,0xD5U,0x49U,0x1DU,0x17U,0x23U,0x20U,0x00U,0x00U,0x00U,0x64U,0x24U,0x16U,0x06U,0x47U,0x45U,0x7CU,0x04U,0x14U,0x15U,0x27U,0x64U,0x00U,0x00U},
    /* 急 */ {0x00U,0x30U,0x18U,0x9CU,0x96U,0x93U,0x92U,0x92U,0x92U,0x9EU,0x96U,0x90U,0xF0U,0x00U,0x00U,0x00U,0x00U,0xC0U,0x62U,0x12U,0x02U,0xFAU,0x86U,0x8EU,0x8AU,0x82U,0xE2U,0x22U,0x1BU,0x30U,0xC0U,0x00U},
    /* 制 */ {0x00U,0x70U,0x4EU,0x48U,0x48U,0xFFU,0x48U,0x48U,0x48U,0x40U,0x00U,0xFCU,0x00U,0x00U,0xFFU,0x00U,0x00U,0x3EU,0x3EU,0x02U,0x02U,0xFFU,0x02U,0x22U,0x3EU,0x00U,0x00U,0x87U,0x80U,0x80U,0xFFU,0x00U},
    /* 动 */ {0x00U,0x22U,0x22U,0xE2U,0x22U,0x22U,0xA2U,0x22U,0x08U,0x08U,0xFFU,0x1FU,0x08U,0x08U,0xF8U,0x00U,0x00U,0x08U,0x0FU,0x09U,0x0CU,0x04U,0x07U,0x2CU,0x30U,0x1CU,0x07U,0x20U,0x20U,0x38U,0x1FU,0x00U},
    /* 失 */ {0x00U,0x40U,0x30U,0x1EU,0x0AU,0x08U,0x08U,0xFFU,0x7FU,0x08U,0x08U,0x08U,0x08U,0x08U,0x00U,0x00U,0x00U,0xC1U,0x41U,0x61U,0x21U,0x31U,0x0DU,0x07U,0x07U,0x0DU,0x11U,0x21U,0x61U,0x41U,0xC1U,0x00U},
    /* 效 */ {0x00U,0x88U,0x68U,0x08U,0x0FU,0x88U,0xA8U,0xC8U,0xC8U,0x78U,0xEFU,0x08U,0x08U,0xF8U,0x08U,0x00U,0x00U,0xC1U,0x61U,0x33U,0x1EU,0x0FU,0x30U,0x00U,0x40U,0x60U,0x33U,0x1EU,0x3FU,0x60U,0xC0U,0x00U},
    /* 故 */ {0x00U,0x10U,0x10U,0x10U,0xFFU,0x10U,0x10U,0xD0U,0x70U,0xFFU,0x09U,0x08U,0x88U,0xF8U,0x08U,0x00U,0x00U,0x7FU,0x21U,0x21U,0x21U,0x21U,0x3FU,0xC0U,0x40U,0x21U,0x1FU,0x1CU,0x37U,0x40U,0xC0U,0x00U},
    /* 障 */ {0x00U,0xFEU,0x02U,0x62U,0xDEU,0x02U,0x12U,0xD2U,0x5EU,0x52U,0x53U,0x52U,0x5EU,0xD2U,0x12U,0x00U,0x00U,0xFFU,0x00U,0x08U,0x09U,0x17U,0x10U,0x17U,0x15U,0x15U,0xFDU,0x15U,0x15U,0x17U,0x10U,0x00U},
    /* 锁 */ {0x00U,0x18U,0x26U,0xE7U,0x24U,0x24U,0x04U,0xE2U,0x2CU,0x28U,0xBFU,0x20U,0x28U,0xE6U,0x02U,0x00U,0x00U,0x02U,0x02U,0x7FU,0x62U,0x22U,0xA2U,0xDFU,0x40U,0x20U,0x1FU,0x20U,0x20U,0x5FU,0xC0U,0x00U},
    /* 定 */ {0x00U,0x3CU,0x04U,0x24U,0x24U,0x24U,0x24U,0xE7U,0xE7U,0x24U,0x24U,0x24U,0x24U,0x3CU,0x3CU,0x00U,0x00U,0xC0U,0x70U,0x1FU,0x1BU,0x30U,0x20U,0x7FU,0x7FU,0x44U,0x44U,0x44U,0x44U,0x40U,0x40U,0x00U},
    /* 速 */ {0x00U,0x82U,0x86U,0x8CU,0x00U,0x04U,0xF4U,0x14U,0x14U,0xFFU,0xFFU,0x14U,0x14U,0xF4U,0x04U,0x00U,0x00U,0x40U,0x20U,0x1FU,0x20U,0x70U,0x49U,0x4DU,0x47U,0x7FU,0x7FU,0x47U,0x4DU,0x49U,0x50U,0x00U},
    /* 度 */ {0x00U,0x00U,0xFCU,0x04U,0x24U,0x24U,0xFCU,0x24U,0x27U,0x24U,0x24U,0xFCU,0x24U,0x24U,0x24U,0x00U,0x00U,0xF0U,0x1FU,0x00U,0x84U,0xC4U,0x4DU,0x55U,0x65U,0x25U,0x65U,0x55U,0x4CU,0xC4U,0x80U,0x00U},
    /* 转 */ {0x00U,0x84U,0x74U,0x0FU,0xF4U,0x04U,0x04U,0x24U,0x24U,0xE4U,0x3EU,0x27U,0x24U,0x24U,0x24U,0x00U,0x00U,0x11U,0x11U,0x19U,0xFFU,0x09U,0x09U,0x00U,0x02U,0x13U,0x32U,0x72U,0x4EU,0x06U,0x02U,0x00U},
    /* 向 */ {0x00U,0x00U,0xF8U,0x08U,0x08U,0x88U,0x8CU,0x8FU,0x88U,0x88U,0x88U,0x08U,0x08U,0xF8U,0xF8U,0x00U,0x00U,0x00U,0xFFU,0x00U,0x00U,0x3FU,0x08U,0x08U,0x08U,0x08U,0x0FU,0x80U,0x80U,0xFFU,0x7FU,0x00U},
    /* 总 */ {0x00U,0x00U,0x00U,0xF0U,0x10U,0x16U,0x1CU,0x10U,0x10U,0x18U,0x1EU,0x12U,0xF0U,0x00U,0x00U,0x00U,0x00U,0x60U,0x38U,0x01U,0x79U,0xF9U,0x81U,0x87U,0x8DU,0x99U,0x81U,0xE1U,0x01U,0x18U,0x60U,0x00U},
    /* 线 */ {0x00U,0x60U,0x70U,0xCEU,0xC2U,0x30U,0x00U,0x10U,0x10U,0x1FU,0xFFU,0x90U,0x9AU,0x8EU,0x88U,0x00U,0x00U,0x44U,0x66U,0x25U,0x24U,0x32U,0x00U,0xC1U,0x41U,0x61U,0x2FU,0x78U,0xC8U,0x84U,0xE2U,0x00U},
    /* 断 */ {0x00U,0xFEU,0x00U,0x2EU,0xA0U,0xFFU,0xA0U,0x2EU,0x00U,0xFCU,0x44U,0x46U,0x42U,0xC2U,0x42U,0x40U,0x00U,0x7FU,0x20U,0x26U,0x21U,0x3FU,0x20U,0xE3U,0x70U,0x1FU,0x00U,0x00U,0x00U,0xFFU,0x00U,0x00U},
    /* 开 */ {0x00U,0x41U,0x41U,0x41U,0x41U,0xFFU,0x41U,0x41U,0x41U,0x41U,0xFFU,0xFFU,0x41U,0x41U,0x41U,0x00U,0x00U,0x20U,0x30U,0x18U,0x0FU,0x01U,0x00U,0x00U,0x00U,0x00U,0x7FU,0x7FU,0x00U,0x00U,0x00U,0x00U},
    /* 启 */ {0x00U,0x00U,0xFCU,0x94U,0x94U,0x94U,0x96U,0x92U,0x92U,0x92U,0x92U,0x92U,0x93U,0xF0U,0x00U,0x00U,0x00U,0xE0U,0x3FU,0x00U,0xFCU,0x44U,0x44U,0x44U,0x44U,0x44U,0x44U,0x44U,0x44U,0xFCU,0x00U,0x00U},
    /* 米 */ {0x00U,0x80U,0x86U,0x8CU,0xB0U,0x80U,0x80U,0xFFU,0xFFU,0x80U,0x80U,0xB0U,0x9CU,0x86U,0x80U,0x00U,0x00U,0x60U,0x20U,0x10U,0x08U,0x06U,0x03U,0xFFU,0xFFU,0x03U,0x06U,0x08U,0x10U,0x30U,0x60U,0x00U},
    /* 秒 */ {0x00U,0x24U,0x26U,0xFEU,0xFEU,0x22U,0x20U,0xE0U,0x38U,0x00U,0xFFU,0xFFU,0x00U,0x18U,0xE0U,0x00U,0x00U,0x0CU,0x07U,0xFFU,0xFFU,0x03U,0x00U,0xC1U,0x40U,0x44U,0x27U,0x33U,0x18U,0x0EU,0x00U,0x00U},
    /* 语 */ {0x00U,0x20U,0x21U,0xE6U,0x04U,0x80U,0x89U,0x89U,0xF9U,0x8FU,0x89U,0x89U,0xE9U,0xB9U,0x81U,0x00U,0x00U,0x00U,0x00U,0x3FU,0x10U,0x08U,0x7EU,0x7EU,0x22U,0x22U,0x22U,0x22U,0x22U,0x7EU,0x00U,0x00U},
    /* 音 */ {0x00U,0x40U,0x44U,0x44U,0x4CU,0x74U,0x44U,0x47U,0x47U,0x44U,0x74U,0x5CU,0x44U,0x44U,0x40U,0x00U,0x00U,0x00U,0x00U,0xFFU,0x49U,0x49U,0x49U,0x49U,0x49U,0x49U,0x49U,0x49U,0xFFU,0x00U,0x00U,0x00U},
    /* 停 */ {0x40U,0x60U,0x30U,0xFCU,0x07U,0x04U,0x74U,0x54U,0x54U,0x57U,0x57U,0x54U,0x54U,0x74U,0x04U,0x00U,0x00U,0x00U,0x00U,0xFFU,0x00U,0x07U,0x01U,0x85U,0x85U,0xFDU,0xFDU,0x05U,0x05U,0x05U,0x07U,0x00U},
    /* 车 */ {0x00U,0x08U,0x08U,0xC8U,0x78U,0x1CU,0x0FU,0x08U,0xE8U,0x08U,0x08U,0x08U,0x08U,0x08U,0x08U,0x00U,0x00U,0x08U,0x09U,0x09U,0x09U,0x09U,0x09U,0x09U,0xFFU,0x09U,0x09U,0x09U,0x09U,0x09U,0x08U,0x00U}
};

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
    static const uint8_t slash[5] = {0x20U,0x10U,0x08U,0x04U,0x02U};

    if (c >= '0' && c <= '9') { return digit[(uint16_t)(c - '0')]; }
    if (c >= 'A' && c <= 'Z') { return alpha[(uint16_t)(c - 'A')]; }
    if (c == ':') { return colon; }
    if (c == '-') { return dash; }
    if (c == '.') { return dot; }
    if (c == '%') { return pct; }
    if (c == '=') { return equal; }
    if (c == '/') { return slash; }
    return zero;
}

static uint16_t *oled2_put_str(uint16_t *p, const char *s)
{
    while (*s != '\0') { *p++ = (uint16_t)(*s++); }
    return p;
}

static uint16_t *oled2_put_token(uint16_t *p, uint16_t token)
{
    *p++ = token;
    return p;
}

static uint16_t *oled2_put_u32(uint16_t *p, uint32_t value)
{
    char tmp[10];
    uint16_t n = 0U;
    do
    {
        tmp[n++] = (char)('0' + (uint16_t)(value % 10UL));
        value /= 10UL;
    } while (value != 0UL);
    while (n > 0U) { *p++ = (uint16_t)tmp[--n]; }
    return p;
}

static uint16_t *oled2_put_f1(uint16_t *p, float value)
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

static uint16_t *oled2_put_state(uint16_t *p, uint16_t state)
{
    switch (state)
    {
    case SYS_MODE_INIT:
        return oled2_put_token(p, ZH_QI);
    case SYS_MODE_STANDBY:
        p = oled2_put_token(p, ZH_DAI);
        return oled2_put_token(p, ZH_JI);
    case SYS_MODE_ACTIVE:
        p = oled2_put_token(p, ZH_YUN);
        return oled2_put_token(p, ZH_XING);
    case SYS_MODE_DEGRADED:
        p = oled2_put_token(p, ZH_JIANG);
        return oled2_put_token(p, ZH_LEVEL);
    case SYS_MODE_MRM:
        p = oled2_put_token(p, ZH_ZUI);
        p = oled2_put_token(p, ZH_XIAO);
        p = oled2_put_token(p, ZH_FENG);
        return oled2_put_token(p, ZH_XIAN);
    case SYS_MODE_EMERGENCY_BRAKE:
        p = oled2_put_token(p, ZH_JIN);
        p = oled2_put_token(p, ZH_JI_URGENT);
        p = oled2_put_token(p, ZH_ZHI);
        return oled2_put_token(p, ZH_DONG);
    case SYS_MODE_FAILSAFE:
        p = oled2_put_token(p, ZH_SHI);
        return oled2_put_token(p, ZH_XIAO_EFFECT);
    default:
        p = oled2_put_token(p, ZH_GU);
        p = oled2_put_token(p, ZH_ZHANG);
        p = oled2_put_token(p, ZH_SUO);
        return oled2_put_token(p, ZH_DING);
    }
}

static void oled2_build_text(uint32_t now_ms, const ControlOutput_t *out,
                             const SocCommand_t *primary,
                             const McuStatus_t *status, bool can_healthy)
{
    uint16_t *p = s_text;
    uint16_t row = s_page / 2U;
    memset(s_text, 0, sizeof(s_text));

    /* ASR-PRO 最近 3 秒有命令到达 → 覆盖翻页内容，让现场操作员/乘客立刻看到
     * 语音指令识别结果。过期自动回到正常车辆数据页。 */
    if ((s_asr_cmd != 0U) && ((now_ms - s_asr_until_ms) < OLED2_ASR_DISPLAY_MS))
    {
        if (row != 1U) { return; }
        p = oled2_put_token(p, ZH_YU);
        p = oled2_put_token(p, ZH_YIN);
        *p++ = ':';
        switch (s_asr_cmd)
        {
        case ASR_CMD_STOP:
            p = oled2_put_token(p, ZH_TING);
            p = oled2_put_token(p, ZH_CHE);
            break;
        case ASR_CMD_QUERY_STATUS:
            p = oled2_put_token(p, ZH_ZHUANG);
            p = oled2_put_token(p, ZH_TAI);
            break;
        case ASR_CMD_QUERY_SPEED:
            p = oled2_put_token(p, ZH_SU);
            p = oled2_put_token(p, ZH_DU);
            break;
        case ASR_CMD_FAULT_INJECT:
            p = oled2_put_token(p, ZH_GU);
            p = oled2_put_token(p, ZH_ZHANG);
            break;
        default:
            p = oled2_put_str(p, "?");
            break;
        }
        *p = 0U;
        return;
    }

    switch (row)
    {
    case 0U:
        p = oled2_put_token(p, ZH_ZHUANG);
        p = oled2_put_token(p, ZH_TAI);
        *p++ = ':';
        p = oled2_put_state(p, status->system_state);
        break;
    case 1U:
        p = oled2_put_token(p, ZH_SU);
        p = oled2_put_token(p, ZH_DU);
        *p++ = ':';
        p = oled2_put_f1(p, primary->target_speed_ms);
        p = oled2_put_token(p, ZH_MI);
        *p++ = '/';
        p = oled2_put_token(p, ZH_MIAO);
        break;
    case 2U:
        p = oled2_put_token(p, ZH_ZHUAN);
        p = oled2_put_token(p, ZH_XIANG);
        *p++ = ':';
        p = oled2_put_f1(p, out->final_steer_deg);
        p = oled2_put_token(p, ZH_DU);
        break;
    default:
        if (!can_healthy)
        {
            p = oled2_put_token(p, ZH_ZONG);
            p = oled2_put_token(p, ZH_XIAN_LINE);
            *p++ = ':';
            p = oled2_put_token(p, ZH_DUAN);
            p = oled2_put_token(p, ZH_KAI);
        }
        else
        {
            p = oled2_put_token(p, ZH_ZHI);
            p = oled2_put_token(p, ZH_DONG);
            *p++ = ':';
            p = oled2_put_u32(p,
                (uint32_t)(out->final_brake * 100.0f + 0.5f));
            *p++ = '%';
        }
        break;
    }
    *p = 0U;
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

static uint16_t oled2_scaled_ascii_column(uint16_t token, uint16_t column,
                                           uint16_t half)
{
    const uint8_t *glyph;
    uint16_t value = 0U;
    uint16_t bit;
    if (column >= 5U || token > 0x7FU) { return 0U; }
    glyph = oled2_glyph((char)token);
    for (bit = 0U; bit < 8U; bit++)
    {
        uint16_t target_y = half * 8U + bit;
        uint16_t source_y = target_y / 2U;
        if (source_y < 7U &&
            ((glyph[column] & (uint16_t)(1U << source_y)) != 0U))
        {
            value |= (uint16_t)(1U << bit);
        }
    }
    return value;
}

static uint16_t oled2_render_column(uint16_t screen_column)
{
    uint16_t text_index = 0U;
    uint16_t text_column = 0U;
    uint16_t half = s_page & 1U;
    while (text_index < (uint16_t)(sizeof(s_text) / sizeof(s_text[0])) &&
           s_text[text_index] != 0U)
    {
        uint16_t token = s_text[text_index];
        uint16_t width = (token >= ZH_ZHUANG && token <= ZH_CHE) ? 16U : 6U;
        if (screen_column < text_column + width)
        {
            uint16_t local_column = screen_column - text_column;
            if (token >= ZH_ZHUANG && token <= ZH_CHE)
            {
                uint16_t glyph_index = token - ZH_ZHUANG;
                return s_zh_glyphs[glyph_index]
                                  [half * 16U + local_column];
            }
            return oled2_scaled_ascii_column(token, local_column, half);
        }
        text_column += width;
        text_index++;
    }
    return 0U;
}

static void oled2_fill_data_chunk(bool blank)
{
    uint16_t i;
    uint16_t count = OLED2_WIDTH - s_column;
    if (count > OLED2_DATA_CHUNK) { count = OLED2_DATA_CHUNK; }
    s_tx[0] = 0x40U;
    for (i = 0U; i < count; i++)
    {
        s_tx[i + 1U] = blank ? 0U : oled2_render_column(s_column + i);
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
                /* 首次整屏清零后立即写新页面，不让旧固件内容多停留一个
                 * refresh 周期。后续刷新仍按 OLED2_REFRESH_MS 节流。 */
                s_phase = OLED2_PHASE_WRITE_CMD_START;
            }
            else { s_phase = OLED2_PHASE_CLEAR_CMD_START; }
        }
        break;
    case OLED2_PHASE_REFRESH_WAIT:
        if ((now_ms - s_refresh_ms) >= OLED2_REFRESH_MS)
        {
            s_page = 0U;
            s_phase = OLED2_PHASE_REFRESH_CLEAR_CMD_START;
        }
        break;
    case OLED2_PHASE_REFRESH_CLEAR_CMD_START:
        oled2_fill_page_command(s_page);
        if (!oled2_tx()) { oled2_schedule_retry(now_ms); break; }
        s_column = 0U;
        s_phase = OLED2_PHASE_REFRESH_CLEAR_DATA_START;
        break;
    case OLED2_PHASE_REFRESH_CLEAR_DATA_START:
        oled2_fill_data_chunk(true);
        if (!oled2_tx()) { oled2_schedule_retry(now_ms); break; }
        s_column += s_tx_len - 1U;
        if (s_column >= OLED2_WIDTH)
        {
            /* 重新发页/列命令后才写内容，显式保证短文本右侧全为黑。 */
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
            else { s_phase = OLED2_PHASE_REFRESH_CLEAR_CMD_START; }
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
