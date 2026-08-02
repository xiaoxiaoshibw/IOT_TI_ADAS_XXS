/*
 * bench_responder.c — ESP32 CAN(TWAI) benchmark 应答器
 *
 * 与 F280025C 侧 f280025c_bench/bench_main.c **完全等价**的 echo 应答器，
 * 用于 PC 双通道基准 (can_benchmark/pc/bench.py) 的对照组。
 *
 * 接线：TWAI_TX = GPIO17 → 收发器 TXD；TWAI_RX = GPIO16 ← 收发器 RXD。
 *       （外接 SN65HVD230 / TJA1050 等 3.3V CAN 收发器，120Ω 终端）
 * 波特率：500 kbps，经典 CAN 2.0A 标准帧。
 *
 * 协议（见 pc/canframe.py）：收到 0x301 且 byte0==0xBE 且 CRC-8 正确时，
 *   按 byte2(load_units) 跑 load_units*256 次控制数学核，然后回 0x302。
 *
 * 为求公平：240MHz 满血、-O2、单精度 sinf/cosf（ESP32 有单精度 FPU），
 *   阻塞式 twai_receive → twai_transmit（IDF 惯用最低延迟路径，无额外排队）。
 */
#include <math.h>
#include <stdint.h>

#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TWAI_TX_GPIO   GPIO_NUM_17
#define TWAI_RX_GPIO   GPIO_NUM_16

#define BENCH_REQ_ID   0x301
#define BENCH_RESP_ID  0x302
#define BENCH_MARKER   0xBE
#define DEVICE_ID_ESP32 0xE5

static const char *TAG = "bench";

/* CRC-8 poly 0x31, MSB-first, init 0x00（与 crc8.c / canframe.py 一致） */
static uint8_t crc8(const uint8_t *d, int len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* 帧 CRC：覆盖 [id_lo, id_hi, byte0..byte6] */
static uint8_t crc8_frame(uint32_t id, const uint8_t *data7)
{
    uint8_t buf[9];
    buf[0] = (uint8_t)(id & 0xFF);
    buf[1] = (uint8_t)((id >> 8) & 0xFF);
    for (int i = 0; i < 7; i++) buf[2 + i] = data7[i];
    return crc8(buf, 9);
}

/* 控制数学核：每档 256 次 sin*cos + x^2（横向控制几何里典型的三角运算）。
 * 返回值折进响应，防止编译器把整段循环优化掉。 */
static volatile float g_sink;
static uint16_t bench_compute(uint32_t units)
{
    float acc = 0.0f;
    float x = 0.123f;
    uint32_t n = units * 256u;
    for (uint32_t i = 0; i < n; i++) {
        acc += sinf(x) * cosf(x) + x * x;
        x += 0.0009765625f;   /* 1/1024，float 精确可表示 */
    }
    g_sink = acc;
    return (uint16_t)((int32_t)(acc * 16.0f)) & 0xFFFF;
}

void app_main(void)
{
    twai_general_config_t g_cfg =
        TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = 32;
    g_cfg.tx_queue_len = 32;
    twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "TWAI 500k up. TX=GPIO%d RX=GPIO%d, 等待 0x301...",
             TWAI_TX_GPIO, TWAI_RX_GPIO);

    twai_message_t rx;
    for (;;) {
        if (twai_receive(&rx, portMAX_DELAY) != ESP_OK)
            continue;
        if (rx.identifier != BENCH_REQ_ID || rx.data_length_code < 8)
            continue;
        if (rx.data[0] != BENCH_MARKER)
            continue;
        if (rx.data[7] != crc8_frame(BENCH_REQ_ID, rx.data))
            continue;   /* CRC 不过，丢弃（与 MCU 侧 crc_ok 门控一致） */

        uint16_t comp = bench_compute(rx.data[2]);

        twai_message_t tx = {
            .identifier = BENCH_RESP_ID,
            .data_length_code = 8,
            .extd = 0, .rtr = 0,
        };
        tx.data[0] = BENCH_MARKER;
        tx.data[1] = rx.data[1];              /* 回显 seq */
        tx.data[2] = rx.data[2];              /* 回显 load */
        tx.data[3] = DEVICE_ID_ESP32;
        tx.data[4] = (uint8_t)(comp & 0xFF);
        tx.data[5] = (uint8_t)(comp >> 8);
        tx.data[6] = 0;
        tx.data[7] = crc8_frame(BENCH_RESP_ID, tx.data);

        twai_transmit(&tx, pdMS_TO_TICKS(10));
    }
}
