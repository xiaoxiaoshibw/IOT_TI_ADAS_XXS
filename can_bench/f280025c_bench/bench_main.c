/*
 * bench_main.c — F280025C(C28x) CAN benchmark 应答器（独立最小固件）
 *
 * 与 ESP32 侧 esp32_responder/main/bench_responder.c **算法/字节级等价**的
 * echo 应答器，作为 PC 双通道基准 (can_benchmark/pc/bench.py) 的被测组。
 *
 * 复用 v3 协议 ID：RX 0x301(BENCH_REQ)、TX 0x302(BENCH_RESP)，详见 pc/canframe.py。
 * 请求 byte0 固定 0xBE，CRC-8 门控与主固件 crc_ok() 完全一致。
 *
 * 硬件：CANA，GPIO32=CANA_TX、GPIO33=CANA_RX（与主固件 board.h 一致），500 kbps。
 * 设计：100MHz C28x + TMU（单周期三角）+ FPU。紧循环轮询 RX 邮箱 → 计算核 → 回发，
 *       无 RTOS、无排队，展示硬实时确定性延迟。这是**独立 bench 固件**——
 *       烧它做对比，不与安全栈同时运行（对照 ESP32 也是独立应答器，公平）。
 *
 * 构建：见同目录 README.md（CCS 工程，链接 C2000Ware driverlib + device 支持包）。
 *       本文件不进主固件的量产/测试镜像，仅用于番外基准。
 */
#include <stdint.h>
#include "driverlib.h"
#include "device.h"

#define BENCH_REQ_ID    0x301U
#define BENCH_RESP_ID   0x302U
#define BENCH_MARKER    0xBEU
#define DEVICE_ID_C28X  0x28U
#define CAN_DLC         8U

#define MOBJ_RX_REQ     1U      /* 收 0x301 */
#define MOBJ_TX_RESP    2U      /* 发 0x302 */

/* CANA pinmux（与主固件 board.h 相同：GPIO32 TX / GPIO33 RX） */
#define CANTX_PINCFG    GPIO_32_CANA_TX
#define CANRX_PINCFG    GPIO_33_CANA_RX
#define CANTX_GPIO      32U
#define CANRX_GPIO      33U

/* CRC-8 poly 0x31, MSB-first, init 0x00。data 每元素低 8 位有效（C28x char=16bit）。 */
static uint16_t crc8(const uint16_t *d, uint16_t len)
{
    uint16_t crc = 0x00U;
    uint16_t i, b;
    for (i = 0U; i < len; i++)
    {
        crc ^= (d[i] & 0x00FFU);
        for (b = 0U; b < 8U; b++)
            crc = (crc & 0x80U) ? (((crc << 1) ^ 0x31U) & 0xFFU) : ((crc << 1) & 0xFFU);
    }
    return crc & 0xFFU;
}

/* 帧 CRC：[id_lo, id_hi, byte0..byte6] */
static uint16_t crc8_frame(uint32_t id, const uint16_t *data7)
{
    uint16_t buf[9];
    uint16_t i;
    buf[0] = (uint16_t)(id & 0xFFU);
    buf[1] = (uint16_t)((id >> 8) & 0xFFU);
    for (i = 0U; i < 7U; i++) buf[2U + i] = data7[i] & 0x00FFU;
    return crc8(buf, 9U);
}

/* 控制数学核：每档 256 次 sin*cos + x^2（横向控制几何典型三角运算）。
 * C28x 上 sinf/cosf 走 TMU 单周期，充分体现算力差距。返回值折进响应防优化。 */
#include <math.h>
static volatile float g_sink;
static uint16_t bench_compute(uint16_t units)
{
    float acc = 0.0f;
    float x = 0.123f;
    uint32_t n = (uint32_t)units * 256U;
    uint32_t i;
    for (i = 0U; i < n; i++)
    {
        acc += sinf(x) * cosf(x) + x * x;
        x += 0.0009765625f;    /* 1/1024 */
    }
    g_sink = acc;
    return (uint16_t)((int32_t)(acc * 16.0f)) & 0xFFFFU;
}

static void setupRx(uint16_t obj, uint32_t id)
{
    CAN_setupMessageObject(CANA_BASE, obj, id, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0U,
                           CAN_MSG_OBJ_NO_FLAGS, CAN_DLC);
}
static void setupTx(uint16_t obj, uint32_t id)
{
    CAN_setupMessageObject(CANA_BASE, obj, id, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0U,
                           CAN_MSG_OBJ_NO_FLAGS, CAN_DLC);
}

int main(void)
{
    Device_init();
    Device_initGPIO();

    /* CANA pinmux + 异步限定（CAN 输入不做同步限定） */
    GPIO_setPinConfig(CANTX_PINCFG);
    GPIO_setPinConfig(CANRX_PINCFG);
    GPIO_setQualificationMode(CANRX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(CANTX_GPIO, GPIO_QUAL_ASYNC);

    CAN_initModule(CANA_BASE);
    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 500000UL, 20U);
    CAN_enableRetry(CANA_BASE);
    setupRx(MOBJ_RX_REQ, BENCH_REQ_ID);
    setupTx(MOBJ_TX_RESP, BENCH_RESP_ID);
    CAN_startModule(CANA_BASE);

    uint16_t rx[CAN_DLC];
    uint16_t tx[CAN_DLC];

    for (;;)
    {
        /* 紧轮询 RX 邮箱：有新帧即刻处理，无 RTOS 调度抖动 */
        if (!CAN_readMessage(CANA_BASE, MOBJ_RX_REQ, rx))
            continue;
        if ((rx[0] & 0x00FFU) != BENCH_MARKER)
            continue;
        if ((rx[7] & 0x00FFU) != crc8_frame(BENCH_REQ_ID, rx))
            continue;   /* CRC 门控，与主固件 crc_ok() 一致 */

        uint16_t comp = bench_compute(rx[2] & 0x00FFU);

        tx[0] = BENCH_MARKER;
        tx[1] = rx[1] & 0x00FFU;          /* 回显 seq */
        tx[2] = rx[2] & 0x00FFU;          /* 回显 load */
        tx[3] = DEVICE_ID_C28X;
        tx[4] = comp & 0x00FFU;
        tx[5] = (comp >> 8) & 0x00FFU;
        tx[6] = 0U;
        tx[7] = crc8_frame(BENCH_RESP_ID, tx);
        CAN_sendMessage(CANA_BASE, MOBJ_TX_RESP, CAN_DLC, tx);
    }
}
