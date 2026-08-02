/*
 * test_asr_parser_host.c — ASR-PRO 帧解析状态机的 host 单元测试。
 *
 * 覆盖 §2 修复：进入 ASR_WAIT_ARG 且 cnt>0 时，下一帧的 0x0C 必须
 * 转入 ASR_WAIT_HDR1，不能被二级 if-else 吞掉。速度查询后紧跟停止
 * 指令必须被识别为两个独立帧。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "asr_pro.h"

static void feed(uint8_t b) { asr_rx_feed_for_test(b); }

/* 检查"喂完一帧后是否能取到命令" */
static bool poll_command(uint16_t expected_cmd, uint16_t expected_arg)
{
    uint16_t cmd = 0U, arg = 0xFFFFU;
    if (!AsrPro_pollCommand(&cmd, &arg)) { return false; }
    return (cmd == expected_cmd) && (arg == expected_arg);
}

static void test_stop_command_single_frame(void)
{
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D); feed(0x01);  /* STOP = 0x01 */
    assert(poll_command(ASR_CMD_STOP, 0U));
    assert(!AsrPro_pollCommand(NULL, NULL));   /* no double-read */
}

static void test_query_speed_frame_with_arg(void)
{
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D); feed(0x0E); feed(0xED);  /* QUERY_SPEED + ARG 0xED */
    assert(poll_command(ASR_CMD_QUERY_SPEED, 0xEDU));
}

static void test_query_status_then_stop(void)
{
    /* 单帧无 arg 的命令，必须能继续接收下一帧 */
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D); feed(0x0A);  /* STATUS = 0x0A */
    assert(poll_command(ASR_CMD_QUERY_STATUS, 0U));
    feed(0x0C); feed(0x0D); feed(0x01);  /* STOP = 0x01 */
    assert(poll_command(ASR_CMD_STOP, 0U));
}

static void test_speed_query_then_stop_does_not_lose_stop(void)
{
    /* 这是 P0#2 修复的核心反例。
     * 旧实现里这段字节序列把 STOP 当成 SPEED 的 3 个额外 arg 字节吞掉。 */
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D); feed(0x0E); feed(0xED);  /* QUERY_SPEED + ARG */
    assert(poll_command(ASR_CMD_QUERY_SPEED, 0xEDU));
    feed(0x0C); feed(0x0D); feed(0x01);                /* STOP */
    assert(poll_command(ASR_CMD_STOP, 0U));
}

static void test_three_back_to_back_frames(void)
{
    /* poll 是"取走式"语义，未 poll 的帧会被下一帧的 CMD 字节覆盖
     * (s_pending 保持 true，s_rx_cmd 被新帧覆盖)。本测试在帧间 poll。 */
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D); feed(0x01);   /* STOP */
    assert(poll_command(ASR_CMD_STOP, 0U));
    feed(0x0C); feed(0x0D); feed(0x0A);   /* STATUS */
    assert(poll_command(ASR_CMD_QUERY_STATUS, 0U));
    feed(0x0C); feed(0x0D); feed(0x0E); feed(0xED);  /* SPEED + ARG */
    assert(poll_command(ASR_CMD_QUERY_SPEED, 0xEDU));
}

static void test_garbage_byte_in_hdr0_resyncs(void)
{
    AsrPro_resetForTest();
    feed(0xA5);  /* 噪声字节——应被忽略 */
    feed(0x0C); feed(0x0D); feed(0x01);   /* STOP */
    assert(poll_command(ASR_CMD_STOP, 0U));
}

static void test_noise_after_hdr0_returns_to_hdr0(void)
{
    AsrPro_resetForTest();
    feed(0x0C);      /* 头字节 0 */
    feed(0xFF);      /* 不是 0x0D → 回到 HDR0 */
    feed(0x0C); feed(0x0D); feed(0x01);  /* 重新开局：STOP */
    assert(poll_command(ASR_CMD_STOP, 0U));
}

static void test_max_payload_resets_state(void)
{
    /* 超过 ASR_MAX_PAYLOAD 字节后强制回到 HDR0。ASR_MAX_PAYLOAD=6
     * (命令 + 5 个 arg)，所以第 6 个 arg 进入时 state 应转回 HDR0。 */
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D); feed(0x0E);
    for (uint16_t i = 0U; i < 6U; i++) { feed((uint8_t)(0xA0U + i)); }
    assert(poll_command(ASR_CMD_QUERY_SPEED, 0xA0U));
    feed(0x0C); feed(0x0D); feed(0x01);   /* STOP 仍能被识别 */
    assert(poll_command(ASR_CMD_STOP, 0U));
}

static void test_partial_frame_is_held_not_lost(void)
{
    /* 仅到了 0x0C 0x0D 后还未到达 CMD 字节：不应有任何 pending。 */
    AsrPro_resetForTest();
    feed(0x0C); feed(0x0D);
    uint16_t cmd, arg;
    assert(!AsrPro_pollCommand(&cmd, &arg));
    feed(0x01);  /* STOP 字节补齐 */
    assert(AsrPro_pollCommand(&cmd, &arg));
    assert(cmd == ASR_CMD_STOP);
}

int main(void)
{
    test_stop_command_single_frame();
    test_query_speed_frame_with_arg();
    test_query_status_then_stop();
    test_speed_query_then_stop_does_not_lose_stop();   /* P0#2 核心反例 */
    test_three_back_to_back_frames();
    test_garbage_byte_in_hdr0_resyncs();
    test_noise_after_hdr0_returns_to_hdr0();
    test_max_payload_resets_state();
    test_partial_frame_is_held_not_lost();
    puts("asr parser host tests: PASS");
    return 0;
}
