#include <gtest/gtest.h>

#include <array>

#include "adas_can_gateway/protocol.hpp"

namespace acg = adas::can_gateway;

TEST(CanProtocol, StandardCrcVector) {
  const std::array<std::uint8_t, 9> input{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(acg::crc8(input.data(), input.size()), 0xA2U);
}

TEST(CanProtocol, MatchesMcuPythonGoldenFrames) {
  acg::CommandSnapshot command;
  command.target_steer_deg = 6.5;
  command.target_steer_rate_dps = 300.0;
  command.target_accel_mps2 = 1.2;
  command.target_speed_mps = 13.9;
  command.lateral_enable = true;
  command.longitudinal_enable = true;
  command.lka_active = true;
  command.acc_active = true;
  command.status_word = acg::kStatusControlEnable | acg::kStatusLateralEnable |
      acg::kStatusLongitudinalEnable | acg::kStatusLkaActive |
      acg::kStatusAccActive | (1U << 11U) | (1U << 12U) | (1U << 13U);
  command.system_mode = acg::SystemMode::kActive;
  command.soc_health = 1U;
  command.control_authority = true;

  acg::Encoder encoder;
  EXPECT_EQ(encoder.heartbeat(command).data,
            (std::array<std::uint8_t, 8>{0x02, 0x01, 0x00, 0x31,
                                         0x01, 0x00, 0x01, 0x59}));
  EXPECT_EQ(encoder.lateral(command).data,
            (std::array<std::uint8_t, 8>{0x8A, 0x02, 0xB8, 0x0B,
                                         0x03, 0x00, 0x00, 0x91}));
  EXPECT_EQ(encoder.longitudinal(command).data,
            (std::array<std::uint8_t, 8>{0xB0, 0x04, 0x6E, 0x05,
                                         0x00, 0x13, 0x00, 0x3A}));
  EXPECT_EQ(encoder.status(command).data,
            (std::array<std::uint8_t, 8>{0x1F, 0x38, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x6E}));
}

TEST(CanProtocol, FeedbackRejectsWrongCrcAndConflictingPedals) {
  acg::Frame frame{acg::kMcuControlId,
                   {0x8A, 0x02, 0x50, 0xFB, 0x00, 0x4B, 0x09, 0x00}};
  frame.data[7] = acg::frame_crc(frame.id, frame.data);
  const auto decoded = acg::decode_control_feedback(frame);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_DOUBLE_EQ(decoded->steer_deg, 6.5);
  EXPECT_DOUBLE_EQ(decoded->accel_mps2, -1.2);
  EXPECT_DOUBLE_EQ(decoded->brake, 0.75);

  frame.data[4] = 10U;
  frame.data[5] = 20U;
  frame.data[7] = acg::frame_crc(frame.id, frame.data);
  EXPECT_FALSE(acg::decode_control_feedback(frame).has_value());
  frame.data[7] ^= 0x01U;
  EXPECT_FALSE(acg::decode_control_feedback(frame).has_value());
}

TEST(CanProtocol, DecodesMcuDiagLittleEndianFaultCode) {
  // fault_code=0x2811 (fault_lock|can_bus_off|primary_timeout|estop_active)
  acg::Frame frame{acg::kMcuDiagId, {0x11, 0x28, 0x02, 0x03, 0x00, 0x05, 0x01, 0x00}};
  frame.data[7] = acg::frame_crc(frame.id, frame.data);
  const auto decoded = acg::decode_mcu_diag(frame);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->fault_code, 0x2811U);
  EXPECT_EQ(decoded->reset_reason, 0x02U);
  EXPECT_EQ(decoded->primary_timeouts, 3U);
  EXPECT_EQ(decoded->backup_timeouts, 0U);
  EXPECT_EQ(decoded->crc_errors, 5U);
  EXPECT_EQ(decoded->loop_overruns, 1U);

  frame.data[7] ^= 0x01U;
  EXPECT_FALSE(acg::decode_mcu_diag(frame).has_value());
  frame.id = acg::kMcuHeartbeatId;
  frame.data[7] = acg::frame_crc(frame.id, frame.data);
  EXPECT_FALSE(acg::decode_mcu_diag(frame).has_value());
}

TEST(CanProtocol, DecodesMcuE2eDiagVersionAndBuildFlags) {
  acg::Frame frame{acg::kMcuE2eDiagId, {0x01, 0x02, 0x07, 0x02, 0x00, 0x01, 0x03, 0x00}};
  frame.data[7] = acg::frame_crc(frame.id, frame.data);
  const auto decoded = acg::decode_mcu_e2e_diag(frame);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->primary_seq_errors, 1U);
  EXPECT_EQ(decoded->backup_seq_errors, 2U);
  EXPECT_EQ(decoded->proto_flags, 0x07U);
  EXPECT_EQ(decoded->can_recovery_count, 2U);
  EXPECT_EQ(decoded->self_test_mask, 0U);
  EXPECT_EQ(decoded->build_flags, 0x01U);       // ADAS_TEST_BUILD
  EXPECT_EQ(decoded->version, acg::kProtocolVersion);

  frame.data[7] ^= 0x01U;
  EXPECT_FALSE(acg::decode_mcu_e2e_diag(frame).has_value());
}

TEST(CanProtocol, SequenceWrapIsForward) {
  EXPECT_TRUE(acg::sequence_forward(255U, 0U));
  EXPECT_TRUE(acg::sequence_forward(4U, 5U));
  EXPECT_FALSE(acg::sequence_forward(5U, 5U));
  EXPECT_FALSE(acg::sequence_forward(5U, 4U));
}

// 序号持久化恢复（进程重启不得从 0 重置）：restore(+margin) 后发出的
// 首帧序号相对重启前最后一帧必须仍是 MCU 接受的前向增量。
TEST(CanProtocol, EncoderSequenceRestoreStaysForward) {
  acg::CommandSnapshot command;
  acg::Encoder encoder;
  std::uint8_t last_lateral = 0U;
  for (int i = 0; i < 300; ++i) last_lateral = encoder.lateral(command).data[5];

  const auto saved = encoder.sequences();
  EXPECT_EQ(saved.lateral, static_cast<std::uint8_t>(last_lateral + 1U));

  // 模拟重启：新 Encoder 从持久化状态 +64 余量恢复
  acg::Encoder restarted;
  acg::EncoderSequences seed = saved;
  seed.heartbeat = static_cast<std::uint8_t>(saved.heartbeat + 64U);
  seed.lateral = static_cast<std::uint8_t>(saved.lateral + 64U);
  seed.longitudinal = static_cast<std::uint8_t>(saved.longitudinal + 64U);
  seed.status = static_cast<std::uint8_t>(saved.status + 64U);
  seed.alive = static_cast<std::uint8_t>(saved.alive + 64U);
  restarted.restore(seed);

  const std::uint8_t first_lateral = restarted.lateral(command).data[5];
  EXPECT_TRUE(acg::sequence_forward(last_lateral, first_lateral));
  // 就算落盘后又发了 50 帧才崩溃，增量仍在 [1,127] 内
  EXPECT_TRUE(acg::sequence_forward(
      static_cast<std::uint8_t>(last_lateral + 50U), first_lateral));
}
