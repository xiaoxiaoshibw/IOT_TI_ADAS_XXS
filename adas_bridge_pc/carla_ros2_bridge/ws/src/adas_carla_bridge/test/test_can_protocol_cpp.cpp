#include <array>
#include <cstdint>

#include "adas_carla_bridge/can_protocol.hpp"
#include "gtest/gtest.h"

namespace acb = adas::carla_bridge;

namespace {

acb::Frame finish(std::uint32_t id, std::array<std::uint8_t, 8> data) {
  data[7] = acb::frame_crc(id, data);
  return {id, data};
}

}  // namespace

TEST(CanProtocol, CrcAndSequenceContract) {
  constexpr std::array<std::uint8_t, 9> input{
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(acb::crc8(input.data(), input.size()), 0xA2U);
  EXPECT_TRUE(acb::sequence_forward(255U, 0U));
  EXPECT_TRUE(acb::sequence_forward(1U, 2U));
  EXPECT_FALSE(acb::sequence_forward(2U, 2U));
  EXPECT_FALSE(acb::sequence_forward(2U, 1U));
}

TEST(CanProtocol, GuardRequiresHealthAndThreeControls) {
  double now = 100.0;
  acb::McuFeedbackGuard guard(0.1, 0.2, 0.5, [&now]() { return now; });
  EXPECT_TRUE(guard.feed(finish(acb::kMcuHeartbeatId,
                                {2U, 1U, 0U, 0U, 0U, 1U, 10U, 0U})));
  EXPECT_TRUE(guard.feed(finish(acb::kMcuE2eDiagId,
                                {0U, 0U, 7U, 0U, 0U, 0U, 2U, 0U})));
  for (std::uint8_t sequence = 0U; sequence < 3U; ++sequence) {
    EXPECT_TRUE(guard.feed(finish(
        acb::kMcuControlId,
        {0x2CU, 0x01U, 0x50U, 0xFBU, 0U, 75U, sequence, 0U})));
  }
  const auto current = guard.current();
  EXPECT_FALSE(current.invalid_latched);
  EXPECT_DOUBLE_EQ(current.brake, 0.75);
  EXPECT_NEAR(current.steer, 0.1, 1.0e-12);
}

TEST(CanProtocol, DuplicateAndTimeoutFailClosed) {
  double now = 10.0;
  acb::McuFeedbackGuard guard(0.1, 0.2, 0.5, [&now]() { return now; });
  guard.feed(finish(acb::kMcuHeartbeatId,
                    {2U, 1U, 0U, 0U, 0U, 1U, 10U, 0U}));
  guard.feed(finish(acb::kMcuE2eDiagId,
                    {0U, 0U, 7U, 0U, 0U, 0U, 2U, 0U}));
  const auto control = finish(acb::kMcuControlId,
                              {0U, 0U, 0U, 0U, 0U, 0U, 1U, 0U});
  EXPECT_TRUE(guard.feed(control));
  EXPECT_FALSE(guard.feed(control));
  EXPECT_TRUE(guard.current().invalid_latched);
  now += 1.0;
  EXPECT_DOUBLE_EQ(guard.current().brake, 1.0);
}

TEST(CanProtocol, RejectsBadCrcAndProtocolVersion) {
  acb::McuFeedbackGuard guard;
  EXPECT_FALSE(guard.feed({acb::kMcuControlId, {}}));
  EXPECT_TRUE(guard.feed(finish(acb::kMcuE2eDiagId,
                                {0U, 0U, 7U, 0U, 0U, 0U, 3U, 0U})));
  EXPECT_TRUE(guard.current().invalid_latched);
}
