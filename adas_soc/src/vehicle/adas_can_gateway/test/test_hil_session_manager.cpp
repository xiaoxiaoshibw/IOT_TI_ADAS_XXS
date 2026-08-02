#include <gtest/gtest.h>

#include <stdexcept>

#include "adas_can_gateway/hil_session_manager.hpp"

namespace adas::can_gateway {
namespace {

using adas::can_protocol::SessionAck;
using adas::can_protocol::SessionRequest;

// MCU 自驱授权：网关只观测 0x206，不在 READY 期主动发 PREPARE_ARM。MCU 自己
// 浸泡达成条件后上报 ARMED，网关据此切到 COMMIT_ACTIVE。
TEST(HilSessionManager, ColdStartReachesActiveByObservation) {
  HilSessionManager manager;
  manager.begin(0x12345678U);
  EXPECT_EQ(manager.request(), SessionRequest::kAnnounce);
  EXPECT_TRUE(manager.observe({SessionAck::kSessionAccepted, 0x12345678U, 1U}, 0.01));
  EXPECT_EQ(manager.request(), SessionRequest::kSyncStandby);
  // MCU 上报 READY：网关继续 SYNC_STANDBY（不再需要 PREPARE_ARM）。
  EXPECT_TRUE(manager.observe({SessionAck::kReady, 0x12345678U, 2U}, 0.02));
  EXPECT_EQ(manager.request(), SessionRequest::kSyncStandby);
  EXPECT_FALSE(manager.control_authorized(0.02));
  // MCU 自驱 ARMED。
  EXPECT_TRUE(manager.observe({SessionAck::kArmed, 0x12345678U, 3U}, 0.03));
  EXPECT_EQ(manager.request(), SessionRequest::kCommitActive);
  // COMMIT 后 MCU 回 ACTIVE，网关切到 ACTIVE keepalive 并授权。
  EXPECT_TRUE(manager.observe({SessionAck::kActive, 0x12345678U, 4U}, 0.04));
  EXPECT_EQ(manager.request(), SessionRequest::kActive);
  EXPECT_TRUE(manager.control_authorized(0.04));
}

// 控制流长时间断：MCU 进入 RECOVERY_REQUIRED，网关停止发 0x104。MCU 自愈后重新
// 上报 SESSION_ACCEPTED（同 session id），网关据此清除 recovery 闩锁并恢复握手。
// 无需任何 TCP/服务触发 REARM。
TEST(HilSessionManager, TimeoutRecoverBySessionAcceptedNoRearm) {
  HilSessionManager manager(0.1);
  manager.begin(10U);
  ASSERT_TRUE(manager.observe({SessionAck::kSessionAccepted, 10U, 1U}, 0.01));
  ASSERT_TRUE(manager.observe({SessionAck::kActive, 10U, 2U}, 0.02));
  ASSERT_TRUE(manager.control_authorized(0.02));
  manager.update(0.13);
  EXPECT_TRUE(manager.recovery_required());
  EXPECT_EQ(manager.ack(), SessionAck::kRecoveryRequired);
  EXPECT_EQ(manager.request(), SessionRequest::kNone);
  EXPECT_FALSE(manager.control_authorized(0.13));
  EXPECT_THROW((void)manager.next_frame(), std::logic_error);
  // MCU 自愈：重新上报同 id 的 SESSION_ACCEPTED。
  EXPECT_TRUE(manager.observe({SessionAck::kSessionAccepted, 10U, 3U}, 0.20));
  EXPECT_FALSE(manager.recovery_required());
  EXPECT_EQ(manager.request(), SessionRequest::kSyncStandby);
  // 随后 MCU 自驱到 ACTIVE，恢复授权。
  ASSERT_TRUE(manager.observe({SessionAck::kReady, 10U, 4U}, 0.21));
  ASSERT_TRUE(manager.observe({SessionAck::kArmed, 10U, 5U}, 0.22));
  ASSERT_TRUE(manager.observe({SessionAck::kActive, 10U, 6U}, 0.23));
  EXPECT_TRUE(manager.control_authorized(0.23));
}

TEST(HilSessionManager, RecoveryRequiredStopsControlAuthorization) {
  HilSessionManager manager;
  manager.begin(80U);
  ASSERT_TRUE(manager.observe({SessionAck::kActive, 80U, 1U}, 0.1));
  ASSERT_TRUE(manager.control_authorized(0.1));

  ASSERT_TRUE(manager.observe({SessionAck::kRecoveryRequired, 80U, 2U}, 0.2));
  EXPECT_TRUE(manager.recovery_required());
  EXPECT_FALSE(manager.control_authorized(0.2));
  EXPECT_EQ(manager.request(), SessionRequest::kNone);
  EXPECT_THROW((void)manager.next_frame(), std::logic_error);
}

TEST(HilSessionManager, RejectsForeignAckAndLatchesFatal) {
  HilSessionManager manager;
  manager.begin(7U);
  EXPECT_FALSE(manager.observe({SessionAck::kActive, 8U, 1U}, 0.0));
  EXPECT_TRUE(manager.observe({SessionAck::kFaultLock, 7U, 2U}, 0.1));
  EXPECT_TRUE(manager.fault_locked());
}

TEST(HilSessionManager, ForeignRecoveryLatchDoesNotKeepOldControl) {
  HilSessionManager manager;
  manager.begin(200U);
  // 上行 observed 的旧 session RECOVERY：闩锁 recovery，停止 0x104。
  EXPECT_FALSE(manager.observe({SessionAck::kRecoveryRequired, 100U, 9U}, 0.1));
  EXPECT_TRUE(manager.recovery_required());
  EXPECT_EQ(manager.request(), SessionRequest::kNone);
}

TEST(HilSessionManager, LocalMcuClearRestartsAnnouncementAfterFaultLock) {
  HilSessionManager manager;
  manager.begin(300U);
  ASSERT_TRUE(manager.observe({SessionAck::kFaultLock, 300U, 1U}, 0.1));
  ASSERT_TRUE(manager.fault_locked());
  ASSERT_EQ(manager.request(), SessionRequest::kNone);

  EXPECT_FALSE(manager.observe({SessionAck::kBootWait, 0U, 0U}, 0.2));
  EXPECT_FALSE(manager.fault_locked());
  EXPECT_FALSE(manager.recovery_required());
  EXPECT_EQ(manager.request(), SessionRequest::kAnnounce);
  EXPECT_EQ(manager.ack(), SessionAck::kBootWait);
  EXPECT_NO_THROW((void)manager.next_frame());
}

TEST(HilSessionProtocol, GoldenFrameAndStatusDecode) {
  const auto frame = encode_session_control(SessionRequest::kAnnounce,
                                            0x12345678U, 0U);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{
      3U, 1U, 0x78U, 0x56U, 0x34U, 0x12U, 0U, 0x4BU}));
  Frame status{adas::can_protocol::kCanIdMcuSessionStatus,
               {3U, 4U, 7U, 0U, 0U, 0U, 9U, 0U}};
  status.data[7] = frame_crc(status.id, status.data);
  const auto decoded = decode_session_status(status);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->ack, SessionAck::kActive);
  EXPECT_EQ(decoded->session_id, 7U);
}

}  // namespace
}  // namespace adas::can_gateway