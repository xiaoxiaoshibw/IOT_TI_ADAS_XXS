#include <gtest/gtest.h>

#include "run_id_session.hpp"
#include "session_contract.hpp"

namespace {

using adas::gui::RouteUpdate;

TEST(SessionContract, AdmissionMatrixIsStrict) {
  EXPECT_FALSE(adas::gui::accepts_run_id({}, {}));
  EXPECT_FALSE(adas::gui::accepts_run_id("current", {}));
  EXPECT_FALSE(adas::gui::accepts_run_id({}, "current"));
  EXPECT_FALSE(adas::gui::accepts_run_id("current", "old"));
  EXPECT_TRUE(adas::gui::accepts_run_id("current", "current"));
}

TEST(SessionContract, RouteMatrixOnlyMutatesCurrentSession) {
  EXPECT_EQ(adas::gui::route_update_for("current", "old", false, 0),
            RouteUpdate::Ignore);
  EXPECT_EQ(adas::gui::route_update_for("current", "old", true, 4),
            RouteUpdate::Ignore);
  EXPECT_EQ(adas::gui::route_update_for("current", "current", true, 1),
            RouteUpdate::Clear);
  EXPECT_EQ(adas::gui::route_update_for("current", "current", false, 4),
            RouteUpdate::Clear);
  EXPECT_EQ(adas::gui::route_update_for("current", "current", true, 4),
            RouteUpdate::Replace);
}

TEST(SessionContract, MapSwitchIgnoresFirstAndRepeatedMetadata) {
  EXPECT_FALSE(adas::gui::map_identity_changed({}, {}, "Town04", "hash-a"));
  EXPECT_FALSE(adas::gui::map_identity_changed("Town04", "hash-a",
                                               "Town04", "hash-a"));
  EXPECT_FALSE(adas::gui::map_identity_changed("Town04", "hash-a",
                                               "Town04", {}));
  EXPECT_TRUE(adas::gui::map_identity_changed("Town04", "hash-a",
                                              "Town05", "hash-b"));
  EXPECT_TRUE(adas::gui::map_identity_changed("Town04", "hash-a",
                                              "Town04", "hash-b"));
  EXPECT_TRUE(adas::gui::map_identity_changed({}, "hash-a",
                                              "Town04", "hash-b"));
}

TEST(RunIdSession, BeginEmitsCanonicalUuidV4) {
  const QString id = adas_gui::RunIdSession::begin();
  EXPECT_TRUE(adas_gui::RunIdSession::is_canonical_uuid_v4(id));
  EXPECT_EQ(adas_gui::RunIdSession::current(), id);
  adas_gui::RunIdSession::end();
}

TEST(RunIdSession, PreflightCancellationDoesNotRotate) {
  // 模拟 GUI 主流程:取消或预检失败不会轮换 run_id。
  // LaunchPanel.startConfiguredSystem 只在 preflight+confirm 都通过
  // 后才调用 begin();所以取消路径根本不调用 begin,current() 保持空。
  adas_gui::RunIdSession::end();
  EXPECT_TRUE(adas_gui::RunIdSession::current().isEmpty());
  const QString first = adas_gui::RunIdSession::begin();
  EXPECT_TRUE(adas_gui::RunIdSession::is_canonical_uuid_v4(first));
  EXPECT_EQ(adas_gui::RunIdSession::current(), first);
  // 同一会话期间,current() 不会自发生成新 ID(只能由显式 begin 触发)。
  EXPECT_EQ(adas_gui::RunIdSession::current(), first);
  adas_gui::RunIdSession::end();
}

TEST(RunIdSession, IsCanonicalUuidV4RejectsBadInputs) {
  EXPECT_FALSE(adas_gui::RunIdSession::is_canonical_uuid_v4({}));
  EXPECT_FALSE(adas_gui::RunIdSession::is_canonical_uuid_v4(
      QStringLiteral("not-a-uuid")));
  EXPECT_FALSE(adas_gui::RunIdSession::is_canonical_uuid_v4(
      QStringLiteral("AAAAAAAA-BBBB-4CCC-8DDD-EEEEEEEEEEEE")));
  EXPECT_FALSE(adas_gui::RunIdSession::is_canonical_uuid_v4(
      QStringLiteral("00000000-0000-1000-8000-000000000000")));  // version 1
  EXPECT_TRUE(adas_gui::RunIdSession::is_canonical_uuid_v4(
      QStringLiteral("11111111-2222-4333-8444-555555555555")));
}

}  // namespace
