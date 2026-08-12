#include <gtest/gtest.h>

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
}

}  // namespace
