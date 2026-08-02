#include <gtest/gtest.h>

#include "health_model.hpp"

namespace adas::gui {

TEST(HealthModel, NamesCoverEveryState) {
  EXPECT_STREQ(health_state_name(HealthState::Unknown), "未知");
  EXPECT_STREQ(health_state_name(HealthState::Starting), "启动中");
  EXPECT_STREQ(health_state_name(HealthState::Healthy), "健康");
  EXPECT_STREQ(health_state_name(HealthState::Degraded), "降级");
  EXPECT_STREQ(health_state_name(HealthState::Fault), "故障");
  EXPECT_STREQ(health_state_name(HealthState::Offline), "离线");
}

TEST(HealthModel, HealthyIsGreen) {
  EXPECT_STREQ(health_state_color(HealthState::Healthy), "#2e7d32");
}

TEST(HealthModel, StartingAndDegradedAreYellow) {
  EXPECT_STREQ(health_state_color(HealthState::Starting), "#f9a825");
  EXPECT_STREQ(health_state_color(HealthState::Degraded), "#f9a825");
}

TEST(HealthModel, FaultIsRed) {
  EXPECT_STREQ(health_state_color(HealthState::Fault), "#c62828");
  EXPECT_EQ(health_severity(HealthState::Fault), 3);
}

TEST(HealthModel, OfflineIsAProblemButUnknownIsNot) {
  EXPECT_TRUE(health_is_problem(HealthState::Offline));
  EXPECT_FALSE(health_is_problem(HealthState::Unknown));
}

TEST(HealthModel, SeverityOrdersDegradation) {
  EXPECT_LT(health_severity(HealthState::Degraded),
            health_severity(HealthState::Offline));
  EXPECT_LT(health_severity(HealthState::Offline),
            health_severity(HealthState::Fault));
}

}  // namespace adas::gui
