#include <gtest/gtest.h>

#include "adas_global_planner/global_planner_core.hpp"

TEST(PlannerSession, RejectsEmptyAndNeverSwitchesAfterBinding) {
  std::string expected;
  EXPECT_FALSE(adas::planning::bind_or_accept_run_id(expected, ""));
  EXPECT_TRUE(expected.empty());
  EXPECT_TRUE(adas::planning::bind_or_accept_run_id(expected, "run-new"));
  EXPECT_EQ(expected, "run-new");
  EXPECT_FALSE(adas::planning::bind_or_accept_run_id(expected, ""));
  EXPECT_FALSE(adas::planning::bind_or_accept_run_id(expected, "run-old"));
  EXPECT_EQ(expected, "run-new");
}

namespace ap = adas::planning;

TEST(PlannerFailureRecovery, FirstAttemptFailureClearsRequest) {
  EXPECT_TRUE(ap::should_clear_request_on_failure(/*is_replan=*/false,
                                                   /*previous_route_valid=*/false));
  EXPECT_TRUE(ap::should_clear_request_on_failure(/*is_replan=*/false,
                                                   /*previous_route_valid=*/true));
}

TEST(PlannerFailureRecovery, ReplanFailureWithPreviousRouteRetainsRoute) {
  EXPECT_FALSE(ap::should_clear_request_on_failure(/*is_replan=*/true,
                                                    /*previous_route_valid=*/true));
}

TEST(PlannerFailureRecovery, ReplanFailureWithoutPreviousRouteStillClears) {
  EXPECT_TRUE(ap::should_clear_request_on_failure(/*is_replan=*/true,
                                                  /*previous_route_valid=*/false));
}
