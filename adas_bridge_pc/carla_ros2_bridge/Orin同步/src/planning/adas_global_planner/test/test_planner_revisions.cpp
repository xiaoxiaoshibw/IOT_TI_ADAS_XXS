#include <gtest/gtest.h>

#include "adas_global_planner/global_planner_core.hpp"

namespace ap = adas::planning;

TEST(RouteRevision, StartsNonZeroAndMonotonic) {
  std::uint32_t v = 0U;
  v = ap::next_route_revision(v);
  EXPECT_EQ(v, 1U);
  v = ap::next_route_revision(v);
  EXPECT_EQ(v, 2U);
  v = ap::next_route_revision(v);
  EXPECT_EQ(v, 3U);
}

TEST(RouteRevision, OverflowWrapsToOneSkippingZero) {
  EXPECT_EQ(ap::next_route_revision(std::numeric_limits<std::uint32_t>::max()),
            1U);
}

TEST(RouteRevision, ZeroInputPromotesToOne) {
  EXPECT_EQ(ap::next_route_revision(0U), 1U);
  EXPECT_NE(ap::next_route_revision(0U), 0U);
}
