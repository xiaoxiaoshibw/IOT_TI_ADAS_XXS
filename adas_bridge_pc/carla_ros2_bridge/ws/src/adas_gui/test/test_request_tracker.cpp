#include <gtest/gtest.h>

#include "request_tracker.hpp"

namespace adas::gui {

TEST(RequestTracker, RejectsDuplicateOperation) {
  RequestTracker tracker;
  EXPECT_TRUE(tracker.begin("navigation.goal", "id-1", 100, 5000));
  EXPECT_FALSE(tracker.begin("navigation.goal", "id-2", 101, 5000));
  EXPECT_EQ(tracker.requestId("navigation.goal"), "id-1");
}

TEST(RequestTracker, LateResponseCannotUnlockNewRequest) {
  RequestTracker tracker;
  ASSERT_TRUE(tracker.begin("navigation.goal", "old", 0, 10));
  ASSERT_EQ(tracker.expire(10).size(), 1);
  ASSERT_TRUE(tracker.begin("navigation.goal", "new", 11, 10));
  EXPECT_FALSE(tracker.finish("navigation.goal", "old",
                              RequestState::Acknowledged));
  EXPECT_TRUE(tracker.pending("navigation.goal"));
  EXPECT_TRUE(tracker.finish("navigation.goal", "new",
                             RequestState::Acknowledged));
}

TEST(RequestTracker, TimeoutRestoresOperation) {
  RequestTracker tracker;
  ASSERT_TRUE(tracker.begin("fault.inject", "id", 100, 50));
  EXPECT_TRUE(tracker.expire(149).isEmpty());
  const auto expired = tracker.expire(150);
  ASSERT_EQ(expired.size(), 1);
  EXPECT_EQ(expired.front().state, RequestState::TimedOut);
  EXPECT_FALSE(tracker.pending("fault.inject"));
}

}  // namespace adas::gui
