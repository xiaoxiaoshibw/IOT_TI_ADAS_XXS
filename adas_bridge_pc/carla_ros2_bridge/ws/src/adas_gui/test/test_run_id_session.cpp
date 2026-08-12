#include <gtest/gtest.h>

#include "run_id_session.hpp"

TEST(RunIdSession, StartsEmptyAndOnlyBeginRotates) {
  adas_gui::RunIdSession::end();
  EXPECT_TRUE(adas_gui::RunIdSession::current().isEmpty());
  const QString first = adas_gui::RunIdSession::begin();
  EXPECT_FALSE(first.isEmpty());
  EXPECT_EQ(adas_gui::RunIdSession::current(), first);
  EXPECT_EQ(adas_gui::RunIdSession::current(), first);
  const QString second = adas_gui::RunIdSession::begin();
  EXPECT_FALSE(second.isEmpty());
  EXPECT_NE(second, first);
  adas_gui::RunIdSession::end();
  EXPECT_TRUE(adas_gui::RunIdSession::current().isEmpty());
}
