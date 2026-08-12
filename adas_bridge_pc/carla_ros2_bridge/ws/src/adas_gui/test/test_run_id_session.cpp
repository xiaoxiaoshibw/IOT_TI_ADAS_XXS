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

TEST(RunIdSession, AdoptExternalCanonicalIdForObserverMode) {
  adas_gui::RunIdSession::end();
  const QString external = QStringLiteral(
      "12345678-1234-4abc-8def-1234567890ab");
  EXPECT_TRUE(adas_gui::RunIdSession::adopt(external));
  EXPECT_EQ(adas_gui::RunIdSession::current(), external);
  EXPECT_FALSE(adas_gui::RunIdSession::adopt(QStringLiteral("not-a-uuid")));
  EXPECT_EQ(adas_gui::RunIdSession::current(), external);
  adas_gui::RunIdSession::end();
}
