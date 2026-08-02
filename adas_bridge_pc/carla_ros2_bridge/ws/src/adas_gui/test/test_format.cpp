#include <gtest/gtest.h>

#include <cstdio>

#include "format.hpp"

namespace ag = adas::gui;

TEST(GuiFormat, StateNamesMatchProtocol) {
  EXPECT_STREQ(ag::state_name(0), "INIT");
  EXPECT_STREQ(ag::state_name(2), "ACTIVE");
  EXPECT_STREQ(ag::state_name(4), "MRM");
  EXPECT_STREQ(ag::state_name(6), "FAILSAFE");
  EXPECT_STREQ(ag::state_name(7), "FAULT_LOCK");
  EXPECT_STREQ(ag::state_name(42), "UNKNOWN");
}

TEST(GuiFormat, AlertStatesAreRedNormalGreen) {
  EXPECT_STREQ(ag::state_color(2), "#2e7d32");
  EXPECT_STREQ(ag::state_color(3), "#f9a825");
  EXPECT_STREQ(ag::state_color(4), "#f9a825");
  EXPECT_STREQ(ag::state_color(5), "#c62828");
  EXPECT_STREQ(ag::state_color(6), "#c62828");
  EXPECT_STREQ(ag::state_color(7), "#c62828");
  EXPECT_STREQ(ag::state_color(0), "#616161");
}

TEST(GuiFormat, SourceNamesMatchProtocol) {
  EXPECT_STREQ(ag::source_name(1), "PRIMARY");
  EXPECT_STREQ(ag::source_name(2), "BACKUP");
  EXPECT_STREQ(ag::source_name(9), "MCU_WATCHDOG");
}

TEST(GuiFormat, NegativeAgeMeansNever) {
  EXPECT_EQ(ag::format_age(-1.0f), "never");
  EXPECT_EQ(ag::format_age(0.05f), "50 ms");
  EXPECT_EQ(ag::format_age(2.34f), "2.3 s");
}

TEST(GuiFormat, NavStateNamesMatchNavigationStatusMsg) {
  EXPECT_STREQ(ag::nav_state_name(0), "IDLE");
  EXPECT_STREQ(ag::nav_state_name(2), "PLANNING");
  EXPECT_STREQ(ag::nav_state_name(3), "DRIVING");
  EXPECT_STREQ(ag::nav_state_name(4), "ARRIVED");
  EXPECT_STREQ(ag::nav_state_name(5), "FAILED");
  EXPECT_STREQ(ag::nav_state_name(6), "CANCELED");
  EXPECT_STREQ(ag::nav_state_name(9), "UNKNOWN");
}

TEST(GuiFormat, ChainStateNamesMatchMsgConstants) {
  EXPECT_STREQ(ag::behavior_state_name(0), "车道保持");
  EXPECT_STREQ(ag::behavior_state_name(3), "超车中");
  EXPECT_STREQ(ag::behavior_state_name(6), "紧急");
  EXPECT_STREQ(ag::gate_source_name(0), "跟随器");
  EXPECT_STREQ(ag::gate_source_name(1), "AEB");
  EXPECT_STREQ(ag::gate_source_name(2), "内建停车");
  EXPECT_STREQ(ag::aeb_state_name(3), "紧急制动");
  EXPECT_STREQ(ag::safety_level_name(0), "OK");
  EXPECT_STREQ(ag::safety_level_name(3), "MRM(紧急)");
}

TEST(GuiFormat, FaultBitsMatchGatewayNaming) {
  EXPECT_EQ(ag::fault_bits_text(0), "none");
  EXPECT_EQ(ag::fault_bits_text(1U << 10), "fault_lock");
  EXPECT_EQ(ag::fault_bits_text((1U << 0) | (1U << 11)),
            "primary_timeout+can_bus_off");
  EXPECT_EQ(ag::fault_bits_text(1U << 13), "self_test_failed");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
