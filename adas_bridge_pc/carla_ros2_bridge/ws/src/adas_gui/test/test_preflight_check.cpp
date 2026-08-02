// preflight_check.hpp 分级逻辑单测：只测 classify_*（纯逻辑，无系统调用）。
// 探测函数（probe_*/run_preflight）依赖真实文件系统/网络，不在此覆盖。
#include <gtest/gtest.h>

#include "preflight_check.hpp"

namespace {

using adas::gui::classify_can_adapter;
using adas::gui::classify_carla_executable;
using adas::gui::classify_direct_iface;
using adas::gui::classify_gpu_quality;
using adas::gui::classify_orin_reachable;
using adas::gui::PreflightItem;
using adas::gui::PreflightLevel;

TEST(PreflightCheck, CarlaExecutableMissingIsFail) {
  const auto item = classify_carla_executable(false, "/opt/carla/CarlaUE4.sh");
  EXPECT_EQ(item.level, PreflightLevel::Fail);
}

TEST(PreflightCheck, CarlaExecutablePresentIsOk) {
  const auto item = classify_carla_executable(true, "/opt/carla/CarlaUE4.sh");
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, NonNvidiaEpicLocalWindowIsWarn) {
  // 今日实测复现 3 次的崩溃组合：非 NVIDIA + Epic 画质 + 本地渲染窗口。
  const auto item = classify_gpu_quality("AMD", /*low_quality=*/false,
                                         /*render_offscreen=*/false);
  EXPECT_EQ(item.level, PreflightLevel::Warn);
}

TEST(PreflightCheck, NonNvidiaWithLowQualityIsOk) {
  const auto item = classify_gpu_quality("AMD", /*low_quality=*/true,
                                         /*render_offscreen=*/false);
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, NonNvidiaWithOffscreenIsOk) {
  const auto item = classify_gpu_quality("AMD", /*low_quality=*/false,
                                         /*render_offscreen=*/true);
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, NvidiaEpicLocalWindowIsOk) {
  const auto item = classify_gpu_quality("NVIDIA", /*low_quality=*/false,
                                         /*render_offscreen=*/false);
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, OrinUnreachableIsFail) {
  const auto item = classify_orin_reachable(false, "192.168.100.32");
  EXPECT_EQ(item.level, PreflightLevel::Fail);
}

TEST(PreflightCheck, OrinReachableIsOk) {
  const auto item = classify_orin_reachable(true, "192.168.100.32");
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, MissingDirectIfaceIsWarnNotFail) {
  // 跨网段仍可能靠组播发现凑合工作，不阻断启动。
  const auto item = classify_direct_iface("");
  EXPECT_EQ(item.level, PreflightLevel::Warn);
}

TEST(PreflightCheck, DirectIfaceFoundIsOk) {
  const auto item = classify_direct_iface("enx00e04c176a70");
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, CanAdapterSkippedForRos2Source) {
  const auto item = classify_can_adapter("ros2", /*device_present=*/false);
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, CanAdapterMissingForCanSourceIsWarn) {
  const auto item = classify_can_adapter("can", /*device_present=*/false);
  EXPECT_EQ(item.level, PreflightLevel::Warn);
}

TEST(PreflightCheck, CanAdapterPresentForCanSourceIsOk) {
  const auto item = classify_can_adapter("can", /*device_present=*/true);
  EXPECT_EQ(item.level, PreflightLevel::Ok);
}

TEST(PreflightCheck, ItemIsValueConstructibleForInlinedHelpers) {
  // 防御：分级函数返回 PreflightItem，确保 struct 字段就位不被打乱。
  PreflightItem item;
  EXPECT_EQ(item.level, PreflightLevel::Ok);
  EXPECT_TRUE(item.id.isEmpty());
  EXPECT_TRUE(item.label.isEmpty());
  EXPECT_TRUE(item.detail.isEmpty());
}

}  // namespace
