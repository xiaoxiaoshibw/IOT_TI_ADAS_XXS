#include <gtest/gtest.h>

#include <QApplication>
#include <QImage>
#include <QPainter>

#include "map_view.hpp"
#include "navigation_goal.hpp"

namespace {

adas::gui::GuiLane make_lane(
    qint64 id, std::initializer_list<QPointF> points,
    std::initializer_list<adas::gui::GuiLaneConnection> outgoing = {}) {
  adas::gui::GuiLane lane;
  lane.id = id;
  for (const auto& point : points) lane.centerline << point;
  for (const auto& edge : outgoing) lane.outgoing << edge;
  return lane;
}

// P1.F: 旧"setObjects 截图"测试已删除——场景 actor 不再进入 GUI 渲染路径。
// 下面三件事用于锁定新契约：
//   1. clearForMapChange 必须清空路线且可重复调用。
//   2. setLayers 只接受不包含已删除的 kLayerObjects 的位图。

TEST(MapView, ClearForMapChangeResetsRouteAndVehicle) {
  int argc = 1;
  char app_name[] = "test_clear_for_map_change";
  char* argv[] = {app_name, nullptr};
  QApplication app(argc, argv);
  adas::gui::MapView view;
  view.resize(640, 480);
  view.setRoute({QPointF(0.0, 0.0), QPointF(10.0, 0.0)});
  EXPECT_EQ(view.routeSizeForTest(), 2);
  view.clearForMapChange();
  EXPECT_EQ(view.routeSizeForTest(), 0);
  // 二次调用必须幂等。
  view.clearForMapChange();
  EXPECT_EQ(view.routeSizeForTest(), 0);
}

TEST(MapView, LayerFlagsDoNotExposeScenarioObjects) {
  int argc = 1;
  char app_name[] = "test_layer_flags";
  char* argv[] = {app_name, nullptr};
  QApplication app(argc, argv);
  adas::gui::MapView view;
  EXPECT_EQ(view.layerFlags(), adas::gui::MapView::kDefaultLayers);
  // 任何调用 setLayers 都不得"复活"已删除的 kLayerObjects（已不再定义）。
  view.setLayers(adas::gui::MapView::kDefaultLayers);
  EXPECT_EQ(view.layerFlags(), adas::gui::MapView::kDefaultLayers);
}

TEST(NavigationGoal, FollowsOnlyDeclaredOutgoingConnections) {
  const QVector<adas::gui::GuiLane> lanes = {
      make_lane(10, {QPointF(0.0, 0.0), QPointF(20.0, 0.0)}, {{20, 0}}),
      make_lane(20, {QPointF(20.0, 0.0), QPointF(60.0, 0.0)}),
  };

  const auto goal = adas::gui::resolve_navigation_goal_ahead(
      lanes, 5.0, 0.0, 0.0, 40.0);

  ASSERT_TRUE(goal.valid) << goal.detail.toStdString();
  EXPECT_EQ(goal.start_lane_id, 10);
  EXPECT_EQ(goal.goal_lane_id, 20);
  EXPECT_NEAR(goal.x, 45.0, 1e-6);
  EXPECT_NEAR(goal.y, 0.0, 1e-6);
  EXPECT_NEAR(goal.covered_distance_m, 40.0, 1e-6);
}

TEST(NavigationGoal, DoesNotJumpAcrossGeometricallyTouchingUnlinkedLanes) {
  const QVector<adas::gui::GuiLane> lanes = {
      make_lane(10, {QPointF(0.0, 0.0), QPointF(20.0, 0.0)}),
      make_lane(20, {QPointF(20.0, 0.0), QPointF(100.0, 0.0)}),
  };

  const auto goal = adas::gui::resolve_navigation_goal_ahead(
      lanes, 5.0, 0.0, 0.0, 60.0);

  EXPECT_FALSE(goal.valid);
  EXPECT_EQ(goal.start_lane_id, 10);
}

TEST(NavigationGoal, PrefersStraightEdgeOverLaneChangeBranch) {
  const QVector<adas::gui::GuiLane> lanes = {
      make_lane(1, {QPointF(0.0, 0.0), QPointF(20.0, 0.0)},
                {{2, 3}, {3, 0}}),
      make_lane(2, {QPointF(20.0, 3.5), QPointF(100.0, 3.5)}),
      make_lane(3, {QPointF(20.0, 0.0), QPointF(100.0, 0.0)}),
  };

  const auto goal = adas::gui::resolve_navigation_goal_ahead(
      lanes, 5.0, 0.0, 0.0, 50.0);

  ASSERT_TRUE(goal.valid) << goal.detail.toStdString();
  EXPECT_EQ(goal.goal_lane_id, 3);
  EXPECT_NEAR(goal.x, 55.0, 1e-6);
  EXPECT_NEAR(goal.y, 0.0, 1e-6);
}

TEST(NavigationGoal, RejectsOppositeDirectionLane) {
  const QVector<adas::gui::GuiLane> lanes = {
      make_lane(7, {QPointF(100.0, 0.0), QPointF(0.0, 0.0)}),
  };

  const auto goal = adas::gui::resolve_navigation_goal_ahead(
      lanes, 10.0, 0.0, 0.0, 40.0);

  EXPECT_FALSE(goal.valid);
  EXPECT_EQ(goal.start_lane_id, 0);
}

TEST(MapView, SetLanesAtomicDoesNotDependOnSignalOrder) {
  int argc = 1;
  char app_name[] = "test_atomic_lane_graph";
  char* argv[] = {app_name, nullptr};
  QApplication app(argc, argv);
  adas::gui::MapView view;
  view.resize(640, 480);

  // 旧会话里残留的 route,验证切换到新地图会被清空。
  view.setRoute({QPointF(0.0, 0.0), QPointF(50.0, 0.0)});
  EXPECT_EQ(view.routeSizeForTest(), 2);

  // 第一次地图到达:不算 identity change,不触发 clear,但允许写入数据。
  const QVector<adas::gui::GuiLane> lanes_a = {
      make_lane(1, {QPointF(0.0, 0.0), QPointF(20.0, 0.0)}),
  };
  view.setLanesAtomic(lanes_a, QStringLiteral("Town04"),
                      QStringLiteral("hash-a"));
  EXPECT_EQ(view.routeSizeForTest(), 2);  // 同一会话的旧 route 仍保留
  // 重复发送同 identity:不触发 clear;route 仍保留。
  view.setLanesAtomic(lanes_a, QStringLiteral("Town04"),
                      QStringLiteral("hash-a"));
  EXPECT_EQ(view.routeSizeForTest(), 2);

  // 换地图身份:必须清空旧 route。
  const QVector<adas::gui::GuiLane> lanes_b = {
      make_lane(1, {QPointF(0.0, 0.0), QPointF(40.0, 0.0)}),
  };
  view.setLanesAtomic(lanes_b, QStringLiteral("Town05"),
                      QStringLiteral("hash-b"));
  EXPECT_EQ(view.routeSizeForTest(), 0);

  // 同 map_id 但 hash 改变:同样清空。
  view.setLanesAtomic(lanes_a, QStringLiteral("Town04"),
                      QStringLiteral("hash-c"));
  EXPECT_EQ(view.routeSizeForTest(), 0);
}

}  // namespace
