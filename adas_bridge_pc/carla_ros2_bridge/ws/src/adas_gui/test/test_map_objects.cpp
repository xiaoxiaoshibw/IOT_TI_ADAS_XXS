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

TEST(MapObjects, SnapshotIsStoredAndLayerDefaultsOff) {
  int argc = 1;
  char app_name[] = "test_map_objects";
  char* argv[] = {app_name, nullptr};
  QApplication app(argc, argv);
  adas::gui::MapView view;
  view.resize(640, 480);
  adas::gui::GuiLane lane;
  lane.centerline << QPointF(-20.0, 0.0) << QPointF(100.0, 0.0);
  view.setLanes({lane}, QStringLiteral("Town04"));

  QVector<adas::gui::GuiMapObject> objects;
  for (quint32 id = 1; id <= 20; ++id) {
    adas::gui::GuiMapObject object;
    object.id = id;
    object.classification = 1;
    object.x = static_cast<double>(id) * 3.0;
    object.y = (id % 2 == 0) ? 3.7 : -3.7;
    objects.push_back(object);
  }
  view.setObjects(objects);
  EXPECT_EQ(view.objectCount(), 20);
  EXPECT_EQ(view.layerFlags() & adas::gui::MapView::kLayerObjects, 0U);

  view.setLayers(adas::gui::MapView::kDefaultLayers |
                 adas::gui::MapView::kLayerObjects);
  QImage image(view.size(), QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  view.render(&painter);
  EXPECT_FALSE(image.isNull());
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

}  // namespace
