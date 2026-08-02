#include <gtest/gtest.h>

#include "map_camera.hpp"

namespace ag = adas::gui;

TEST(MapCamera, RoundTripIsIdentity) {
  ag::MapCamera camera;
  camera.set_viewport(800, 600);
  camera.fit(-100.0, -50.0, 100.0, 50.0);
  const auto screen = camera.world_to_screen(37.5, -12.25);
  const auto world = camera.screen_to_world(screen.x, screen.y);
  EXPECT_NEAR(world.x, 37.5, 1e-9);
  EXPECT_NEAR(world.y, -12.25, 1e-9);
}

TEST(MapCamera, ScreenYAxisPointsSouth) {
  ag::MapCamera camera;
  camera.set_viewport(800, 600);
  const auto north = camera.world_to_screen(0.0, 10.0);
  const auto south = camera.world_to_screen(0.0, -10.0);
  EXPECT_LT(north.y, south.y);  // 北在屏幕上方（y 更小）
  const auto east = camera.world_to_screen(10.0, 0.0);
  const auto west = camera.world_to_screen(-10.0, 0.0);
  EXPECT_GT(east.x, west.x);
}

TEST(MapCamera, FitCentersAndContainsBounds) {
  ag::MapCamera camera;
  camera.set_viewport(800, 600);
  camera.fit(0.0, 0.0, 200.0, 100.0);
  EXPECT_NEAR(camera.center_x(), 100.0, 1e-9);
  EXPECT_NEAR(camera.center_y(), 50.0, 1e-9);
  for (const auto& corner : {std::pair{0.0, 0.0}, std::pair{200.0, 100.0},
                             std::pair{0.0, 100.0}, std::pair{200.0, 0.0}}) {
    const auto screen = camera.world_to_screen(corner.first, corner.second);
    EXPECT_GE(screen.x, 0.0);
    EXPECT_LE(screen.x, 800.0);
    EXPECT_GE(screen.y, 0.0);
    EXPECT_LE(screen.y, 600.0);
  }
}

TEST(MapCamera, ZoomKeepsCursorAnchorFixed) {
  ag::MapCamera camera;
  camera.set_viewport(800, 600);
  camera.fit(-100.0, -100.0, 100.0, 100.0);
  const double cursor_x = 613.0, cursor_y = 155.0;
  const auto before = camera.screen_to_world(cursor_x, cursor_y);
  camera.zoom_at(cursor_x, cursor_y, 1.5);
  const auto after = camera.screen_to_world(cursor_x, cursor_y);
  EXPECT_NEAR(before.x, after.x, 1e-9);
  EXPECT_NEAR(before.y, after.y, 1e-9);
  camera.zoom_at(cursor_x, cursor_y, 0.25);
  const auto zoomed_out = camera.screen_to_world(cursor_x, cursor_y);
  EXPECT_NEAR(before.x, zoomed_out.x, 1e-9);
  EXPECT_NEAR(before.y, zoomed_out.y, 1e-9);
}

TEST(MapCamera, PanFollowsDragDirection) {
  ag::MapCamera camera;
  camera.set_viewport(800, 600);
  const auto anchor_before = camera.screen_to_world(400.0, 300.0);
  camera.pan_pixels(50.0, -30.0);  // 向右上拖
  const auto anchor_after = camera.screen_to_world(400.0, 300.0);
  EXPECT_LT(anchor_after.x, anchor_before.x);  // 内容右移 = 视口中心西移
  EXPECT_LT(anchor_after.y, anchor_before.y);  // 内容上移 = 视口中心南移
}

TEST(MapCamera, ZoomIsClampedToSaneRange) {
  ag::MapCamera camera;
  camera.set_viewport(800, 600);
  for (int i = 0; i < 100; ++i) camera.zoom_at(400.0, 300.0, 10.0);
  EXPECT_LE(camera.scale(), 50.0);
  for (int i = 0; i < 100; ++i) camera.zoom_at(400.0, 300.0, 0.1);
  EXPECT_GE(camera.scale(), 0.05);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
