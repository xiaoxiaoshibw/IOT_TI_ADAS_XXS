#ifndef ADAS_GUI__MAP_CAMERA_HPP_
#define ADAS_GUI__MAP_CAMERA_HPP_

// 地图视口的世界坐标(ROS ENU, m) ↔ 屏幕像素变换。纯逻辑，无 Qt 依赖，
// 屏幕 y 轴向下，因此世界 y(北) 取反。

#include <algorithm>

namespace adas::gui {

struct ScreenPoint {
  double x{0.0};
  double y{0.0};
};

struct WorldPoint {
  double x{0.0};
  double y{0.0};
};

class MapCamera {
 public:
  void set_viewport(int width_px, int height_px) {
    width_ = std::max(1, width_px);
    height_ = std::max(1, height_px);
  }

  ScreenPoint world_to_screen(double wx, double wy) const {
    return {width_ / 2.0 + (wx - center_x_) * scale_,
            height_ / 2.0 - (wy - center_y_) * scale_};
  }

  WorldPoint screen_to_world(double sx, double sy) const {
    return {center_x_ + (sx - width_ / 2.0) / scale_,
            center_y_ - (sy - height_ / 2.0) / scale_};
  }

  // 以光标为锚点缩放：缩放前后光标下的世界点保持不动。
  void zoom_at(double sx, double sy, double factor) {
    const WorldPoint anchor = screen_to_world(sx, sy);
    scale_ = std::clamp(scale_ * factor, kMinScale, kMaxScale);
    const WorldPoint moved = screen_to_world(sx, sy);
    center_x_ += anchor.x - moved.x;
    center_y_ += anchor.y - moved.y;
  }

  void pan_pixels(double dx_px, double dy_px) {
    center_x_ -= dx_px / scale_;
    center_y_ += dy_px / scale_;
  }

  // 让世界包围盒完整可见并留边距。
  void fit(double min_x, double min_y, double max_x, double max_y,
           double margin_px = 24.0) {
    const double span_x = std::max(1.0, max_x - min_x);
    const double span_y = std::max(1.0, max_y - min_y);
    const double usable_w = std::max(1.0, width_ - 2.0 * margin_px);
    const double usable_h = std::max(1.0, height_ - 2.0 * margin_px);
    scale_ = std::clamp(std::min(usable_w / span_x, usable_h / span_y),
                        kMinScale, kMaxScale);
    center_x_ = (min_x + max_x) / 2.0;
    center_y_ = (min_y + max_y) / 2.0;
  }

  // 视角中心直接对准世界点（跟车模式），不改缩放。
  void center_on(double wx, double wy) {
    center_x_ = wx;
    center_y_ = wy;
  }

  double scale() const { return scale_; }
  double center_x() const { return center_x_; }
  double center_y() const { return center_y_; }

 private:
  static constexpr double kMinScale = 0.05;   // 20 m/px
  static constexpr double kMaxScale = 50.0;   // 2 cm/px
  double scale_{1.0};
  double center_x_{0.0};
  double center_y_{0.0};
  int width_{800};
  int height_{600};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__MAP_CAMERA_HPP_
