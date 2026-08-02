#ifndef ADAS_GUI__LANE_HIT_HPP_
#define ADAS_GUI__LANE_HIT_HPP_

// 世界坐标点到车道中心线折线集的最近点查询。纯逻辑，无 Qt 依赖（点类型
// 只要求提供 x()/y()，QPointF 与测试桩都满足）。用于：
//   · 悬停时预览目标会吸附到的车道点
//   · 点击超出吸附半径（全局规划器 snap_max_distance_m=8）时就地拒绝，
//     避免目标发出去后规划静默失败
#include <cmath>
#include <iterator>
#include <limits>

namespace adas::gui {

struct LaneHit {
  bool valid{false};
  double x{0.0};        // 最近车道点（线段上的投影点）
  double y{0.0};
  double distance{std::numeric_limits<double>::infinity()};
};

// 点 (px,py) 到线段 (ax,ay)-(bx,by) 的最近点。
inline void nearest_on_segment(double px, double py, double ax, double ay,
                               double bx, double by, LaneHit& best) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 0.0) {
    t = ((px - ax) * dx + (py - ay) * dy) / len2;
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  }
  const double cx = ax + t * dx;
  const double cy = ay + t * dy;
  const double dist = std::hypot(px - cx, py - cy);
  if (dist < best.distance) {
    best.valid = true;
    best.x = cx;
    best.y = cy;
    best.distance = dist;
  }
}

// Lanes: 可迭代，元素含 centerline（可迭代的点序列，点提供 x()/y()）。
template <typename Lanes>
LaneHit nearest_lane_point(const Lanes& lanes, double px, double py,
                           double max_distance_m) {
  LaneHit best;
  for (const auto& lane : lanes) {
    const auto& line = lane.centerline;
    auto it = std::begin(line);
    const auto end = std::end(line);
    if (it == end) continue;
    double ax = it->x(), ay = it->y();
    for (++it; it != end; ++it) {
      const double bx = it->x(), by = it->y();
      nearest_on_segment(px, py, ax, ay, bx, by, best);
      ax = bx;
      ay = by;
    }
  }
  if (best.distance > max_distance_m) best.valid = false;
  return best;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__LANE_HIT_HPP_
