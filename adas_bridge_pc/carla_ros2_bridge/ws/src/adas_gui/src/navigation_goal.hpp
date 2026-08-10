#ifndef ADAS_GUI__NAVIGATION_GOAL_HPP_
#define ADAS_GUI__NAVIGATION_GOAL_HPP_

#include <QHash>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

#include "map_view.hpp"

namespace adas::gui {

struct ResolvedNavigationGoal {
  bool valid{false};
  double x{0.0};
  double y{0.0};
  double covered_distance_m{0.0};
  qint64 start_lane_id{0};
  qint64 goal_lane_id{0};
  QString detail;
};

namespace navigation_detail {

inline double wrap_angle(double value) {
  while (value > M_PI) value -= 2.0 * M_PI;
  while (value < -M_PI) value += 2.0 * M_PI;
  return value;
}

inline double distance(const QPointF& a, const QPointF& b) {
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

struct LaneProjection {
  int lane_index{-1};
  int segment_index{-1};
  double t{0.0};
  double distance_m{std::numeric_limits<double>::infinity()};
  double heading_error_rad{M_PI};
  QPointF point;
};

inline LaneProjection nearest_forward_lane(const QVector<GuiLane>& lanes,
                                           double x, double y,
                                           double yaw_rad) {
  LaneProjection best;
  double best_score = std::numeric_limits<double>::infinity();
  const QPointF query(x, y);
  for (int lane_i = 0; lane_i < lanes.size(); ++lane_i) {
    const auto& line = lanes[lane_i].centerline;
    for (int segment = 0; segment + 1 < line.size(); ++segment) {
      const QPointF a = line[segment];
      const QPointF b = line[segment + 1];
      const double dx = b.x() - a.x();
      const double dy = b.y() - a.y();
      const double length_sq = dx * dx + dy * dy;
      if (length_sq < 1e-8) continue;
      const double raw_t = ((query.x() - a.x()) * dx +
                            (query.y() - a.y()) * dy) / length_sq;
      const double t = std::clamp(raw_t, 0.0, 1.0);
      const QPointF projected(a.x() + t * dx, a.y() + t * dy);
      const double gap = distance(query, projected);
      const double heading_error = std::abs(
          wrap_angle(std::atan2(dy, dx) - yaw_rad));
      // 相邻反向车道可能几何上更近；100° 以上直接拒绝，剩余误差作为
      // 软惩罚参与选举。
      if (heading_error > 1.75) continue;
      const double score = gap + 4.0 * heading_error;
      if (score < best_score) {
        best_score = score;
        best = {lane_i, segment, t, gap, heading_error, projected};
      }
    }
  }
  return best;
}

inline int choose_successor(const QVector<GuiLane>& lanes,
                            const QHash<qint64, int>& index_by_id,
                            const GuiLane& current, qint64 previous_id,
                            const QSet<qint64>& visited) {
  if (current.centerline.size() < 2) return -1;
  const QPointF end = current.centerline.back();
  const QPointF before_end = current.centerline[current.centerline.size() - 2];
  const double current_heading =
      std::atan2(end.y() - before_end.y(), end.x() - before_end.x());
  int best_index = -1;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto& edge : current.outgoing) {
    if (edge.to_lane_id == previous_id || visited.contains(edge.to_lane_id)) continue;
    const auto found = index_by_id.constFind(edge.to_lane_id);
    if (found == index_by_id.cend()) continue;
    const int candidate_index = found.value();
    const auto& candidate = lanes[candidate_index];
    if (candidate.centerline.size() < 2) continue;
    const QPointF start = candidate.centerline.front();
    const QPointF next = candidate.centerline[1];
    const double heading = std::atan2(next.y() - start.y(), next.x() - start.x());
    const double heading_change = std::abs(wrap_angle(heading - current_heading));
    // STRAIGHT/LEFT/RIGHT 是正常道路延续；LANE_CHANGE 只在没有更合理的
    // 前向连接时使用，避免自动目标无故跨车道。
    const double maneuver_penalty = edge.maneuver >= 3 ? 25.0 :
                                    edge.maneuver == 0 ? 0.0 : 3.0;
    const double score = distance(end, start) + 8.0 * heading_change +
                         maneuver_penalty;
    if (score < best_score) {
      best_score = score;
      best_index = candidate_index;
    }
  }
  return best_index;
}

}  // namespace navigation_detail

// 沿 LaneGraph 的有向连接从自车位置向前走指定距离，供“一键场景”的自动导航
// 和“自车前向目标”共同使用。结果仍由 global_planner 做最终 snap/可达性校验。
inline ResolvedNavigationGoal resolve_navigation_goal_ahead(
    const QVector<GuiLane>& lanes, double ego_x, double ego_y,
    double ego_yaw_rad, double requested_distance_m) {
  ResolvedNavigationGoal result;
  if (lanes.isEmpty() || !std::isfinite(ego_x) || !std::isfinite(ego_y) ||
      !std::isfinite(ego_yaw_rad) || !std::isfinite(requested_distance_m) ||
      requested_distance_m < 5.0) {
    result.detail = QStringLiteral("地图/位姿/距离无效");
    return result;
  }
  const auto projection = navigation_detail::nearest_forward_lane(
      lanes, ego_x, ego_y, ego_yaw_rad);
  if (projection.lane_index < 0 || projection.distance_m > 12.0) {
    result.detail = QStringLiteral("自车附近找不到同向车道");
    return result;
  }

  QHash<qint64, int> index_by_id;
  for (int i = 0; i < lanes.size(); ++i) index_by_id.insert(lanes[i].id, i);
  int lane_index = projection.lane_index;
  int segment_index = projection.segment_index;
  QPointF cursor = projection.point;
  double covered = 0.0;
  qint64 previous_id = 0;
  QSet<qint64> visited;
  result.start_lane_id = lanes[lane_index].id;

  for (int hop = 0; hop < 256; ++hop) {
    const auto& lane = lanes[lane_index];
    visited.insert(lane.id);
    for (int i = segment_index + 1; i < lane.centerline.size(); ++i) {
      const QPointF next = lane.centerline[i];
      const double segment_length = navigation_detail::distance(cursor, next);
      if (segment_length < 1e-6) {
        cursor = next;
        continue;
      }
      if (covered + segment_length >= requested_distance_m) {
        const double ratio = (requested_distance_m - covered) / segment_length;
        result.valid = true;
        result.x = cursor.x() + ratio * (next.x() - cursor.x());
        result.y = cursor.y() + ratio * (next.y() - cursor.y());
        result.covered_distance_m = requested_distance_m;
        result.goal_lane_id = lane.id;
        result.detail = QStringLiteral("沿车道图前向 %1 m")
                            .arg(requested_distance_m, 0, 'f', 0);
        return result;
      }
      covered += segment_length;
      cursor = next;
    }
    const int successor = navigation_detail::choose_successor(
        lanes, index_by_id, lane, previous_id, visited);
    if (successor < 0) break;
    previous_id = lane.id;
    lane_index = successor;
    segment_index = -1;
    cursor = lanes[lane_index].centerline.front();
  }

  // 短路/图边缺失时，只要已经向前走出足够安全距离，就使用可达末端；
  // 不把离自车几米的退化目标发给规划器。
  if (covered >= std::min(30.0, requested_distance_m * 0.5)) {
    result.valid = true;
    result.x = cursor.x();
    result.y = cursor.y();
    result.covered_distance_m = covered;
    result.goal_lane_id = lanes[lane_index].id;
    result.detail = QStringLiteral("车道图末端，可用前向距离 %1 m")
                        .arg(covered, 0, 'f', 0);
  } else {
    result.detail = QStringLiteral("车道连接不足，无法生成安全前向目标");
  }
  return result;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__NAVIGATION_GOAL_HPP_
