#include "adas_global_planner/semantic_route.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace adas::planning {
namespace {

struct DirectedLine {
  std::vector<MapPoint> points;
  std::vector<double> stations;
  double length_m{0.0};
};

struct Projection {
  std::size_t segment{0U};
  double fraction{0.0};
  double station_m{0.0};
  double lateral_distance_m{0.0};
  MapPoint point;
};

struct LaneChangeJoin {
  double source_start_station_m{0.0};
  double target_station_m{0.0};
  MapPoint source_start;
  MapPoint target;
};

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double angle_difference(double first, double second) {
  return std::abs(normalize_angle(first - second));
}

bool is_lane_change(Maneuver maneuver) {
  return maneuver == Maneuver::kLaneChangeLeft ||
         maneuver == Maneuver::kLaneChangeRight;
}

DirectedLine make_directed_line(const std::vector<MapPoint>& input) {
  DirectedLine line;
  line.points = input;
  double direction_score = 0.0;
  for (std::size_t index = 1U; index < line.points.size(); ++index) {
    const auto& from = line.points[index - 1U];
    const auto& to = line.points[index];
    direction_score += (to.x - from.x) * std::cos(from.yaw) +
                       (to.y - from.y) * std::sin(from.yaw);
  }
  if (direction_score < 0.0) std::reverse(line.points.begin(), line.points.end());

  line.stations.reserve(line.points.size());
  line.stations.push_back(0.0);
  for (std::size_t index = 1U; index < line.points.size(); ++index) {
    line.length_m += std::hypot(
        line.points[index].x - line.points[index - 1U].x,
        line.points[index].y - line.points[index - 1U].y);
    line.stations.push_back(line.length_m);
  }
  return line;
}

MapPoint point_at_station(const DirectedLine& line, double station_m) {
  const double station = std::clamp(station_m, 0.0, line.length_m);
  const auto upper = std::upper_bound(
      line.stations.begin(), line.stations.end(), station);
  if (upper == line.stations.begin()) return line.points.front();
  if (upper == line.stations.end()) return line.points.back();
  const std::size_t next = static_cast<std::size_t>(
      std::distance(line.stations.begin(), upper));
  const std::size_t previous = next - 1U;
  const double span = line.stations[next] - line.stations[previous];
  const double fraction = span > 1e-12
                              ? (station - line.stations[previous]) / span
                              : 0.0;
  MapPoint point;
  point.x = line.points[previous].x +
            fraction * (line.points[next].x - line.points[previous].x);
  point.y = line.points[previous].y +
            fraction * (line.points[next].y - line.points[previous].y);
  point.yaw = normalize_angle(
      line.points[previous].yaw +
      fraction * normalize_angle(line.points[next].yaw -
                                 line.points[previous].yaw));
  return point;
}

Projection project_to_line(const DirectedLine& line, double x, double y) {
  Projection best;
  double best_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index < line.points.size(); ++index) {
    const auto& from = line.points[index - 1U];
    const auto& to = line.points[index];
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double squared_length = dx * dx + dy * dy;
    const double fraction = squared_length > 1e-12
                                ? std::clamp(((x - from.x) * dx +
                                              (y - from.y) * dy) /
                                                 squared_length,
                                             0.0, 1.0)
                                : 0.0;
    const double projected_x = from.x + fraction * dx;
    const double projected_y = from.y + fraction * dy;
    const double squared = (x - projected_x) * (x - projected_x) +
                           (y - projected_y) * (y - projected_y);
    if (squared < best_squared) {
      best_squared = squared;
      best.segment = index - 1U;
      best.fraction = fraction;
      best.station_m = line.stations[index - 1U] +
                       fraction * (line.stations[index] -
                                   line.stations[index - 1U]);
      best.lateral_distance_m = std::sqrt(squared);
      best.point = point_at_station(line, best.station_m);
    }
  }
  return best;
}

void append_point(SemanticRouteResult& result, const RouteSegment& segment,
                  const MapPoint& point, Maneuver maneuver,
                  double min_point_spacing_m) {
  if (!result.points.empty()) {
    const auto& previous = result.points.back().pose;
    const double distance = std::hypot(point.x - previous.x,
                                       point.y - previous.y);
    if (distance < min_point_spacing_m) return;
    result.length_m += distance;
  }
  SemanticRoutePoint converted;
  converted.pose = point;
  converted.lane_id = segment.lane_id;
  converted.road_id = road_id_from_encoded_lane(segment.lane_id);
  converted.speed_limit_mps = segment.speed_limit_mps;
  converted.maneuver = maneuver;
  result.points.push_back(converted);
}

void append_slice(SemanticRouteResult& result, const RouteSegment& segment,
                  const DirectedLine& line, double begin_station_m,
                  double end_station_m, const RouteGeometryConfig& config) {
  const double begin = std::clamp(begin_station_m, 0.0, line.length_m);
  const double end = std::clamp(end_station_m, begin, line.length_m);
  const double length = end - begin;
  const std::size_t intervals = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
              length / std::max(config.sample_spacing_m,
                                config.min_point_spacing_m))));
  for (std::size_t index = 0U; index <= intervals; ++index) {
    const double fraction = static_cast<double>(index) /
                            static_cast<double>(intervals);
    append_point(result, segment, point_at_station(line, begin + fraction * length),
                 segment.entry_maneuver, config.min_point_spacing_m);
  }
}

std::optional<LaneChangeJoin> find_lane_change_join(
    const DirectedLine& source, double source_entry_station_m,
    const DirectedLine& target, double target_end_station_m,
    Maneuver maneuver,
    const RouteGeometryConfig& config) {
  std::optional<LaneChangeJoin> best;
  const double upper = std::clamp(target_end_station_m, 0.0, target.length_m);
  const double step = std::max(config.sample_spacing_m, 0.5);
  std::vector<double> candidate_stations;
  for (double station = 0.0; station < upper; station += step) {
    candidate_stations.push_back(station);
  }
  candidate_stations.push_back(upper);
  for (const double candidate_station : candidate_stations) {
    const MapPoint target_point = point_at_station(target, candidate_station);
    const Projection on_source = project_to_line(source, target_point.x,
                                                 target_point.y);
    if (on_source.station_m < source_entry_station_m +
                                  config.lane_change_min_length_m ||
        on_source.lateral_distance_m < config.lane_change_min_lateral_m ||
        on_source.lateral_distance_m > config.lane_change_max_lateral_m ||
        angle_difference(on_source.point.yaw, target_point.yaw) >
            config.lane_change_max_heading_difference_rad) {
      continue;
    }
    const double lateral =
        -std::sin(on_source.point.yaw) *
            (target_point.x - on_source.point.x) +
        std::cos(on_source.point.yaw) *
            (target_point.y - on_source.point.y);
    if ((maneuver == Maneuver::kLaneChangeLeft && lateral <= 0.0) ||
        (maneuver == Maneuver::kLaneChangeRight && lateral >= 0.0)) {
      continue;
    }
    LaneChangeJoin join;
    join.source_start_station_m =
        on_source.station_m - config.lane_change_min_length_m;
    join.target_station_m = candidate_station;
    join.source_start = point_at_station(source, join.source_start_station_m);
    join.target = target_point;
    best = join;
  }
  return best;
}

void append_lane_change(SemanticRouteResult& result,
                        const RouteSegment& target_segment,
                        const LaneChangeJoin& join, Maneuver maneuver,
                        const RouteGeometryConfig& config) {
  const double tangent_scale = config.lane_change_min_length_m;
  const double chord = std::hypot(join.target.x - join.source_start.x,
                                  join.target.y - join.source_start.y);
  const std::size_t intervals = std::max<std::size_t>(
      2U, static_cast<std::size_t>(std::ceil(
              chord / std::max(config.sample_spacing_m,
                               config.min_point_spacing_m))));
  for (std::size_t index = 1U; index <= intervals; ++index) {
    const double t = static_cast<double>(index) /
                     static_cast<double>(intervals);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    const double m0x = tangent_scale * std::cos(join.source_start.yaw);
    const double m0y = tangent_scale * std::sin(join.source_start.yaw);
    const double m1x = tangent_scale * std::cos(join.target.yaw);
    const double m1y = tangent_scale * std::sin(join.target.yaw);
    MapPoint point;
    point.x = h00 * join.source_start.x + h10 * m0x +
              h01 * join.target.x + h11 * m1x;
    point.y = h00 * join.source_start.y + h10 * m0y +
              h01 * join.target.y + h11 * m1y;
    const double dh00 = 6.0 * t2 - 6.0 * t;
    const double dh10 = 3.0 * t2 - 4.0 * t + 1.0;
    const double dh01 = -dh00;
    const double dh11 = 3.0 * t2 - 2.0 * t;
    const double derivative_x = dh00 * join.source_start.x + dh10 * m0x +
                                dh01 * join.target.x + dh11 * m1x;
    const double derivative_y = dh00 * join.source_start.y + dh10 * m0y +
                                dh01 * join.target.y + dh11 * m1y;
    point.yaw = std::atan2(derivative_y, derivative_x);
    append_point(result, target_segment, point, maneuver,
                 config.min_point_spacing_m);
  }
}

}  // namespace

std::int32_t road_id_from_encoded_lane(std::int64_t lane_id) {
  if (lane_id <= 0) return 0;
  const std::int64_t value = lane_id >> 24;
  return static_cast<std::int32_t>(
      std::clamp<std::int64_t>(value, 0,
                               std::numeric_limits<std::int32_t>::max()));
}

SemanticRouteResult build_semantic_route(const GlobalRoute& route,
                                         double start_x, double start_y,
                                         double goal_x, double goal_y,
                                         const RouteGeometryConfig& config) {
  SemanticRouteResult result;
  if (!route.valid || route.segments.empty()) {
    result.failure_reason = "route has no valid segments";
    return result;
  }
  if (config.sample_spacing_m <= 0.0 || config.min_point_spacing_m <= 0.0 ||
      config.max_connection_distance_m <= 0.0 ||
      config.lane_change_min_length_m <= 0.0) {
    result.failure_reason = "route geometry configuration is invalid";
    return result;
  }

  std::vector<DirectedLine> lines;
  lines.reserve(route.segments.size());
  for (const auto& segment : route.segments) {
    if (segment.centerline.size() < 2U) {
      result.failure_reason = "route contains a degenerate segment";
      return result;
    }
    lines.push_back(make_directed_line(segment.centerline));
    if (lines.back().length_m <= config.min_point_spacing_m) {
      result.failure_reason = "route contains a zero-length segment";
      return result;
    }
  }

  const Projection start = project_to_line(lines.front(), start_x, start_y);
  const Projection goal = project_to_line(lines.back(), goal_x, goal_y);
  if (route.segments.size() == 1U &&
      goal.station_m + config.min_point_spacing_m < start.station_m) {
    result.failure_reason = "goal lies behind start on the same directed lane";
    return result;
  }

  double entry_station = start.station_m;
  for (std::size_t index = 0U; index < route.segments.size(); ++index) {
    const bool last = index + 1U == route.segments.size();
    const auto& segment = route.segments[index];
    const auto& line = lines[index];
    if (last) {
      if (goal.station_m + config.min_point_spacing_m < entry_station) {
        result.failure_reason = "goal lies behind the connected route entry";
        result.points.clear();
        result.length_m = 0.0;
        return result;
      }
      append_slice(result, segment, line, entry_station, goal.station_m, config);
      break;
    }

    const auto& target_segment = route.segments[index + 1U];
    const auto& target_line = lines[index + 1U];
    const double target_end = index + 2U == route.segments.size()
                                  ? goal.station_m
                                  : target_line.length_m;
    if (is_lane_change(target_segment.entry_maneuver)) {
      const auto join = find_lane_change_join(
          line, entry_station, target_line, target_end,
          target_segment.entry_maneuver, config);
      if (!join) {
        result.failure_reason =
            "lane-change has no safe common forward connection interval";
        result.points.clear();
        result.length_m = 0.0;
        return result;
      }
      append_slice(result, segment, line, entry_station,
                   join->source_start_station_m, config);
      append_lane_change(result, target_segment, *join,
                         target_segment.entry_maneuver, config);
      entry_station = join->target_station_m;
      continue;
    }

    append_slice(result, segment, line, entry_station, line.length_m, config);
    const MapPoint source_end = point_at_station(line, line.length_m);
    const Projection target_entry = project_to_line(
        target_line, source_end.x, source_end.y);
    if (target_entry.lateral_distance_m > config.max_connection_distance_m ||
        angle_difference(source_end.yaw, target_entry.point.yaw) >
            config.max_connection_heading_jump_rad) {
      result.failure_reason =
          "successor has no safe forward geometric connection";
      result.points.clear();
      result.length_m = 0.0;
      return result;
    }
    entry_station = target_entry.station_m;
  }

  if (result.points.size() < 2U) {
    result.failure_reason = "trimmed route has fewer than two points";
    result.points.clear();
    result.length_m = 0.0;
    return result;
  }
  result.points.back().stop = true;
  result.valid = true;
  return result;
}

}  // namespace adas::planning
