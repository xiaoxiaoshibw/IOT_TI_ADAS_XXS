#include "adas_global_planner/route_validator.hpp"

#include <algorithm>
#include <cmath>

namespace adas::planning {
namespace {

double angle_difference(double first, double second) {
  return std::abs(std::atan2(std::sin(first - second),
                             std::cos(first - second)));
}

RouteValidationResult invalid(RouteValidationResult result,
                              std::size_t index, std::string reason) {
  result.valid = false;
  result.offending_index = index;
  result.reason = std::move(reason);
  return result;
}

}  // namespace

RouteValidationResult validate_route(
    const std::vector<SemanticRoutePoint>& points,
    const RouteValidationConfig& config) {
  RouteValidationResult result;
  if (points.size() < config.min_route_points) {
    return invalid(result, points.size(), "route has too few points");
  }

  std::size_t duplicates = 0U;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const auto& point = points[index];
    if (!std::isfinite(point.pose.x) || !std::isfinite(point.pose.y) ||
        !std::isfinite(point.pose.yaw)) {
      return invalid(result, index, "route contains a non-finite pose");
    }
    if (!std::isfinite(point.speed_limit_mps) || point.speed_limit_mps <= 0.0) {
      return invalid(result, index, "route contains an invalid speed limit");
    }
    if (point.lane_id == 0 ||
        point.road_id != road_id_from_encoded_lane(point.lane_id)) {
      return invalid(result, index, "route contains inconsistent road/lane semantics");
    }
    if (index == 0U) continue;

    const auto& previous = points[index - 1U];
    const double dx = point.pose.x - previous.pose.x;
    const double dy = point.pose.y - previous.pose.y;
    const double gap = std::hypot(dx, dy);
    result.route_length_m += gap;
    result.maximum_adjacent_gap_m =
        std::max(result.maximum_adjacent_gap_m, gap);
    if (gap < config.min_point_spacing_m) ++duplicates;
    if (gap > config.max_adjacent_gap_m) {
      return invalid(result, index, "adjacent point gap exceeds limit");
    }

    const double heading_jump =
        angle_difference(point.pose.yaw, previous.pose.yaw);
    result.maximum_heading_jump_rad =
        std::max(result.maximum_heading_jump_rad, heading_jump);
    if (heading_jump > config.max_heading_jump_rad) {
      return invalid(result, index, "heading jump exceeds limit");
    }

    const double forward =
        dx * std::cos(previous.pose.yaw) + dy * std::sin(previous.pose.yaw);
    const double reverse = std::max(0.0, -forward);
    result.maximum_reverse_progress_m =
        std::max(result.maximum_reverse_progress_m, reverse);
    if (reverse > config.max_reverse_progress_m) {
      return invalid(result, index, "reverse progress exceeds limit");
    }
  }

  if (static_cast<double>(duplicates) /
          static_cast<double>(points.size() - 1U) >
      config.maximum_duplicate_ratio) {
    return invalid(result, 0U, "duplicate point ratio exceeds limit");
  }
  if (result.route_length_m < config.minimum_route_length_m) {
    return invalid(result, points.size() - 1U, "route is too short");
  }
  for (std::size_t index = 0U; index + 1U < points.size(); ++index) {
    if (points[index].stop) {
      return invalid(result, index, "STOP appears before route endpoint");
    }
  }
  if (!points.back().stop) {
    return invalid(result, points.size() - 1U, "route endpoint is not STOP");
  }

  result.valid = true;
  result.reason = "route geometry is safe";
  return result;
}

}  // namespace adas::planning
