#ifndef ADAS_GLOBAL_PLANNER__SEMANTIC_ROUTE_HPP_
#define ADAS_GLOBAL_PLANNER__SEMANTIC_ROUTE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "adas_global_planner/global_planner_core.hpp"

namespace adas::planning {

struct SemanticRoutePoint {
  MapPoint pose;
  std::int64_t lane_id{0};
  std::int32_t road_id{0};
  double speed_limit_mps{0.0};
  Maneuver maneuver{Maneuver::kStraight};
  bool stop{false};
};

struct SemanticRouteResult {
  bool valid{false};
  std::string failure_reason;
  double length_m{0.0};
  std::vector<SemanticRoutePoint> points;
};

struct RouteGeometryConfig {
  double sample_spacing_m{2.0};
  double min_point_spacing_m{0.05};
  double max_connection_distance_m{3.0};
  double max_connection_heading_jump_rad{1.2};
  double lane_change_min_length_m{10.0};
  double lane_change_min_lateral_m{1.0};
  double lane_change_max_lateral_m{6.0};
  double lane_change_max_heading_difference_rad{0.35};
};

std::int32_t road_id_from_encoded_lane(std::int64_t lane_id);

SemanticRouteResult build_semantic_route(const GlobalRoute& route,
                                         double start_x, double start_y,
                                         double goal_x, double goal_y,
                                         const RouteGeometryConfig& config = {});

}  // namespace adas::planning

#endif  // ADAS_GLOBAL_PLANNER__SEMANTIC_ROUTE_HPP_
