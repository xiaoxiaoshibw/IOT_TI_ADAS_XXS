#ifndef ADAS_GLOBAL_PLANNER__ROUTE_VALIDATOR_HPP_
#define ADAS_GLOBAL_PLANNER__ROUTE_VALIDATOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "adas_global_planner/semantic_route.hpp"

namespace adas::planning {

struct RouteValidationConfig {
  double max_adjacent_gap_m{3.0};
  double min_point_spacing_m{0.05};
  double max_heading_jump_rad{1.2};
  double max_reverse_progress_m{0.5};
  double maximum_duplicate_ratio{0.1};
  double minimum_route_length_m{0.1};
  std::size_t min_route_points{2U};
};

struct RouteValidationResult {
  bool valid{false};
  std::string reason;
  std::size_t offending_index{0U};
  double maximum_adjacent_gap_m{0.0};
  double maximum_heading_jump_rad{0.0};
  double maximum_reverse_progress_m{0.0};
  double route_length_m{0.0};
};

RouteValidationResult validate_route(
    const std::vector<SemanticRoutePoint>& points,
    const RouteValidationConfig& config = {});

}  // namespace adas::planning

#endif  // ADAS_GLOBAL_PLANNER__ROUTE_VALIDATOR_HPP_
