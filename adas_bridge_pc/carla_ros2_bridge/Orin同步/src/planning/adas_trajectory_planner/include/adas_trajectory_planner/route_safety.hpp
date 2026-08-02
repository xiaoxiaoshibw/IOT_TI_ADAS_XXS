#ifndef ADAS_TRAJECTORY_PLANNER__ROUTE_SAFETY_HPP_
#define ADAS_TRAJECTORY_PLANNER__ROUTE_SAFETY_HPP_

namespace adas::planning {

inline bool route_requires_safe_stop(bool require_active_route,
                                     bool navigation_active,
                                     bool route_status_valid,
                                     bool path_available,
                                     double route_age_s,
                                     double timeout_s) {
  if (!navigation_active) return require_active_route;
  const bool fresh = route_age_s >= 0.0 && route_age_s < timeout_s;
  return !route_status_valid || !path_available || !fresh;
}

}  // namespace adas::planning

#endif  // ADAS_TRAJECTORY_PLANNER__ROUTE_SAFETY_HPP_
