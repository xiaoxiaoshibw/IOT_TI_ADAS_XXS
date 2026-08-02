#ifndef ADAS_BEHAVIOR_PLANNER__NAVIGATION_ACTIVATION_HPP_
#define ADAS_BEHAVIOR_PLANNER__NAVIGATION_ACTIVATION_HPP_

namespace adas::planning {

inline bool navigation_requires_stop(bool require_active_route,
                                     bool route_observed,
                                     bool route_valid) {
  if (require_active_route) return !route_observed || !route_valid;
  return route_observed && !route_valid;
}

}  // namespace adas::planning

#endif  // ADAS_BEHAVIOR_PLANNER__NAVIGATION_ACTIVATION_HPP_
