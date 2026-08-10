#ifndef ADAS_GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_
#define ADAS_GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace adas::planning {

struct MapPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

using Pose = MapPoint;

enum class Maneuver : std::uint8_t {
  kStraight = 0,
  kLeft = 1,
  kRight = 2,
  kLaneChangeLeft = 3,
  kLaneChangeRight = 4,
};

struct LaneSegment {
  std::int64_t id{0};
  std::vector<MapPoint> centerline;
  double speed_limit_mps{13.9};
  bool junction{false};
};

struct LaneConnection {
  std::int64_t from{0};
  std::int64_t to{0};
  Maneuver maneuver{Maneuver::kStraight};
  double extra_cost_m{0.0};
};

struct PlannerCost {
  double lane_change_penalty_m{8.0};
  double junction_penalty_m{2.0};
  double turn_penalty_m{1.0};
  double snap_max_distance_m{8.0};
};

struct RouteSegment {
  std::int64_t lane_id{0};
  Maneuver entry_maneuver{Maneuver::kStraight};
  double length_m{0.0};
  double accumulated_cost_m{0.0};
  double speed_limit_mps{0.0};
  bool junction{false};
  std::vector<MapPoint> centerline;
};

struct GlobalRoute {
  bool valid{false};
  std::string failure_reason;
  std::int64_t start_lane_id{0};
  std::int64_t goal_lane_id{0};
  double total_cost_m{0.0};
  std::vector<RouteSegment> segments;
};

class LaneGraph {
 public:
  bool add_lane(const LaneSegment& lane);
  bool add_connection(const LaneConnection& connection);
  const LaneSegment* lane(std::int64_t id) const;
  const std::vector<LaneConnection>& outgoing(std::int64_t id) const;
  std::int64_t nearest_lane(double x, double y, double max_distance_m) const;
  std::size_t lane_count() const { return lanes_.size(); }

 private:
  std::unordered_map<std::int64_t, LaneSegment> lanes_;
  std::unordered_map<std::int64_t, std::vector<LaneConnection>> outgoing_;
};

class GlobalPlannerCore {
 public:
  explicit GlobalPlannerCore(PlannerCost cost = PlannerCost());

  GlobalRoute plan(const LaneGraph& graph, std::int64_t start_lane_id,
                   std::int64_t goal_lane_id) const;
  GlobalRoute plan(const LaneGraph& graph, double start_x, double start_y,
                   double goal_x, double goal_y) const;

  // Replan from the current pose to the original goal.  The output route is
  // only replaced by the caller after this method returns true, allowing the
  // node to retain a valid previous route on failure.
  bool replan(const LaneGraph& graph, const Pose& current, const Pose& goal,
              GlobalRoute& route) const;

 private:
  double transition_cost(const LaneSegment& destination,
                         const LaneConnection& connection) const;
  PlannerCost cost_;
};

class ReplanPolicy {
 public:
  ReplanPolicy(double threshold_m, double cooldown_s);

  bool request(double deviation_m, double now_s);
  void reset();

 private:
  double threshold_m_;
  double cooldown_s_;
  bool attempted_{false};
  double last_attempt_s_{0.0};
};

double polyline_length(const std::vector<MapPoint>& points);

}  // namespace adas::planning

#endif  // ADAS_GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_
