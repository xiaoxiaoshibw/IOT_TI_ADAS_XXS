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

// Empty incoming IDs are never accepted. An empty expected ID binds once to
// the first non-empty ID; a bound session never switches implicitly.
bool bind_or_accept_run_id(std::string& expected, const std::string& incoming);

// P0.3: GlobalRoute.route_id 是 uint32,0 是保留值(让消费端识别"未初始化
// revision")。每次语义路线变化都得拿到一个非零新 revision;心跳(同一条
// 路线重复发送)允许复用上次 ID。溢出从 UINT32_MAX 回到 1,跳过 0。
// last == 0 → 返回 1,避免下游把它当作"保留值"误丢弃。
std::uint32_t next_route_revision(std::uint32_t last);

// P0.B 失效恢复策略：纯函数，可独立 gtest。
// - 首次规划失败（is_replan=false） ⇒ 清空请求（必须清除 goal 与路线）。
// - 重规划失败 + 已有可用路线（previous_route_valid=true） ⇒ 保留旧路线，发布 DRIVING。
// - 重规划失败且无旧路线 ⇒ 与"首次失败"行为一致：清空请求。
// 返回 false ⇒ 节点应当 clear_request_state()；返回 true ⇒ 仅保留旧路线。
bool should_clear_request_on_failure(bool is_replan, bool previous_route_valid);

}  // namespace adas::planning

#endif  // ADAS_GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_
