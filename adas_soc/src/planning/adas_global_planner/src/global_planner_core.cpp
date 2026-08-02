#include "adas_global_planner/global_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace adas::planning {
namespace {

constexpr std::int64_t kInvalidLane = 0;

double point_segment_distance(double px, double py, const MapPoint& a, const MapPoint& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double denom = dx * dx + dy * dy;
  if (denom <= 1e-12) return std::hypot(px - a.x, py - a.y);
  const double t = std::clamp(((px - a.x) * dx + (py - a.y) * dy) / denom, 0.0, 1.0);
  return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

MapPoint lane_anchor(const LaneSegment& lane) {
  return lane.centerline.empty() ? MapPoint{} : lane.centerline.back();
}

struct QueueItem {
  double priority{0.0};
  std::int64_t lane_id{0};
  bool operator>(const QueueItem& other) const { return priority > other.priority; }
};

}  // namespace

double polyline_length(const std::vector<MapPoint>& points) {
  double result = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    result += std::hypot(points[i].x - points[i - 1].x,
                         points[i].y - points[i - 1].y);
  }
  return result;
}

bool LaneGraph::add_lane(const LaneSegment& lane) {
  if (lane.id == kInvalidLane || lane.centerline.size() < 2 ||
      lane.speed_limit_mps <= 0.0 || lanes_.count(lane.id) != 0U) {
    return false;
  }
  lanes_.emplace(lane.id, lane);
  return true;
}

bool LaneGraph::add_connection(const LaneConnection& connection) {
  if (connection.from == connection.to || connection.extra_cost_m < 0.0 ||
      lane(connection.from) == nullptr || lane(connection.to) == nullptr) {
    return false;
  }
  auto& edges = outgoing_[connection.from];
  const auto duplicate = std::find_if(edges.begin(), edges.end(), [&](const auto& edge) {
    return edge.to == connection.to && edge.maneuver == connection.maneuver;
  });
  if (duplicate != edges.end()) return false;
  edges.push_back(connection);
  return true;
}

const LaneSegment* LaneGraph::lane(std::int64_t id) const {
  const auto it = lanes_.find(id);
  return it == lanes_.end() ? nullptr : &it->second;
}

const std::vector<LaneConnection>& LaneGraph::outgoing(std::int64_t id) const {
  static const std::vector<LaneConnection> empty;
  const auto it = outgoing_.find(id);
  return it == outgoing_.end() ? empty : it->second;
}

std::int64_t LaneGraph::nearest_lane(double x, double y, double max_distance_m) const {
  if (!std::isfinite(x) || !std::isfinite(y) || max_distance_m < 0.0) return kInvalidLane;
  double best = max_distance_m;
  std::int64_t best_id = kInvalidLane;
  for (const auto& [id, lane_value] : lanes_) {
    for (std::size_t i = 1; i < lane_value.centerline.size(); ++i) {
      const double distance = point_segment_distance(
          x, y, lane_value.centerline[i - 1], lane_value.centerline[i]);
      if (distance <= best) {
        best = distance;
        best_id = id;
      }
    }
  }
  return best_id;
}

GlobalPlannerCore::GlobalPlannerCore(PlannerCost cost) : cost_(cost) {}

double GlobalPlannerCore::transition_cost(const LaneSegment& destination,
                                          const LaneConnection& connection) const {
  double value = polyline_length(destination.centerline) + connection.extra_cost_m;
  if (destination.junction) value += cost_.junction_penalty_m;
  switch (connection.maneuver) {
    case Maneuver::kLaneChangeLeft:
    case Maneuver::kLaneChangeRight:
      value += cost_.lane_change_penalty_m;
      break;
    case Maneuver::kLeft:
    case Maneuver::kRight:
      value += cost_.turn_penalty_m;
      break;
    case Maneuver::kStraight:
      break;
  }
  return value;
}

GlobalRoute GlobalPlannerCore::plan(const LaneGraph& graph, std::int64_t start_lane_id,
                                    std::int64_t goal_lane_id) const {
  GlobalRoute route;
  route.start_lane_id = start_lane_id;
  route.goal_lane_id = goal_lane_id;
  const auto* start = graph.lane(start_lane_id);
  const auto* goal = graph.lane(goal_lane_id);
  if (start == nullptr || goal == nullptr) {
    route.failure_reason = "start or goal lane does not exist";
    return route;
  }

  using CostMap = std::unordered_map<std::int64_t, double>;
  CostMap best_cost;
  std::unordered_map<std::int64_t, LaneConnection> parent;
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  best_cost[start_lane_id] = polyline_length(start->centerline);
  open.push({best_cost[start_lane_id], start_lane_id});

  const MapPoint goal_anchor = lane_anchor(*goal);
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    const auto known = best_cost.find(current.lane_id);
    if (known == best_cost.end()) continue;
    if (current.lane_id == goal_lane_id) break;

    for (const auto& edge : graph.outgoing(current.lane_id)) {
      const auto* next = graph.lane(edge.to);
      if (next == nullptr) continue;
      const double candidate = known->second + transition_cost(*next, edge);
      const auto old = best_cost.find(edge.to);
      if (old == best_cost.end() || candidate < old->second) {
        best_cost[edge.to] = candidate;
        parent[edge.to] = edge;
        const MapPoint anchor = lane_anchor(*next);
        const double heuristic = std::hypot(anchor.x - goal_anchor.x, anchor.y - goal_anchor.y);
        open.push({candidate + heuristic, edge.to});
      }
    }
  }

  if (best_cost.count(goal_lane_id) == 0U) {
    route.failure_reason = "goal is unreachable";
    return route;
  }

  std::vector<std::int64_t> lane_ids{goal_lane_id};
  std::vector<Maneuver> maneuvers;
  while (lane_ids.back() != start_lane_id) {
    const auto it = parent.find(lane_ids.back());
    if (it == parent.end()) {
      route.failure_reason = "route reconstruction failed";
      return route;
    }
    maneuvers.push_back(it->second.maneuver);
    lane_ids.push_back(it->second.from);
  }
  std::reverse(lane_ids.begin(), lane_ids.end());
  std::reverse(maneuvers.begin(), maneuvers.end());

  double accumulated = 0.0;
  for (std::size_t i = 0; i < lane_ids.size(); ++i) {
    const auto* lane_value = graph.lane(lane_ids[i]);
    RouteSegment segment;
    segment.lane_id = lane_value->id;
    segment.entry_maneuver = i == 0 ? Maneuver::kStraight : maneuvers[i - 1];
    segment.length_m = polyline_length(lane_value->centerline);
    accumulated = best_cost[lane_ids[i]];
    segment.accumulated_cost_m = accumulated;
    segment.speed_limit_mps = lane_value->speed_limit_mps;
    segment.junction = lane_value->junction;
    segment.centerline = lane_value->centerline;
    route.segments.push_back(std::move(segment));
  }
  route.total_cost_m = best_cost[goal_lane_id];
  route.valid = true;
  return route;
}

GlobalRoute GlobalPlannerCore::plan(const LaneGraph& graph, double start_x, double start_y,
                                    double goal_x, double goal_y) const {
  const auto start_lane = graph.nearest_lane(start_x, start_y, cost_.snap_max_distance_m);
  const auto goal_lane = graph.nearest_lane(goal_x, goal_y, cost_.snap_max_distance_m);
  if (start_lane == kInvalidLane || goal_lane == kInvalidLane) {
    GlobalRoute route;
    route.failure_reason = "start or goal is too far from a driving lane";
    return route;
  }
  return plan(graph, start_lane, goal_lane);
}

}  // namespace adas::planning
