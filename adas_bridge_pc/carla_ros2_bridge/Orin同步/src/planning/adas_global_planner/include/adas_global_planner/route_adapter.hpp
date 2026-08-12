#ifndef ADAS_GLOBAL_PLANNER__ROUTE_ADAPTER_HPP_
#define ADAS_GLOBAL_PLANNER__ROUTE_ADAPTER_HPP_

#include <cmath>
#include <cstdint>
#include <string>

#include "adas_msgs/msg/global_route.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace adas::planning {

inline bool global_route_frame_valid(const std::string& header_frame,
                                     const std::string& declared_frame) {
  return !header_frame.empty() && header_frame == declared_frame;
}

// P0.C/P0.3: 纯判定函数；adapter 的运行时分支全部由它产出，方便 gtest 直接覆盖。
// run_id / frame / point / 状态检查全部 fail-closed；不满足任何一条都视作
// "不应被采纳的 route"，adapter 不会更新已采纳状态或再发布 Path。
struct RouteAdapterDecision {
  bool accept{false};            // 全部硬约束满足 + 状态机允许采纳
  bool publish_clear{false};     // 当前会话存在但 status 非 VALID / 路线空
  std::string reason;            // 调试日志用
};

inline bool route_point_finite(const adas_msgs::msg::RoutePoint& point) {
  return std::isfinite(static_cast<double>(point.x)) &&
         std::isfinite(static_cast<double>(point.y)) &&
         std::isfinite(static_cast<double>(point.yaw));
}

inline RouteAdapterDecision evaluate_route_for_adapter(
    const std::string& bound_run_id,
    const adas_msgs::msg::GlobalRoute& route) {
  RouteAdapterDecision decision;
  // P0.C: 适配器必须按会话绑定；本机 run_id 为空就拒绝一切消息,
  // split-topology 没有共享 ID 时不会让旧数据漏过。
  if (bound_run_id.empty()) {
    decision.reason = "adapter has no bound run_id";
    return decision;
  }
  if (route.run_id.empty()) {
    decision.reason = "incoming route has empty run_id";
    return decision;
  }
  if (route.run_id != bound_run_id) {
    decision.reason = "incoming route run_id does not match bound session";
    return decision;
  }
  // P0.3: 0 route_id 视为保留/未初始化，规划器必须给出非零 revision
  // 才能让"第二条有效路线"被消费端接受。
  if (route.route_id == 0U) {
    decision.reason = "incoming route has reserved route_id 0";
    return decision;
  }
  if (!global_route_frame_valid(route.header.frame_id, route.frame_id) ||
      route.header.frame_id.empty()) {
    decision.reason = "frame_id mismatch or empty";
    return decision;
  }
  if (route.status != adas_msgs::msg::GlobalRoute::STATUS_VALID) {
    decision.accept = true;            // 当前会话的清空/失败/取消也算合法状态
    decision.publish_clear = true;
    decision.reason = "non-VALID status triggers clear path";
    return decision;
  }
  if (route.points.size() < 2U) {
    decision.reason = "VALID route has fewer than 2 points";
    return decision;
  }
  for (const auto& point : route.points) {
    if (!route_point_finite(point)) {
      decision.reason = "route point contains non-finite x/y/yaw";
      return decision;
    }
  }
  if (!std::isfinite(static_cast<double>(route.length)) ||
      route.length < 0.0F) {
    decision.reason = "route length is non-finite or negative";
    return decision;
  }
  decision.accept = true;
  decision.publish_clear = false;
  decision.reason = "valid route";
  return decision;
}

class RouteAdapterNode : public rclcpp::Node {
 public:
  RouteAdapterNode();

  // 测试访问器：不参与 rclcpp 启动逻辑；让 gtest 直接覆盖 dedup 状态。
  bool test_published_once() const { return published_once_; }
  std::uint32_t test_last_route_id() const { return last_route_id_; }
  std::uint8_t test_last_status() const { return last_status_; }
  const std::string& test_bound_run_id() const { return bound_run_id_; }
  // 测试钩子：单元测试可显式绑定 run_id（生产路径由 ROS 参数注入）。
  void test_set_bound_run_id(const std::string& value) { bound_run_id_ = value; }
  // 测试钩子：直接调用 adapt_and_publish 走一次完整评估+发布路径。
  void test_feed(const adas_msgs::msg::GlobalRoute& route) {
    adapt_and_publish(route);
  }
  // 测试钩子：重置 dedup 状态,排除构造期 startup clear 的影响。
  void test_reset_state() {
    published_once_ = false;
    last_route_id_ = 0U;
    last_status_ = adas_msgs::msg::GlobalRoute::STATUS_INVALID;
  }

 private:
  void adapt_and_publish(const adas_msgs::msg::GlobalRoute& route);
  void publish_cleared_path();

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Subscription<adas_msgs::msg::GlobalRoute>::SharedPtr subscription_;

  // P0.C: 本机会话 run_id；空表示 split-topology 未注入，fail-closed。
  std::string bound_run_id_;
  std::string frame_id_;
  // P0.3: dedup key = run_id + route_id + status。
  // 重复心跳（同 route_id + status）不再重复发布。
  bool published_once_{false};
  std::uint32_t last_route_id_{0U};
  std::uint8_t last_status_{adas_msgs::msg::GlobalRoute::STATUS_INVALID};
  // 启动时立即清空一次,避免 transient_local 旧路径跨会话残留。
  bool startup_clear_published_{false};
};

}  // namespace adas::planning

#endif  // ADAS_GLOBAL_PLANNER__ROUTE_ADAPTER_HPP_
