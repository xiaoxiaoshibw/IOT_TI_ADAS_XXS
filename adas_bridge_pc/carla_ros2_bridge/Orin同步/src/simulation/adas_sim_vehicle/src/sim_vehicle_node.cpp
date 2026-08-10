// adas_sim_vehicle 节点壳：参数装配 + 消息转换 + 定时器，算法全部在 core
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "adas_msgs/msg/actuation_command.hpp"
#include "adas_msgs/msg/lane_state.hpp"
#include "adas_msgs/msg/steering_report.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "adas_sim_vehicle/sim_vehicle_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace adas::sim {

class SimVehicleNode : public rclcpp::Node {
 public:
  explicit SimVehicleNode(const rclcpp::NodeOptions& options)
      : Node("sim_vehicle", options) {
    // ── 参数 ──
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    VehicleParams vp;
    vp.wheelbase_m = declare_parameter<double>("vehicle.wheelbase_m", 2.7);
    vp.max_steer_rad = declare_parameter<double>("vehicle.max_steer_rad", 0.6);
    vp.max_accel_mps2 = declare_parameter<double>("vehicle.max_accel_mps2", 3.0);
    vp.max_decel_mps2 = declare_parameter<double>("vehicle.max_decel_mps2", 8.0);
    vp.steer_tau_s = declare_parameter<double>("vehicle.steer_tau_s", 0.2);
    vp.drag_accel_mps2 = declare_parameter<double>("vehicle.drag_accel_mps2", 0.1);

    const auto seg_lengths = declare_parameter<std::vector<double>>(
        "track.segment_lengths_m", std::vector<double>{500.0});
    const auto seg_curvatures = declare_parameter<std::vector<double>>(
        "track.segment_curvatures", std::vector<double>{0.0});
    const double lane_width = declare_parameter<double>("track.lane_width_m", 3.5);
    if (seg_lengths.size() != seg_curvatures.size() || seg_lengths.empty()) {
      throw std::invalid_argument("track.segment_lengths_m 与 segment_curvatures 必须等长非空");
    }
    std::vector<TrackSegment> segments;
    for (std::size_t i = 0; i < seg_lengths.size(); ++i) {
      segments.push_back(TrackSegment{seg_lengths[i], seg_curvatures[i]});
    }
    core_ = std::make_unique<SimVehicleCore>(vp, segments, lane_width);
    core_->set_initial_state(declare_parameter<double>("initial.station_m", 0.0),
                             declare_parameter<double>("initial.lateral_offset_m", 0.0),
                             declare_parameter<double>("initial.speed_mps", 0.0));

    // schema-v1 场景由 launch Python 审计后展平为基础类型数组。节点仍执行
    // 严格等长/offset 校验，避免半套 actor 参数进入仿真。
    const bool scripted_enabled =
        declare_parameter<bool>("scripted.enabled", false);
    const auto actor_ids = declare_parameter<std::vector<std::int64_t>>(
        "scripted.ids", std::vector<std::int64_t>{});
    const auto actor_classes = declare_parameter<std::vector<std::int64_t>>(
        "scripted.classifications", std::vector<std::int64_t>{});
    const auto actor_stations = declare_parameter<std::vector<double>>(
        "scripted.initial_station_m", std::vector<double>{});
    const auto actor_laterals = declare_parameter<std::vector<double>>(
        "scripted.initial_lateral_m", std::vector<double>{});
    const auto actor_speeds = declare_parameter<std::vector<double>>(
        "scripted.initial_speed_mps", std::vector<double>{});
    const auto actor_accels = declare_parameter<std::vector<double>>(
        "scripted.accel_limit_mps2", std::vector<double>{});
    const auto profile_offsets = declare_parameter<std::vector<std::int64_t>>(
        "scripted.profile_offsets", std::vector<std::int64_t>{});
    const auto profile_times = declare_parameter<std::vector<double>>(
        "scripted.profile_times_s", std::vector<double>{});
    const auto profile_speeds = declare_parameter<std::vector<double>>(
        "scripted.profile_speeds_mps", std::vector<double>{});
    const auto brake_starts = declare_parameter<std::vector<double>>(
        "scripted.hard_brake_start_s", std::vector<double>{});
    const auto brake_ends = declare_parameter<std::vector<double>>(
        "scripted.hard_brake_end_s", std::vector<double>{});
    const auto trigger_gaps = declare_parameter<std::vector<double>>(
        "scripted.trigger_ego_gap_m", std::vector<double>{});
    const auto crossing_ends = declare_parameter<std::vector<double>>(
        "scripted.crossing_end_lateral_m", std::vector<double>{});
    const auto crossing_speeds = declare_parameter<std::vector<double>>(
        "scripted.crossing_speed_mps", std::vector<double>{});
    if (scripted_enabled) {
      const std::size_t count = actor_ids.size();
      const bool actor_arrays_ok =
          actor_classes.size() == count && actor_stations.size() == count &&
          actor_laterals.size() == count && actor_speeds.size() == count &&
          actor_accels.size() == count && brake_starts.size() == count &&
          brake_ends.size() == count && trigger_gaps.size() == count &&
          crossing_ends.size() == count && crossing_speeds.size() == count;
      const bool profile_ok = profile_offsets.size() == count + 1 &&
                              !profile_offsets.empty() &&
                              profile_offsets.front() == 0 &&
                              profile_offsets.back() ==
                                  static_cast<std::int64_t>(profile_times.size()) &&
                              profile_times.size() == profile_speeds.size();
      if (!actor_arrays_ok || !profile_ok) {
        throw std::invalid_argument("scripted actor arrays/offsets are inconsistent");
      }
      std::vector<ScriptedActor> actors;
      actors.reserve(count);
      for (std::size_t i = 0; i < count; ++i) {
        if (actor_ids[i] <= 0 || actor_classes[i] < 0 || actor_classes[i] > 4 ||
            profile_offsets[i] < 0 || profile_offsets[i + 1] < profile_offsets[i]) {
          throw std::invalid_argument("scripted actor ID/class/profile offset is invalid");
        }
        ScriptedActor actor;
        actor.id = static_cast<std::uint32_t>(actor_ids[i]);
        actor.classification = static_cast<ScriptedActorClass>(actor_classes[i]);
        actor.initial_station_m = actor_stations[i];
        actor.initial_lateral_m = actor_laterals[i];
        actor.initial_speed_mps = actor_speeds[i];
        actor.accel_limit_mps2 = actor_accels[i];
        actor.hard_brake_start_s = brake_starts[i];
        actor.hard_brake_end_s = brake_ends[i];
        actor.trigger_ego_gap_m = trigger_gaps[i];
        actor.crossing_end_lateral_m = crossing_ends[i];
        actor.crossing_speed_mps = crossing_speeds[i];
        for (std::int64_t j = profile_offsets[i]; j < profile_offsets[i + 1]; ++j) {
          actor.speed_profile.emplace_back(profile_times[j], profile_speeds[j]);
        }
        actors.push_back(std::move(actor));
      }
      core_->set_scripted_actors(actors);
    }

    // 脚本化前车（ACC 场景注入，默认关闭）
    LeadScript lead;
    lead.enabled = declare_parameter<bool>("lead.enabled", false);
    lead.initial_station_m = declare_parameter<double>("lead.initial_station_m", 60.0);
    lead.initial_speed_mps = declare_parameter<double>("lead.initial_speed_mps", 10.0);
    lead.accel_mps2 = declare_parameter<double>("lead.accel_mps2", 2.0);
    const auto ev_times = declare_parameter<std::vector<double>>(
        "lead.event_times_s", std::vector<double>{});
    const auto ev_speeds = declare_parameter<std::vector<double>>(
        "lead.event_speeds_mps", std::vector<double>{});
    if (ev_times.size() != ev_speeds.size()) {
      throw std::invalid_argument("lead.event_times_s 与 event_speeds_mps 必须等长");
    }
    for (std::size_t i = 0; i < ev_times.size(); ++i) {
      lead.events.emplace_back(ev_times[i], ev_speeds[i]);
    }
    if (lead.enabled && !scripted_enabled) {
      core_->set_lead_script(lead);
    }

    // 脚本化邻道车（超车放弃场景注入，默认关闭）
    AdjacentCarScript adj;
    adj.enabled = declare_parameter<bool>("adjacent.enabled", false);
    adj.spawn_time_s = declare_parameter<double>("adjacent.spawn_time_s", 0.0);
    adj.initial_station_m = declare_parameter<double>("adjacent.initial_station_m", 0.0);
    adj.lateral_m = declare_parameter<double>("adjacent.lateral_m", 3.5);
    adj.speed_mps = declare_parameter<double>("adjacent.speed_mps", 10.0);
    if (adj.enabled && !scripted_enabled) {
      core_->set_adjacent_car_script(adj);
    }

    // 脚本化横穿行人（AEB 场景注入，默认关闭）
    PedestrianScript ped;
    ped.enabled = declare_parameter<bool>("pedestrian.enabled", false);
    ped.station_m = declare_parameter<double>("pedestrian.station_m", 150.0);
    ped.trigger_ego_gap_m = declare_parameter<double>("pedestrian.trigger_ego_gap_m", 35.0);
    ped.start_lateral_m = declare_parameter<double>("pedestrian.start_lateral_m", -5.0);
    ped.end_lateral_m = declare_parameter<double>("pedestrian.end_lateral_m", 5.0);
    ped.speed_mps = declare_parameter<double>("pedestrian.speed_mps", 1.5);
    if (ped.enabled && !scripted_enabled) {
      core_->set_pedestrian_script(ped);
    }

    // ── ROS 接口 ──
    const auto sensor_qos = rclcpp::SensorDataQoS();
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(
        "/adas/localization/kinematic_state", sensor_qos);
    pub_lane_ = create_publisher<adas_msgs::msg::LaneState>(
        "/adas/perception/lane_state", sensor_qos);
    pub_steer_ = create_publisher<adas_msgs::msg::SteeringReport>(
        "/adas/vehicle/steering_report", sensor_qos);
    pub_objects_ = create_publisher<adas_msgs::msg::TrackedObjectArray>(
        "/adas/perception/objects_raw", sensor_qos);

    sub_actuation_ = create_subscription<adas_msgs::msg::ActuationCommand>(
        "/adas/vehicle/actuation_cmd", rclcpp::QoS(1),
        [this](adas_msgs::msg::ActuationCommand::ConstSharedPtr msg) {
          actuation_.throttle = msg->throttle;
          actuation_.brake = msg->brake;
          actuation_.steer = msg->steer;
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz_),
        [this]() { on_timer(); });
    RCLCPP_INFO(get_logger(), "sim_vehicle 就绪：赛道 %zu 点, %.0fHz",
                core_->centerline().size(), rate_hz_);
  }

 private:
  void on_timer() {
    core_->step(actuation_, 1.0 / rate_hz_);
    const auto now = this->now();
    const auto s = core_->state();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = s.pose.x;
    odom.pose.pose.position.y = s.pose.y;
    odom.pose.pose.orientation.z = std::sin(s.pose.yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(s.pose.yaw / 2.0);
    odom.twist.twist.linear.x = s.velocity_mps;
    odom.twist.twist.angular.z = s.yaw_rate_rps;
    pub_odom_->publish(odom);

    const auto ls = core_->lane_state();
    adas_msgs::msg::LaneState lane;
    lane.header.stamp = now;
    lane.header.frame_id = "base_link";
    lane.valid = ls.valid;
    lane.lateral_offset = ls.lateral_offset;
    lane.heading_error = ls.heading_error;
    lane.curvature = ls.curvature;
    lane.lane_width = ls.lane_width;
    pub_lane_->publish(lane);

    adas_msgs::msg::SteeringReport steer;
    steer.header.stamp = now;
    steer.steering_tire_angle_rad = static_cast<float>(core_->steering_angle_rad());
    pub_steer_->publish(steer);

    // 原始目标（选举归 object_tracker，此处 primary_lead_id 恒为 -1）
    adas_msgs::msg::TrackedObjectArray objects;
    objects.header.stamp = now;
    objects.header.frame_id = "odom";
    objects.primary_lead_id = -1;
    for (const auto& actor : core_->snapshot_objects()) {
      adas_msgs::msg::TrackedObject o;
      o.id = actor.id;
      o.classification = static_cast<std::uint8_t>(actor.classification);
      o.pose.pose.position.x = actor.x;
      o.pose.pose.position.y = actor.y;
      o.pose.pose.orientation.z = std::sin(actor.yaw / 2.0);
      o.pose.pose.orientation.w = std::cos(actor.yaw / 2.0);
      o.twist.twist.linear.x = actor.v_mps;
      if (actor.classification == ScriptedActorClass::Pedestrian) {
        o.dimensions.x = 0.5;
        o.dimensions.y = 0.5;
        o.dimensions.z = 1.7;
      } else if (actor.classification == ScriptedActorClass::Bicycle) {
        o.dimensions.x = 1.8;
        o.dimensions.y = 0.6;
        o.dimensions.z = 1.7;
      } else {
        o.dimensions.x = 4.5;
        o.dimensions.y = 1.8;
        o.dimensions.z = 1.5;
      }
      objects.objects.push_back(o);
    }
    pub_objects_->publish(objects);
  }

  double rate_hz_{50.0};
  std::unique_ptr<SimVehicleCore> core_;
  common::ActuationData actuation_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<adas_msgs::msg::LaneState>::SharedPtr pub_lane_;
  rclcpp::Publisher<adas_msgs::msg::SteeringReport>::SharedPtr pub_steer_;
  rclcpp::Publisher<adas_msgs::msg::TrackedObjectArray>::SharedPtr pub_objects_;
  rclcpp::Subscription<adas_msgs::msg::ActuationCommand>::SharedPtr sub_actuation_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::sim

RCLCPP_COMPONENTS_REGISTER_NODE(adas::sim::SimVehicleNode)
