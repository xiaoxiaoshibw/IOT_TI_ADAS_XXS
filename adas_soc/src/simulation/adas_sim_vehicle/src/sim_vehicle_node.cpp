// adas_sim_vehicle 节点壳：参数装配 + 消息转换 + 定时器，算法全部在 core
#include <cmath>
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
    if (lead.enabled) {
      core_->set_lead_script(lead);
    }

    // 脚本化邻道车（超车放弃场景注入，默认关闭）
    AdjacentCarScript adj;
    adj.enabled = declare_parameter<bool>("adjacent.enabled", false);
    adj.spawn_time_s = declare_parameter<double>("adjacent.spawn_time_s", 0.0);
    adj.initial_station_m = declare_parameter<double>("adjacent.initial_station_m", 0.0);
    adj.lateral_m = declare_parameter<double>("adjacent.lateral_m", 3.5);
    adj.speed_mps = declare_parameter<double>("adjacent.speed_mps", 10.0);
    if (adj.enabled) {
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
    if (ped.enabled) {
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
    const auto lead = core_->lead_state();
    if (lead.present) {
      adas_msgs::msg::TrackedObject o;
      o.id = 1;
      o.classification = adas_msgs::msg::TrackedObject::CLASS_CAR;
      o.pose.pose.position.x = lead.x;
      o.pose.pose.position.y = lead.y;
      o.pose.pose.orientation.z = std::sin(lead.yaw / 2.0);
      o.pose.pose.orientation.w = std::cos(lead.yaw / 2.0);
      o.twist.twist.linear.x = lead.v_mps;
      o.dimensions.x = 4.5;
      o.dimensions.y = 1.8;
      o.dimensions.z = 1.5;
      objects.objects.push_back(o);
    }
    const auto adj = core_->adjacent_car_state();
    if (adj.present) {
      adas_msgs::msg::TrackedObject o;
      o.id = 3;
      o.classification = adas_msgs::msg::TrackedObject::CLASS_CAR;
      o.pose.pose.position.x = adj.x;
      o.pose.pose.position.y = adj.y;
      o.pose.pose.orientation.z = std::sin(adj.yaw / 2.0);
      o.pose.pose.orientation.w = std::cos(adj.yaw / 2.0);
      o.twist.twist.linear.x = adj.v_mps;
      o.dimensions.x = 4.5;
      o.dimensions.y = 1.8;
      o.dimensions.z = 1.5;
      objects.objects.push_back(o);
    }
    const auto ped = core_->pedestrian_state();
    if (ped.present) {
      adas_msgs::msg::TrackedObject o;
      o.id = 2;
      o.classification = adas_msgs::msg::TrackedObject::CLASS_PEDESTRIAN;
      o.pose.pose.position.x = ped.x;
      o.pose.pose.position.y = ped.y;
      o.pose.pose.orientation.z = std::sin(ped.yaw / 2.0);
      o.pose.pose.orientation.w = std::cos(ped.yaw / 2.0);
      o.twist.twist.linear.x = ped.v_mps;
      o.dimensions.x = 0.5;
      o.dimensions.y = 0.5;
      o.dimensions.z = 1.7;
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
