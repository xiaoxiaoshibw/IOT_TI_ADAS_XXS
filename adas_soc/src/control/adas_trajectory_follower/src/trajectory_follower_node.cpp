// adas_trajectory_follower 生命周期节点：轨迹跟随控制与输入/耗时诊断。
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "adas_common/cycle_time_monitor.hpp"
#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/steering_report.hpp"
#include "adas_msgs/msg/trajectory.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"  // generated header (C++ name lowercased)
#include "adas_trajectory_follower/lqr_lateral.hpp"
#include "adas_trajectory_follower/pid_longitudinal.hpp"
#include "adas_trajectory_follower/pure_pursuit_lateral.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::control {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrajectoryFollowerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit TrajectoryFollowerNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("trajectory_follower", options) {
    declare_parameter<double>("rate_hz", 50.0);
    declare_parameter<double>("trajectory_timeout_s", 0.3);
    declare_parameter<double>("odom_timeout_s", 0.15);
    declare_parameter<std::string>("lateral_controller_mode", "pure_pursuit");
    declare_parameter<double>("pure_pursuit.wheelbase_m", 2.7);
    declare_parameter<double>("pure_pursuit.lookahead_gain_s", 0.8);
    declare_parameter<double>("pure_pursuit.min_lookahead_m", 3.0);
    declare_parameter<double>("pure_pursuit.max_lookahead_m", 20.0);
    declare_parameter<double>("pure_pursuit.max_steer_rad", 0.6);
    // Commit 4 — 自适应前视参数（自适应公式见 pure_pursuit_lateral.cpp）。
    // 历史 lookahead_gain_s 在 run() 中已被自适应公式替代，但保留以兼容。
    declare_parameter<double>("pure_pursuit.adaptive.base_speed_coeff", 0.7);
    declare_parameter<double>("pure_pursuit.adaptive.base_speed_offset_m", 2.0);
    declare_parameter<double>("pure_pursuit.adaptive.curve_gain", 4.0);
    declare_parameter<double>("pure_pursuit.adaptive.curve_factor_min", 0.6);
    declare_parameter<double>("pure_pursuit.adaptive.max_lookahead_high_m", 12.0);
    declare_parameter<double>("lqr.wheelbase_m", 2.7);
    declare_parameter<double>("lqr.steer_tau_s", 0.2);
    declare_parameter<double>("lqr.q_lat", 1.0);
    declare_parameter<double>("lqr.q_yaw", 1.0);
    declare_parameter<double>("lqr.r_steer", 30.0);
    declare_parameter<double>("lqr.max_steer_rad", 0.6);
    declare_parameter<double>("lqr.v_grid_step", 1.0);
    declare_parameter<double>("lqr.preview_s", 0.1);
    declare_parameter<std::string>("longitudinal_controller_mode", "pid");
    declare_parameter<double>("pid.kp", 1.0);
    declare_parameter<double>("pid.ki", 0.3);
    declare_parameter<double>("pid.kd", 0.0);
    declare_parameter<double>("pid.integral_limit", 2.0);
    declare_parameter<double>("pid.max_accel_mps2", 3.0);
    declare_parameter<double>("pid.max_decel_mps2", 4.0);
    declare_parameter<double>("pid.stop_speed_mps", 0.5);
    declare_parameter<double>("pid.start_speed_mps", 0.8);
    declare_parameter<double>("pid.stop_hold_accel_mps2", -1.5);
    declare_parameter<double>("pid.preview_time_s", 0.4);
    // Commit 6b — 积分冻结带（|error| < 此值跳过积分更新）
    declare_parameter<double>("pid.integrator_freeze_band_mps", 0.5);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      rate_hz_ = get_parameter("rate_hz").as_double();
      trajectory_timeout_s_ = get_parameter("trajectory_timeout_s").as_double();
      odom_timeout_s_ = get_parameter("odom_timeout_s").as_double();
      common::require_timeout_exceeds_period("trajectory_timeout_s", trajectory_timeout_s_,
                                             "rate_hz", rate_hz_);
      common::require_timeout_exceeds_period("odom_timeout_s", odom_timeout_s_, "rate_hz",
                                             rate_hz_);
      create_controllers();

      const auto sensor_qos = rclcpp::SensorDataQoS();
      sub_traj_ = create_subscription<adas_msgs::msg::Trajectory>(
          "/adas/planning/trajectory", rclcpp::QoS(1).reliable(),
          [this](adas_msgs::msg::Trajectory::ConstSharedPtr msg) { on_trajectory(msg); });
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/adas/localization/kinematic_state", sensor_qos,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            odom_ = msg;
            odom_rx_time_ = std::chrono::steady_clock::now();
            odom_received_ = true;
          });
      sub_steer_ = create_subscription<adas_msgs::msg::SteeringReport>(
          "/adas/vehicle/steering_report", sensor_qos,
          [this](adas_msgs::msg::SteeringReport::ConstSharedPtr msg) {
            steer_ = msg;
            // Commit 4 — 把当前真实转角送入 Pure Pursuit 内部 last_steer_。
            // 第一条 SteeringReport 到来后即播种；之后不影响（幂等）。
            if (lateral_) {
              auto* pp = dynamic_cast<PurePursuitLateral*>(lateral_.get());
              if (pp != nullptr) {
                pp->seed_from_steering_report(
                    static_cast<double>(msg->steering_tire_angle_rad));
              }
            }
          });
      // Commit 5 — 订阅 /adas/perception/objects 跟踪主前车 id 变化。
      // primary_lead_id 切换时重置 PID 积分（避免不同 lead 的积分状态串车）。
      sub_objects_ = create_subscription<adas_msgs::msg::TrackedObjectArray>(
          "/adas/perception/objects", sensor_qos,
          [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr msg) {
            objects_ = msg;
            objects_rx_time_ = std::chrono::steady_clock::now();
            objects_received_ = true;
          });
      pub_cmd_ = create_publisher<adas_msgs::msg::Control>(
          "/adas/control/trajectory_follower/control_cmd", rclcpp::QoS(1).reliable());
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      cycle_time_.configure(rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-trajectory-follower");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      parameters_valid_ = true;
      last_error_.clear();
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      parameters_valid_ = false;
      last_error_ = e.what();
      last_error_time_ = now();
      RCLCPP_ERROR(get_logger(), "trajectory_follower configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    trajectory_.clear();
    odom_.reset();
    steer_.reset();
    trajectory_received_ = false;
    odom_received_ = false;
    lateral_->reset();
    longitudinal_->reset();
    timing_.reset();
    cycle_time_.reset();
    output_count_ = 0U;
    output_enabled_ = true;
    pub_cmd_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    if (pub_cmd_) pub_cmd_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    release_resources();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    last_error_ = "lifecycle error";
    last_error_time_ = now();
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

 private:
  void create_controllers() {
    lateral_mode_ = get_parameter("lateral_controller_mode").as_string();
    if (lateral_mode_ == "pure_pursuit") {
      PurePursuitParams pp;
      pp.wheelbase_m = get_parameter("pure_pursuit.wheelbase_m").as_double();
      pp.lookahead_gain_s = get_parameter("pure_pursuit.lookahead_gain_s").as_double();
      pp.min_lookahead_m = get_parameter("pure_pursuit.min_lookahead_m").as_double();
      pp.max_lookahead_m = get_parameter("pure_pursuit.max_lookahead_m").as_double();
      pp.max_steer_rad = get_parameter("pure_pursuit.max_steer_rad").as_double();
      // Commit 4 — 自适应前视参数
      pp.base_speed_coeff =
          get_parameter("pure_pursuit.adaptive.base_speed_coeff").as_double();
      pp.base_speed_offset_m =
          get_parameter("pure_pursuit.adaptive.base_speed_offset_m").as_double();
      pp.curve_gain =
          get_parameter("pure_pursuit.adaptive.curve_gain").as_double();
      pp.curve_factor_min =
          get_parameter("pure_pursuit.adaptive.curve_factor_min").as_double();
      pp.max_lookahead_high_m =
          get_parameter("pure_pursuit.adaptive.max_lookahead_high_m").as_double();
      common::require_positive("pure_pursuit.wheelbase_m", pp.wheelbase_m);
      common::require_nonnegative("pure_pursuit.lookahead_gain_s", pp.lookahead_gain_s);
      common::require_positive("pure_pursuit.min_lookahead_m", pp.min_lookahead_m);
      common::require_positive("pure_pursuit.max_lookahead_m", pp.max_lookahead_m);
      if (pp.max_lookahead_m < pp.min_lookahead_m) {
        throw std::invalid_argument("pure_pursuit.max_lookahead_m must be >= min_lookahead_m");
      }
      common::require_positive("pure_pursuit.max_steer_rad", pp.max_steer_rad);
      common::require_nonnegative("pure_pursuit.adaptive.base_speed_coeff",
                                  pp.base_speed_coeff);
      common::require_nonnegative("pure_pursuit.adaptive.base_speed_offset_m",
                                  pp.base_speed_offset_m);
      common::require_nonnegative("pure_pursuit.adaptive.curve_gain", pp.curve_gain);
      if (pp.curve_factor_min <= 0.0 || pp.curve_factor_min > 1.0) {
        throw std::invalid_argument(
            "pure_pursuit.adaptive.curve_factor_min must be in (0, 1]");
      }
      common::require_positive("pure_pursuit.adaptive.max_lookahead_high_m",
                               pp.max_lookahead_high_m);
      if (pp.max_lookahead_high_m < pp.min_lookahead_m) {
        throw std::invalid_argument(
            "pure_pursuit.adaptive.max_lookahead_high_m must be >= min_lookahead_m");
      }
      lateral_ = std::make_unique<PurePursuitLateral>(pp);
    } else if (lateral_mode_ == "lqr") {
      LqrLateralParams lq;
      lq.wheelbase_m = get_parameter("lqr.wheelbase_m").as_double();
      lq.steer_tau_s = get_parameter("lqr.steer_tau_s").as_double();
      lq.dt_s = 1.0 / get_parameter("rate_hz").as_double();
      lq.q_lat = get_parameter("lqr.q_lat").as_double();
      lq.q_yaw = get_parameter("lqr.q_yaw").as_double();
      lq.r_steer = get_parameter("lqr.r_steer").as_double();
      lq.max_steer_rad = get_parameter("lqr.max_steer_rad").as_double();
      lq.v_grid_step = get_parameter("lqr.v_grid_step").as_double();
      lq.preview_s = get_parameter("lqr.preview_s").as_double();
      common::require_positive("lqr.wheelbase_m", lq.wheelbase_m);
      common::require_positive("lqr.v_grid_step", lq.v_grid_step);
      common::require_positive("lqr.q_lat", lq.q_lat);
      common::require_positive("lqr.q_yaw", lq.q_yaw);
      common::require_positive("lqr.r_steer", lq.r_steer);
      common::require_positive("lqr.max_steer_rad", lq.max_steer_rad);
      lateral_ = std::make_unique<LqrLateral>(lq);
    } else {
      throw std::invalid_argument(
          "lateral_controller_mode must be 'pure_pursuit' or 'lqr'");
    }

    longitudinal_mode_ = get_parameter("longitudinal_controller_mode").as_string();
    if (longitudinal_mode_ != "pid") {
      throw std::invalid_argument("longitudinal_controller_mode must be 'pid'");
    }
    PidLongitudinalParams pl;
    pl.kp = get_parameter("pid.kp").as_double();
    pl.ki = get_parameter("pid.ki").as_double();
    pl.kd = get_parameter("pid.kd").as_double();
    pl.integral_limit = get_parameter("pid.integral_limit").as_double();
    pl.max_accel_mps2 = get_parameter("pid.max_accel_mps2").as_double();
    pl.max_decel_mps2 = get_parameter("pid.max_decel_mps2").as_double();
    pl.stop_speed_mps = get_parameter("pid.stop_speed_mps").as_double();
    pl.start_speed_mps = get_parameter("pid.start_speed_mps").as_double();
    pl.stop_hold_accel_mps2 = get_parameter("pid.stop_hold_accel_mps2").as_double();
    pl.preview_time_s = get_parameter("pid.preview_time_s").as_double();
    pl.integrator_freeze_band_mps = get_parameter("pid.integrator_freeze_band_mps").as_double();
    common::require_nonnegative("pid.kp", pl.kp);
    common::require_nonnegative("pid.ki", pl.ki);
    common::require_nonnegative("pid.kd", pl.kd);
    common::require_positive("pid.integral_limit", pl.integral_limit);
    common::require_positive("pid.max_accel_mps2", pl.max_accel_mps2);
    common::require_positive("pid.max_decel_mps2", pl.max_decel_mps2);
    common::require_nonnegative("pid.stop_speed_mps", pl.stop_speed_mps);
    common::require_positive("pid.start_speed_mps", pl.start_speed_mps);
    if (pl.start_speed_mps <= pl.stop_speed_mps) {
      throw std::invalid_argument("pid.start_speed_mps must exceed pid.stop_speed_mps");
    }
    common::require_finite("pid.stop_hold_accel_mps2", pl.stop_hold_accel_mps2);
    if (pl.stop_hold_accel_mps2 >= 0.0) {
      throw std::invalid_argument("pid.stop_hold_accel_mps2 must be negative");
    }
    common::require_nonnegative("pid.preview_time_s", pl.preview_time_s);
    common::require_nonnegative("pid.integrator_freeze_band_mps",
                                pl.integrator_freeze_band_mps);
    longitudinal_ = std::make_unique<PidLongitudinal>(pl);
  }

  void on_trajectory(adas_msgs::msg::Trajectory::ConstSharedPtr msg) {
    trajectory_rx_time_ = std::chrono::steady_clock::now();
    trajectory_received_ = true;
    for (const auto& point : msg->points) {
      const auto& position = point.pose.position;
      const auto& orientation = point.pose.orientation;
      const double time_from_start = rclcpp::Duration(point.time_from_start).seconds();
      if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
          !std::isfinite(position.z) || !std::isfinite(orientation.z) ||
          !std::isfinite(orientation.w) ||
          !std::isfinite(point.longitudinal_velocity_mps) ||
          !std::isfinite(point.acceleration_mps2) ||
          !std::isfinite(point.curvature) || !std::isfinite(time_from_start)) {
        trajectory_.clear();
        inputs_valid_ = false;
        output_valid_ = false;
        last_error_ = "trajectory contains non-finite point";
        last_error_time_ = now();
        return;
      }
    }
    trajectory_.clear();
    trajectory_.reserve(msg->points.size());
    for (const auto& point : msg->points) {
      common::TrajPoint converted;
      converted.x = point.pose.position.x;
      converted.y = point.pose.position.y;
      converted.yaw =
          2.0 * std::atan2(point.pose.orientation.z, point.pose.orientation.w);
      converted.velocity_mps = point.longitudinal_velocity_mps;
      converted.acceleration_mps2 = point.acceleration_mps2;
      converted.curvature = point.curvature;
      converted.time_from_start_s = rclcpp::Duration(point.time_from_start).seconds();
      trajectory_.push_back(converted);
    }
  }

  double receive_age(const std::chrono::steady_clock::time_point& received,
                     bool present) const {
    if (!present) return -1.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - received).count();
  }

  bool fresh(const std::chrono::steady_clock::time_point& received, bool present,
             double timeout) const {
    const double age = receive_age(received, present);
    return age >= 0.0 && age < timeout;
  }

  void on_timer() {
    common::ScopedTimingSample sample(timing_);
    const double cycle_dt = cycle_time_.tick();
    if (trajectory_.size() < 2 || !odom_ ||
        !fresh(trajectory_rx_time_, trajectory_received_, trajectory_timeout_s_) ||
        !fresh(odom_rx_time_, odom_received_, odom_timeout_s_)) {
      inputs_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "轨迹/里程计缺失或过期，本拍不发控制指令");
      return;
    }
    inputs_valid_ = true;
    ControlInput input;
    input.trajectory = &trajectory_;
    input.state.pose.x = odom_->pose.pose.position.x;
    input.state.pose.y = odom_->pose.pose.position.y;
    input.state.pose.yaw =
        2.0 * std::atan2(odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
    input.state.velocity_mps = odom_->twist.twist.linear.x;
    input.state.yaw_rate_rps = odom_->twist.twist.angular.z;
    input.steering_angle_rad = steer_ ? steer_->steering_tire_angle_rad : 0.0;
    input.dt = cycle_dt;
    // Commit 5 — 主前车 id 切换时重置 PID 积分（lead A 累积的积分不应泄漏给 lead B）。
    // 仅在 primary_lead_id 实际变化时 reset，sticky retain 期内的"主车 id 不变"
    // 不会触发 reset。
    if (objects_ && fresh(objects_rx_time_, objects_received_, trajectory_timeout_s_)) {
      const int cur_lead = objects_->primary_lead_id;
      // 首次进入（last=-1）不触发 reset；后续变化则 reset。
      if (last_primary_lead_id_ != -1 && cur_lead != last_primary_lead_id_) {
        if (longitudinal_) longitudinal_->reset();
      }
      last_primary_lead_id_ = cur_lead;
    }
    const auto lateral = lateral_->run(input);
    const auto longitudinal = longitudinal_->run(input);
    output_valid_ = std::isfinite(lateral.steering_tire_angle_rad) &&
                    std::isfinite(lateral.rotation_rate_rad_s) &&
                    std::isfinite(longitudinal.velocity_mps) &&
                    std::isfinite(longitudinal.acceleration_mps2);
    if (!output_valid_) {
      last_error_ = "controller produced non-finite output";
      last_error_time_ = now();
      return;
    }
    adas_msgs::msg::Control cmd;
    cmd.header.stamp = now();
    cmd.lateral.steering_tire_angle_rad =
        static_cast<float>(lateral.steering_tire_angle_rad);
    cmd.lateral.steering_tire_rotation_rate_rad_s =
        static_cast<float>(lateral.rotation_rate_rad_s);
    cmd.longitudinal.velocity_mps = static_cast<float>(longitudinal.velocity_mps);
    cmd.longitudinal.acceleration_mps2 =
        static_cast<float>(longitudinal.acceleration_mps2);
    if (pub_cmd_ && pub_cmd_->is_activated()) {
      pub_cmd_->publish(cmd);
      ++output_count_;
    }
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto timing = timing_.snapshot();
    const auto cycle = cycle_time_.snapshot();
    const double trajectory_age = receive_age(trajectory_rx_time_, trajectory_received_);
    const double odom_age = receive_age(odom_rx_time_, odom_received_);
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string summary = "controller healthy";
    if (!parameters_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "invalid parameters";
    } else if (!output_enabled_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      summary = "inactive";
    } else if (!inputs_valid_ || !output_valid_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      summary = "input or output invalid";
    } else if (timing.warning) {
      level = timing.error ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                           : diagnostic_msgs::msg::DiagnosticStatus::WARN;
      summary = "processing budget warning";
    }
    stat.summary(level, summary);
    stat.add("lifecycle_state", get_current_state().label());
    stat.add("parameters_valid", parameters_valid_);
    stat.add("output_enabled", output_enabled_);
    stat.add("inputs_valid", inputs_valid_);
    stat.add("output_valid", output_valid_);
    stat.add("trajectory_age_s", trajectory_age);
    stat.add("odometry_age_s", odom_age);
    stat.add("steering_report_received", static_cast<bool>(steer_));
    stat.add("trajectory_point_count", trajectory_.size());
    stat.add("lateral_controller", lateral_mode_);
    stat.add("longitudinal_controller", longitudinal_mode_);
    const auto* pid = dynamic_cast<const PidLongitudinal*>(longitudinal_.get());
    stat.add("longitudinal_state", pid ? static_cast<int>(pid->state()) : -1);
    stat.add("output_count", output_count_);
    stat.add("processing_last_ms", timing.last_ms);
    stat.add("processing_average_ms", timing.average_ms);
    stat.add("processing_max_ms", timing.max_ms);
    stat.add("processing_budget_ms", timing.budget_ms);
    stat.add("cycle_last_ms", cycle.last_raw_s * 1000.0);
    stat.add("cycle_average_ms", cycle.average_raw_s * 1000.0);
    stat.add("cycle_max_jitter_ms", cycle.max_abs_jitter_s * 1000.0);
    stat.add("cycle_clamped_samples", cycle.clamped_samples);
    stat.add("last_error", last_error_);
    stat.add("last_error_time_s", last_error_time_.seconds());
  }

  void release_resources() {
    output_enabled_ = false;
    if (timer_) timer_->cancel();
    timer_.reset();
    diagnostics_.reset();
    pub_cmd_.reset();
    sub_traj_.reset();
    sub_odom_.reset();
    sub_steer_.reset();
    sub_objects_.reset();
    lateral_.reset();
    longitudinal_.reset();
    trajectory_.clear();
    odom_.reset();
    steer_.reset();
    objects_.reset();
    last_primary_lead_id_ = -1;
  }

  double rate_hz_{50.0};
  double trajectory_timeout_s_{0.3};
  double odom_timeout_s_{0.15};
  bool output_enabled_{false};
  bool parameters_valid_{false};
  bool inputs_valid_{false};
  bool output_valid_{true};
  std::size_t output_count_{0U};
  std::string lateral_mode_;
  std::string longitudinal_mode_;
  std::string last_error_;
  rclcpp::Time last_error_time_{0, 0, RCL_ROS_TIME};
  common::TimingMonitor timing_;
  common::CycleTimeMonitor cycle_time_;
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  std::unique_ptr<LateralControllerBase> lateral_;
  std::unique_ptr<LongitudinalControllerBase> longitudinal_;
  common::Trajectory trajectory_;
  std::chrono::steady_clock::time_point trajectory_rx_time_{};
  std::chrono::steady_clock::time_point odom_rx_time_{};
  bool trajectory_received_{false};
  bool odom_received_{false};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  adas_msgs::msg::SteeringReport::ConstSharedPtr steer_;
  // Commit 5 — 主前车 id 变化时重置 PID 积分（避免不同 lead 间的累积误差串车）。
  adas_msgs::msg::TrackedObjectArray::ConstSharedPtr objects_;
  std::chrono::steady_clock::time_point objects_rx_time_{};
  bool objects_received_{false};
  int last_primary_lead_id_{-1};
  rclcpp::Subscription<adas_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<adas_msgs::msg::SteeringReport>::SharedPtr sub_steer_;
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::Control>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::control

RCLCPP_COMPONENTS_REGISTER_NODE(adas::control::TrajectoryFollowerNode)
