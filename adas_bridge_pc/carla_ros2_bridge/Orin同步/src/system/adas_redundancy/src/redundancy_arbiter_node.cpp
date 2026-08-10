// adas_redundancy 生命周期节点：双栈 gate 输出仲裁 → 全局下发命令。
// 订阅 /primary、/backup 两套（重映射后的）gate cmd/status + AEB 状态，
// 100Hz 选源并发布 /adas/control/gate/control_cmd（vehicle_interface 的唯一上游）。
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "adas_common/cycle_time_monitor.hpp"
#include "adas_common/parameter_validation.hpp"
#include "adas_common/timing_monitor.hpp"
#include "adas_msgs/msg/aeb_status.hpp"
#include "adas_msgs/msg/control.hpp"
#include "adas_msgs/msg/gate_status.hpp"
#include "adas_redundancy/arbiter_core.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace adas::system {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RedundancyArbiterNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit RedundancyArbiterNode(const rclcpp::NodeOptions& options)
      : LifecycleNode("redundancy_arbiter", options) {
    declare_parameter<double>("rate_hz", 100.0);
    declare_parameter<double>("cmd_timeout_s", 0.06);
    declare_parameter<double>("recover_stable_s", 1.0);
    declare_parameter<double>("takeover_guard_s", 1.0);
    declare_parameter<double>("guard_steer_rate_rps", 0.25);
    declare_parameter<double>("guard_accel_rate_mps3", 4.0);
    declare_parameter<double>("aeb_brake_rate_mps3", 12.0);
    declare_parameter<double>("normal_steer_rate_rps", 0.8);
    declare_parameter<double>("normal_accel_rate_mps3", 10.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      rate_hz_ = get_parameter("rate_hz").as_double();
      params_.cmd_timeout_s = get_parameter("cmd_timeout_s").as_double();
      params_.recover_stable_s = get_parameter("recover_stable_s").as_double();
      params_.takeover_guard_s = get_parameter("takeover_guard_s").as_double();
      params_.guard_steer_rate_rps = get_parameter("guard_steer_rate_rps").as_double();
      params_.guard_accel_rate_mps3 = get_parameter("guard_accel_rate_mps3").as_double();
      params_.aeb_brake_rate_mps3 = get_parameter("aeb_brake_rate_mps3").as_double();
      params_.normal_steer_rate_rps = get_parameter("normal_steer_rate_rps").as_double();
      params_.normal_accel_rate_mps3 = get_parameter("normal_accel_rate_mps3").as_double();
      common::require_positive("rate_hz", rate_hz_);
      common::require_timeout_exceeds_period("cmd_timeout_s", params_.cmd_timeout_s,
                                             "rate_hz", rate_hz_);
      common::require_positive("takeover_guard_s", params_.takeover_guard_s);
      core_ = std::make_unique<ArbiterCore>(params_);

      subscribe_stack("/primary", primary_);
      subscribe_stack("/backup", backup_);
      pub_cmd_ = create_publisher<adas_msgs::msg::Control>(
          "/adas/control/gate/control_cmd", rclcpp::QoS(1).reliable());

      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_),
                                 [this]() { on_timer(); });
      timer_->cancel();
      timing_.set_budget_ms(1000.0 / rate_hz_);
      cycle_time_.configure(rate_hz_);
      diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
      diagnostics_->setHardwareID("soc-redundancy-arbiter");
      diagnostics_->add("runtime", [this](auto& stat) { produce_diagnostics(stat); });
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "redundancy_arbiter configure 失败: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    core_ = std::make_unique<ArbiterCore>(params_);
    timing_.reset();
    cycle_time_.reset();
    last_role_ = ActiveRole::kNone;
    pub_cmd_->on_activate();
    timer_->reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    if (timer_) timer_->cancel();
    if (pub_cmd_) pub_cmd_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
    release();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
    release();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
    if (timer_) timer_->cancel();
    return CallbackReturn::SUCCESS;
  }

 private:
  struct StackChannels {
    adas_msgs::msg::Control::ConstSharedPtr cmd;
    rclcpp::Time cmd_rx{0, 0, RCL_ROS_TIME};
    adas_msgs::msg::GateStatus::ConstSharedPtr status;
    adas_msgs::msg::AebStatus::ConstSharedPtr aeb;
    rclcpp::Subscription<adas_msgs::msg::Control>::SharedPtr sub_cmd;
    rclcpp::Subscription<adas_msgs::msg::GateStatus>::SharedPtr sub_status;
    rclcpp::Subscription<adas_msgs::msg::AebStatus>::SharedPtr sub_aeb;
  };

  void subscribe_stack(const std::string& prefix, StackChannels& ch) {
    ch.sub_cmd = create_subscription<adas_msgs::msg::Control>(
        prefix + "/adas/control/gate/control_cmd", rclcpp::QoS(1).reliable(),
        [this, &ch](adas_msgs::msg::Control::ConstSharedPtr msg) {
          ch.cmd = msg;
          ch.cmd_rx = now();
        });
    ch.sub_status = create_subscription<adas_msgs::msg::GateStatus>(
        prefix + "/adas/control/gate/status", rclcpp::QoS(1).reliable().transient_local(),
        [&ch](adas_msgs::msg::GateStatus::ConstSharedPtr msg) { ch.status = msg; });
    ch.sub_aeb = create_subscription<adas_msgs::msg::AebStatus>(
        prefix + "/adas/control/aeb/status", rclcpp::QoS(1).reliable().transient_local(),
        [&ch](adas_msgs::msg::AebStatus::ConstSharedPtr msg) { ch.aeb = msg; });
  }

  StackObservation observe(const StackChannels& ch) const {
    StackObservation obs;
    if (ch.cmd) {
      obs.received = true;
      obs.stamp_s = ch.cmd_rx.seconds();
      obs.cmd.lateral.steering_tire_angle_rad = ch.cmd->lateral.steering_tire_angle_rad;
      obs.cmd.lateral.rotation_rate_rad_s =
          ch.cmd->lateral.steering_tire_rotation_rate_rad_s;
      obs.cmd.longitudinal.velocity_mps = ch.cmd->longitudinal.velocity_mps;
      obs.cmd.longitudinal.acceleration_mps2 = ch.cmd->longitudinal.acceleration_mps2;
    }
    obs.nominal = ch.status &&
                  ch.status->selected_source != adas_msgs::msg::GateStatus::SOURCE_BUILTIN_STOP;
    obs.aeb_active =
        ch.aeb && ch.aeb->state == adas_msgs::msg::AebStatus::STATE_EMERGENCY;
    return obs;
  }

  void on_timer() {
    const common::ScopedTimingSample sample(timing_);
    ArbiterInputs in;
    in.now_s = now().seconds();
    in.dt = cycle_time_.tick();
    in.primary = observe(primary_);
    in.backup = observe(backup_);

    const auto d = core_->update(in);
    if (d.role != last_role_) {
      RCLCPP_WARN(get_logger(), "冗余切换: %d → %d (%s)", static_cast<int>(last_role_),
                  static_cast<int>(d.role), d.reason.c_str());
      last_role_ = d.role;
    }
    if (d.valid) {
      adas_msgs::msg::Control cmd;
      cmd.header.stamp = now();
      cmd.lateral.steering_tire_angle_rad =
          static_cast<float>(d.cmd.lateral.steering_tire_angle_rad);
      cmd.longitudinal.velocity_mps = static_cast<float>(d.cmd.longitudinal.velocity_mps);
      cmd.longitudinal.acceleration_mps2 =
          static_cast<float>(d.cmd.longitudinal.acceleration_mps2);
      pub_cmd_->publish(cmd);
    }
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const auto snap = timing_.snapshot();
    const auto cycle = cycle_time_.snapshot();
    if (snap.error) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "tick 超周期预算");
    } else if (snap.warning) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "tick 接近周期预算");
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "nominal");
    }
    stat.add("last_ms", snap.last_ms);
    stat.add("max_ms", snap.max_ms);
    stat.add("cycle_last_ms", cycle.last_raw_s * 1000.0);
    stat.add("cycle_average_ms", cycle.average_raw_s * 1000.0);
    stat.add("cycle_max_jitter_ms", cycle.max_abs_jitter_s * 1000.0);
    stat.add("cycle_clamped_samples", cycle.clamped_samples);
    stat.add("active_role", static_cast<int>(core_ ? core_->role() : ActiveRole::kNone));
    stat.add("primary_cmd_age_s",
             primary_.cmd ? (now() - primary_.cmd_rx).seconds() : -1.0);
    stat.add("backup_cmd_age_s", backup_.cmd ? (now() - backup_.cmd_rx).seconds() : -1.0);
  }

  void release() {
    timer_.reset();
    pub_cmd_.reset();
    primary_ = StackChannels{};
    backup_ = StackChannels{};
    diagnostics_.reset();
    core_.reset();
  }

  double rate_hz_{100.0};
  ArbiterParams params_;
  std::unique_ptr<ArbiterCore> core_;
  ActiveRole last_role_{ActiveRole::kNone};
  StackChannels primary_;
  StackChannels backup_;
  common::TimingMonitor timing_;
  common::CycleTimeMonitor cycle_time_{100.0};
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;

  rclcpp_lifecycle::LifecyclePublisher<adas_msgs::msg::Control>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::system

RCLCPP_COMPONENTS_REGISTER_NODE(adas::system::RedundancyArbiterNode)
