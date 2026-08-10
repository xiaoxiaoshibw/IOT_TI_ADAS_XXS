#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace adas::perception {

using namespace std::chrono_literals;

template <typename T>
diagnostic_msgs::msg::KeyValue value(const std::string& key, const T& data) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  std::ostringstream stream;
  stream << data;
  result.value = stream.str();
  return result;
}

class LidarPerceptionNode final : public rclcpp::Node {
 public:
  LidarPerceptionNode() : Node("lidar_perception") {
    topic_ = declare_parameter<std::string>(
        "input_topic", "/adas/sensors/front/points");
    expected_frame_ = declare_parameter<std::string>(
        "expected_frame", "lidar_front");
    expected_hz_ = declare_parameter<double>("expected_hz", 10.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 0.25);
    min_points_ = declare_parameter<int64_t>("min_points", 500);
    max_points_ = declare_parameter<int64_t>("max_points", 500000);
    if (!std::isfinite(expected_hz_) || expected_hz_ <= 0.0 ||
        !std::isfinite(stale_timeout_s_) || stale_timeout_s_ <= 0.0 ||
        min_points_ <= 0 || max_points_ < min_points_) {
      throw std::invalid_argument("invalid LiDAR perception parameters");
    }

    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));
    points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          on_points(std::move(message));
        });
    timer_ = create_wall_timer(1s, [this]() { publish_diagnostics(); });
    RCLCPP_INFO(get_logger(), "LiDAR ingress active: %s expected=%.1f Hz",
                topic_.c_str(), expected_hz_);
  }

 private:
  static bool has_float32_field(const sensor_msgs::msg::PointCloud2& cloud,
                                const char* name, std::uint32_t offset) {
    return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                       [name, offset](const auto& field) {
                         return field.name == name && field.offset == offset &&
                                field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                                field.count == 1U;
                       });
  }

  std::string validate(const sensor_msgs::msg::PointCloud2& cloud) const {
    if (cloud.header.frame_id != expected_frame_) return "unexpected_frame";
    if (cloud.height != 1U || cloud.point_step != 16U || cloud.is_bigendian) {
      return "unsupported_layout";
    }
    if (!has_float32_field(cloud, "x", 0U) ||
        !has_float32_field(cloud, "y", 4U) ||
        !has_float32_field(cloud, "z", 8U) ||
        !has_float32_field(cloud, "intensity", 12U)) {
      return "missing_xyzi_fields";
    }
    const auto points = static_cast<std::uint64_t>(cloud.width) * cloud.height;
    if (points < static_cast<std::uint64_t>(min_points_)) return "too_few_points";
    if (points > static_cast<std::uint64_t>(max_points_)) return "too_many_points";
    if (cloud.row_step != cloud.point_step * cloud.width ||
        cloud.data.size() != cloud.row_step * cloud.height) {
      return "invalid_data_size";
    }
    return {};
  }

  void on_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    const auto received = std::chrono::steady_clock::now();
    const std::string error = validate(*message);
    if (!error.empty()) {
      ++invalid_clouds_;
      last_error_ = error;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Rejected LiDAR cloud: %s", error.c_str());
      return;
    }
    if (received_once_) {
      const double interval_s =
          std::chrono::duration<double>(received - last_receive_).count();
      last_interval_ms_ = interval_s * 1000.0;
      const double expected_period_s = 1.0 / expected_hz_;
      if (interval_s > stale_timeout_s_) {
        ++stream_restarts_;
      } else {
        max_interval_ms_ = std::max(max_interval_ms_, last_interval_ms_);
      }
      if (interval_s > 1.5 * expected_period_s &&
          interval_s <= stale_timeout_s_) {
        estimated_drops_ += static_cast<std::uint64_t>(
            std::max(1.0, std::floor(interval_s / expected_period_s) - 1.0));
      }
    }
    last_receive_ = received;
    received_once_ = true;
    last_point_count_ = message->width * message->height;
    ++valid_clouds_;
    last_error_.clear();
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar_perception: ingress";
    status.hardware_id = "jetson-orin-nano";
    const double age_s = received_once_
                             ? std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - last_receive_)
                                   .count()
                             : -1.0;
    if (!received_once_ || age_s > stale_timeout_s_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = received_once_ ? "point cloud stale" : "no point cloud";
    } else if (!last_error_.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = last_error_;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "LiDAR point cloud healthy";
    }
    status.values = {
        value("topic", topic_),
        value("frame", expected_frame_),
        value("age_s", age_s),
        value("last_point_count", last_point_count_),
        value("valid_clouds", valid_clouds_),
        value("invalid_clouds", invalid_clouds_),
        value("estimated_drops", estimated_drops_),
        value("stream_restarts", stream_restarts_),
        value("last_interval_ms", last_interval_ms_),
        value("max_interval_ms", max_interval_ms_),
    };
    message.status.push_back(std::move(status));
    diagnostics_->publish(message);
  }

  std::string topic_;
  std::string expected_frame_;
  double expected_hz_{10.0};
  double stale_timeout_s_{0.25};
  std::int64_t min_points_{500};
  std::int64_t max_points_{500000};
  bool received_once_{false};
  std::chrono::steady_clock::time_point last_receive_{};
  std::uint64_t valid_clouds_{0U};
  std::uint64_t invalid_clouds_{0U};
  std::uint64_t estimated_drops_{0U};
  std::uint64_t stream_restarts_{0U};
  std::uint64_t last_point_count_{0U};
  double last_interval_ms_{0.0};
  double max_interval_ms_{0.0};
  std::string last_error_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace adas::perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<adas::perception::LidarPerceptionNode>());
  } catch (const std::exception& error) {
    std::cerr << "lidar_perception failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
