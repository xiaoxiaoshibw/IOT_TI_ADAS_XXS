#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "adas_carla_bridge/can_protocol.hpp"
#include "adas_msgs/msg/actuation_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace adas::carla_bridge {
namespace {

int open_socketcan(const std::string& interface_name) {
  const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw std::runtime_error("SocketCAN socket failed: " +
                             std::string(std::strerror(errno)));
  }
  ifreq request{};
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1U);
  if (ioctl(fd, SIOCGIFINDEX, &request) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("SocketCAN interface " + interface_name +
                             " unavailable: " + error);
  }
  constexpr can_filter filters[] = {
      {kMcuControlId, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
      {kMcuHeartbeatId, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
      {kMcuDiagId, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
      {kMcuE2eDiagId, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
  };
  if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("SocketCAN filter failed: " + error);
  }
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("SocketCAN nonblocking mode failed: " + error);
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    throw std::runtime_error("SocketCAN bind failed: " + error);
  }
  return fd;
}

}  // namespace

class CanFeedbackNode final : public rclcpp::Node {
 public:
  CanFeedbackNode() : Node("pc_can_feedback") {
    interface_name_ = declare_parameter<std::string>("can_interface", "can0");
    feedback_timeout_s_ =
        declare_parameter<double>("feedback_timeout_s", 0.1);
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/adas/pc/mcu_actuation");
    guard_ = std::make_unique<McuFeedbackGuard>(feedback_timeout_s_);
    socket_fd_ = open_socketcan(interface_name_);
    publisher_ = create_publisher<adas_msgs::msg::ActuationCommand>(
        output_topic_, rclcpp::QoS(1).reliable());
    receive_timer_ = create_wall_timer(std::chrono::milliseconds(1),
                                       [this]() { receive(); });
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                       [this]() { publish(); });
    RCLCPP_INFO(get_logger(),
                "C++ SocketCAN feedback active: %s -> %s (timeout %.3f s)",
                interface_name_.c_str(), output_topic_.c_str(),
                feedback_timeout_s_);
  }

  ~CanFeedbackNode() override {
    if (socket_fd_ >= 0) close(socket_fd_);
  }

 private:
  void receive() {
    for (int count = 0; count < 64; ++count) {
      can_frame raw{};
      const ssize_t size = read(socket_fd_, &raw, sizeof(raw));
      if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
      if (size != static_cast<ssize_t>(sizeof(raw))) {
        if (size < 0) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                "SocketCAN read failed: %s", std::strerror(errno));
        }
        return;
      }
      if ((raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
          raw.can_dlc != 8U) {
        ++rejected_frames_;
        continue;
      }
      Frame frame{raw.can_id & CAN_SFF_MASK, {}};
      std::copy_n(raw.data, 8U, frame.data.begin());
      if (!guard_->feed(frame)) ++rejected_frames_;
    }
  }

  void publish() {
    const auto snapshot = guard_->current();
    adas_msgs::msg::ActuationCommand message;
    message.header.stamp = now();
    message.throttle = static_cast<float>(snapshot.throttle);
    message.brake = static_cast<float>(snapshot.brake);
    message.steer = static_cast<float>(snapshot.steer);
    publisher_->publish(message);
    if (snapshot.invalid_latched || snapshot.stale) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "MCU feedback fail-closed: stale=%s latched=%s age=%.4f invalid=%lu",
          snapshot.stale ? "true" : "false",
          snapshot.invalid_latched ? "true" : "false", snapshot.age_s,
          static_cast<unsigned long>(snapshot.invalid_count));
    }
  }

  int socket_fd_{-1};
  std::string interface_name_;
  std::string output_topic_;
  double feedback_timeout_s_{0.1};
  std::uint64_t rejected_frames_{0U};
  std::unique_ptr<McuFeedbackGuard> guard_;
  rclcpp::Publisher<adas_msgs::msg::ActuationCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace adas::carla_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<adas::carla_bridge::CanFeedbackNode>());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "C++ CAN feedback startup failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
