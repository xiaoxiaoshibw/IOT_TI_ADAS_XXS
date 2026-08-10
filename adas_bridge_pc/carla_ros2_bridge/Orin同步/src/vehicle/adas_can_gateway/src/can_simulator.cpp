#include "adas_can_gateway/can_simulator.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace adas::can_gateway {

SimCanInterface::SimCanInterface(std::string interface_name)
    : interface_name_(std::move(interface_name)) {
  if (interface_name_.empty()) {
    throw std::invalid_argument("SocketCAN interface must not be empty");
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw std::runtime_error("SocketCAN socket failed: " +
                             std::string(std::strerror(errno)));
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1U);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
    const std::string error = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("SocketCAN interface " + interface_name_ +
                             " unavailable: " + error);
  }

  const can_filter filters[] = {
      {kMcuControlId, CAN_SFF_MASK},
      {kMcuHeartbeatId, CAN_SFF_MASK},
      {kMcuDiagId, CAN_SFF_MASK},
      {kMcuE2eDiagId, CAN_SFF_MASK},
      {adas::can_protocol::kCanIdMcuSessionStatus, CAN_SFF_MASK},
  };
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters,
                 sizeof(filters)) < 0) {
    const std::string error = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("SocketCAN filter failed: " + error);
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("SocketCAN nonblocking mode failed: " + error);
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("SocketCAN bind failed: " + error);
  }
}

SimCanInterface::~SimCanInterface() {
  if (socket_fd_ >= 0) close(socket_fd_);
}

void SimCanInterface::send(const Frame& frame) {
  can_frame raw{};
  raw.can_id = frame.id;
  raw.can_dlc = static_cast<__u8>(frame.data.size());
  std::copy(frame.data.begin(), frame.data.end(), raw.data);
  if (write(socket_fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    throw std::runtime_error("SocketCAN write failed: " +
                             std::string(std::strerror(errno)));
  }
}

std::vector<Frame> SimCanInterface::receive(std::size_t max_frames) {
  std::vector<Frame> frames;
  frames.reserve(max_frames);
  while (frames.size() < max_frames) {
    can_frame raw{};
    const ssize_t size = read(socket_fd_, &raw, sizeof(raw));
    if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (size != static_cast<ssize_t>(sizeof(raw))) {
      throw std::runtime_error("SocketCAN read failed: " +
                               std::string(std::strerror(errno)));
    }
    if ((raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
        raw.can_dlc != 8U) {
      continue;
    }
    Frame frame{raw.can_id & CAN_SFF_MASK, {}};
    std::copy_n(raw.data, 8U, frame.data.begin());
    frames.push_back(frame);
  }
  return frames;
}

}  // namespace adas::can_gateway
