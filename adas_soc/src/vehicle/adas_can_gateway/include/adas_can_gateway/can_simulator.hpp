#ifndef ADAS_CAN_GATEWAY__CAN_SIMULATOR_HPP_
#define ADAS_CAN_GATEWAY__CAN_SIMULATOR_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "adas_can_gateway/protocol.hpp"

namespace adas::can_gateway {

// Linux SocketCAN transport used by PC SIL.  Keeping this transport separate
// from the USB-CAN implementation makes the SIL path exercise the same wire
// frames without requiring PEAK/CANalyst hardware.
class SimCanInterface final {
 public:
  explicit SimCanInterface(std::string interface_name);
  ~SimCanInterface();

  SimCanInterface(const SimCanInterface&) = delete;
  SimCanInterface& operator=(const SimCanInterface&) = delete;

  void send(const Frame& frame);
  std::vector<Frame> receive(std::size_t max_frames = 32U);
  const std::string& interface_name() const { return interface_name_; }

 private:
  int socket_fd_{-1};
  std::string interface_name_;
};

}  // namespace adas::can_gateway

#endif  // ADAS_CAN_GATEWAY__CAN_SIMULATOR_HPP_
