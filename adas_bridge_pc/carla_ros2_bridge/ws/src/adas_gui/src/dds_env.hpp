#ifndef ADAS_GUI__DDS_ENV_HPP_
#define ADAS_GUI__DDS_ENV_HPP_

// HIL 拓扑：PC(CARLA+桥+GUI) ↔ Orin(SoC+CAN 网关) 全走直连内网 192.168.100.x。
// PC 是多网卡机（wifi + ZeroTier + 直连 Orin 的 USB 网卡），单靠 DDS 组播会绑到
// 错误网卡导致「发现得到、收不到数据」。common.sh 已给出正确口径（CycloneDDS +
// 绑直连网卡 + 单播 Peer + domain 43），但任何绕过 common.sh 的启动方式（桌面图标、
// 裸 `ros2 run`）都会退回默认 rmw 而收不到 Orin。这里把同一口径固化进 GUI 进程：
// 只在对应变量「未设置」时注入，故 common.sh/手动 export 仍然优先。
//
// 纯逻辑（build_cyclonedds_uri / pick_direct_iface）无系统调用，gtest 覆盖；
// apply_dds_defaults() 负责运行期采集网卡并 setenv，必须在 rclcpp::init 之前调用。

#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>

namespace adas::gui {

// 直连 Orin 的内网段（与 common.sh / adas-hil.env 的 192.168.100.x 一致）。
inline const char* kDirectSubnetPrefix() { return "192.168.100."; }
inline const char* kDefaultOrinPeer() { return "192.168.100.32"; }

// 生成与 common.sh 完全一致的 CycloneDDS 配置：绑定 iface + 单播 peer。
// peer 为空时省略 <Peers>（此时退化为该网卡上的组播发现）。
inline std::string build_cyclonedds_uri(const std::string& iface,
                                        const std::string& peer) {
  std::string uri = "<CycloneDDS><Domain><General><Interfaces>";
  uri += "<NetworkInterface name=\"" + iface + "\"/>";
  uri += "</Interfaces></General>";
  if (!peer.empty()) {
    uri += "<Discovery><Peers><Peer Address=\"" + peer + "\"/></Peers></Discovery>";
  }
  uri += "</Domain></CycloneDDS>";
  return uri;
}

// 从 (网卡名, IPv4) 列表里挑直连 Orin 的那块（IPv4 落在 192.168.100.x）。
// 找不到返回空串——调用方据此保持默认组播行为，不硬塞一个错网卡。
inline std::string pick_direct_iface(
    const std::vector<std::pair<std::string, std::string>>& ifaces) {
  const std::string prefix = kDirectSubnetPrefix();
  for (const auto& [name, ipv4] : ifaces) {
    if (ipv4.rfind(prefix, 0) == 0) return name;
  }
  return {};
}

// 采集本机全部 IPv4 网卡（跳过 loopback）。运行期用，不进纯逻辑测试。
inline std::vector<std::pair<std::string, std::string>> enumerate_ipv4_ifaces() {
  std::vector<std::pair<std::string, std::string>> out;
  struct ifaddrs* head = nullptr;
  if (getifaddrs(&head) != 0) return out;
  for (struct ifaddrs* ifa = head; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;
    if ((ifa->ifa_flags & IFF_LOOPBACK) != 0) continue;
    char buf[INET_ADDRSTRLEN] = {0};
    auto* sin = reinterpret_cast<struct sockaddr_in*>(ifa->ifa_addr);
    if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) continue;
    out.emplace_back(ifa->ifa_name ? ifa->ifa_name : "", buf);
  }
  freeifaddrs(head);
  return out;
}

// 仅当变量未设置时写入（不覆盖 common.sh / 用户 export）。
inline void setenv_if_unset(const char* key, const std::string& value) {
  if (std::getenv(key) == nullptr) {
    ::setenv(key, value.c_str(), /*overwrite=*/0);
  }
}

// 把 HIL 直连 DDS 口径固化进本进程。必须在 rclcpp::init() 之前调用，
// 否则 rmw 选择与 CycloneDDS 参与者已用旧值初始化。
// 网卡/Peer 可用 ADAS_DDS_IFACE / ADAS_DDS_PEER 覆盖；探测不到直连网卡时
// 只设 domain/rmw，不硬绑网卡（回退默认发现，避免误绑更糟）。
inline void apply_dds_defaults() {
  setenv_if_unset("ROS_DOMAIN_ID", "43");
  setenv_if_unset("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
  // 抑制 Humble(Orin)↔Jazzy(PC) 跨版本 type-hash 装饰性刷屏（不影响数据）。
  setenv_if_unset("RCUTILS_LOGGING_MIN_SEVERITY", "ERROR");

  if (std::getenv("CYCLONEDDS_URI") != nullptr) return;  // 已由 common.sh 给定

  const char* iface_override = std::getenv("ADAS_DDS_IFACE");
  std::string iface = iface_override ? iface_override
                                     : pick_direct_iface(enumerate_ipv4_ifaces());
  if (iface.empty()) return;  // 未探测到直连网卡：保持默认发现

  const char* peer_override = std::getenv("ADAS_DDS_PEER");
  std::string peer = peer_override ? peer_override : kDefaultOrinPeer();
  ::setenv("CYCLONEDDS_URI", build_cyclonedds_uri(iface, peer).c_str(),
           /*overwrite=*/0);
}

}  // namespace adas::gui

#endif  // ADAS_GUI__DDS_ENV_HPP_
