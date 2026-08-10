// dds_env.hpp 纯逻辑单测：CycloneDDS 口径构造 + 直连网卡挑选。
// 口径须与 adas_pc/scripts/common.sh 的 CYCLONEDDS_URI 一致。
#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "dds_env.hpp"

namespace {

using adas::gui::build_cyclonedds_uri;
using adas::gui::apply_dds_defaults;
using adas::gui::pick_direct_iface;

TEST(DdsEnv, UriBindsInterfaceAndPeer) {
  const std::string uri = build_cyclonedds_uri("enx00e04c176a70", "192.168.100.32");
  // 与 common.sh 逐字一致：绑网卡 + 单播 peer。
  EXPECT_EQ(uri,
            "<CycloneDDS><Domain><General><Interfaces>"
            "<NetworkInterface name=\"enx00e04c176a70\"/>"
            "</Interfaces></General>"
            "<Discovery><Peers><Peer Address=\"192.168.100.32\"/></Peers></Discovery>"
            "</Domain></CycloneDDS>");
}

TEST(DdsEnv, UriWithoutPeerOmitsDiscovery) {
  const std::string uri = build_cyclonedds_uri("eth0", "");
  EXPECT_NE(uri.find("<NetworkInterface name=\"eth0\"/>"), std::string::npos);
  EXPECT_EQ(uri.find("<Peers>"), std::string::npos);
  EXPECT_EQ(uri.find("<Discovery>"), std::string::npos);
}

TEST(DdsEnv, PicksDirectSubnetInterface) {
  const std::vector<std::pair<std::string, std::string>> ifaces = {
      {"lo", "127.0.0.1"},
      {"wlp192s0", "10.12.149.186"},       // wifi
      {"zt6q3kbgiw", "10.218.44.2"},        // ZeroTier
      {"enx00e04c176a70", "192.168.100.1"}, // 直连 Orin
  };
  EXPECT_EQ(pick_direct_iface(ifaces), "enx00e04c176a70");
}

TEST(DdsEnv, ReturnsEmptyWhenNoDirectLink) {
  const std::vector<std::pair<std::string, std::string>> ifaces = {
      {"wlp192s0", "10.12.149.186"},
      {"zt6q3kbgiw", "10.218.44.2"},
  };
  EXPECT_TRUE(pick_direct_iface(ifaces).empty());
}

TEST(DdsEnv, LocalThreeMachineKeepsDefaultRmwUntouched) {
  const char* old_local = std::getenv("ADAS_LOCAL_THREE_MACHINE");
  const char* old_rmw = std::getenv("RMW_IMPLEMENTATION");
  const std::string saved_local = old_local ? old_local : "";
  const std::string saved_rmw = old_rmw ? old_rmw : "";
  ::setenv("ADAS_LOCAL_THREE_MACHINE", "1", 1);
  ::unsetenv("RMW_IMPLEMENTATION");
  apply_dds_defaults();
  EXPECT_EQ(std::getenv("RMW_IMPLEMENTATION"), nullptr);
  if (old_local) ::setenv("ADAS_LOCAL_THREE_MACHINE", saved_local.c_str(), 1);
  else ::unsetenv("ADAS_LOCAL_THREE_MACHINE");
  if (old_rmw) ::setenv("RMW_IMPLEMENTATION", saved_rmw.c_str(), 1);
  else ::unsetenv("RMW_IMPLEMENTATION");
}

}  // namespace
