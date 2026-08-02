// OrinStackManager 命令行拼装单元测试。
// 只测 build_*_argv 的纯函数逻辑；不真 ssh（测试环境无 Orin，且
// sshpass/ssh 副作用不可控）。
#include <gtest/gtest.h>

#include <QString>
#include <QStringList>

#include "orin_stack_manager.hpp"

namespace {

using adas::gui::OrinStackManager;

TEST(OrinStackManager, SshArgvContainsHostUserAndCommand) {
  const QStringList argv =
      OrinStackManager::build_ssh_argv(QStringLiteral("192.168.100.32"),
                                       QStringLiteral("jetson"),
                                       QStringLiteral("ip link show can1"));
  // 第一个必须是 "ssh"：否则被 sshpass 透传时 execvp 找不到可执行文件名，
  // sshpass 会把后续的 -o 当成自己的选项并报错 "invalid option -- 'o'"。
  ASSERT_FALSE(argv.isEmpty());
  EXPECT_EQ(argv.first(), QStringLiteral("ssh"));
  EXPECT_TRUE(argv.contains(QStringLiteral("-o")));
  EXPECT_TRUE(argv.contains(QStringLiteral("BatchMode=yes")));
  EXPECT_TRUE(argv.contains(QStringLiteral("ConnectTimeout=8")));
  EXPECT_TRUE(argv.contains(QStringLiteral("StrictHostKeyChecking=accept-new")));
  EXPECT_TRUE(argv.contains(QStringLiteral("jetson@192.168.100.32")));
  EXPECT_EQ(argv.last(), QStringLiteral("ip link show can1"));
}

TEST(OrinStackManager, SshpassArgvPrependsPasswordFlag) {
  const QStringList argv =
      OrinStackManager::build_sshpass_argv(QStringLiteral("192.168.100.32"),
                                           QStringLiteral("jetson"),
                                           QStringLiteral("yahboom"),
                                           QStringLiteral("systemctl --no-pager"));
  EXPECT_EQ(argv.first(), QStringLiteral("-p"));
  EXPECT_EQ(argv.at(1), QStringLiteral("yahboom"));
  // sshpass -p pwd ssh -o ... user@host command：argv 里 "ssh" 必须出现，
  // 且位置在 -p pwd 之后（sshpass 看到 ssh 就把剩余透传给 ssh）。
  ASSERT_GE(argv.indexOf(QStringLiteral("ssh")), 2);
  EXPECT_TRUE(argv.contains(QStringLiteral("jetson@192.168.100.32")));
  EXPECT_EQ(argv.last(), QStringLiteral("systemctl --no-pager"));
}

TEST(OrinStackManager, FlashArgvUsesDsliteShPath) {
  const QStringList argv = OrinStackManager::build_flash_argv(
      QStringLiteral("/tmp/firmware.out"));
  // bash -c "<cmd>" 形式，第一项是 -c
  ASSERT_FALSE(argv.isEmpty());
  EXPECT_EQ(argv.first(), QStringLiteral("-c"));
  // dslite.sh 直接接 path（不要再传 flash 子命令，参见用户 memory）
  EXPECT_TRUE(argv.last().contains(QStringLiteral("dslite.sh")));
  EXPECT_TRUE(argv.last().contains(QStringLiteral("/tmp/firmware.out")));
  EXPECT_FALSE(argv.last().contains(QStringLiteral("flash ")))
      << "dslite.sh 不应再传 flash 子命令";
}

TEST(OrinStackManager, SshArgvUsesAcceptNewHostKeyChecking) {
  // 防止 GUI 首次连 Orin 时卡在 host key 确认（无人工 stdin）。
  const QStringList argv =
      OrinStackManager::build_ssh_argv(QStringLiteral("10.0.0.5"),
                                       QStringLiteral("u"),
                                       QStringLiteral("echo hi"));
  EXPECT_TRUE(argv.contains(QStringLiteral("StrictHostKeyChecking=accept-new")));
}

}  // namespace