// SecureSettings 的最小单元测试：用 QTemporaryDir 隔离 secrets.ini。
// 只测纯文件 IO，不测 ssh 集成（那个由 OrinStackManager 测试覆盖）。
#include <gtest/gtest.h>

#include <QDir>
#include <QFile>
#include <QStandardPaths>
#include <QTemporaryDir>

#include "secure_settings.hpp"

namespace {

using adas::gui::SecureSettings;

class SecureSettingsEnv : public ::testing::Test {
 protected:
  void SetUp() override {
    // XDG_CONFIG_HOME 指向临时目录，避免污染用户配置
    temp_dir_.setAutoRemove(true);
    QDir().mkpath(temp_dir_.path());
    qputenv("XDG_CONFIG_HOME", temp_dir_.path().toUtf8());
  }

  QTemporaryDir temp_dir_;
};

TEST_F(SecureSettingsEnv, SaveAndLoadRoundTrip) {
  SecureSettings& s = SecureSettings::instance();
  const QString host = QStringLiteral("192.168.100.32");
  const QString user = QStringLiteral("jetson");
  const QString pwd = QStringLiteral("yahboom");

  auto save = s.saveOrinPassword(host, user, pwd);
  ASSERT_TRUE(save.ok) << save.detail.toStdString();

  // 验证文件存在且权限 0600
  const QString path = SecureSettings::secretsFilePath();
  ASSERT_TRUE(QFile::exists(path));
  QFile::Permissions perms = QFile::permissions(path);
  EXPECT_TRUE(perms & QFileDevice::ReadOwner);
  EXPECT_TRUE(perms & QFileDevice::WriteOwner);
  EXPECT_FALSE(perms & QFileDevice::ReadGroup);
  EXPECT_FALSE(perms & QFileDevice::ReadOther);

  QString loaded;
  auto load = s.loadOrinPassword(host, user, &loaded);
  ASSERT_TRUE(load.ok) << load.detail.toStdString();
  EXPECT_EQ(loaded, pwd);
}

TEST_F(SecureSettingsEnv, LoadMissingFileReturnsFalse) {
  SecureSettings& s = SecureSettings::instance();
  QString loaded;
  auto load = s.loadOrinPassword(QStringLiteral("10.0.0.99"),
                                 QStringLiteral("u"), &loaded);
  EXPECT_FALSE(load.ok);
  EXPECT_FALSE(load.detail.isEmpty());
  EXPECT_TRUE(loaded.isEmpty());
}

TEST_F(SecureSettingsEnv, DifferentUsersDoNotCollide) {
  SecureSettings& s = SecureSettings::instance();
  ASSERT_TRUE(s.saveOrinPassword(QStringLiteral("h"), QStringLiteral("alice"),
                                 QStringLiteral("a-pwd")).ok);
  ASSERT_TRUE(s.saveOrinPassword(QStringLiteral("h"), QStringLiteral("bob"),
                                 QStringLiteral("b-pwd")).ok);

  QString a, b;
  ASSERT_TRUE(s.loadOrinPassword(QStringLiteral("h"), QStringLiteral("alice"), &a).ok);
  ASSERT_TRUE(s.loadOrinPassword(QStringLiteral("h"), QStringLiteral("bob"), &b).ok);
  EXPECT_EQ(a, QStringLiteral("a-pwd"));
  EXPECT_EQ(b, QStringLiteral("b-pwd"));
}

}  // namespace