#include "secure_settings.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace adas::gui {

namespace {

constexpr const char* kOrg = "adas";
constexpr const char* kApp = "adas_gui";
constexpr const char* kSecretsFile = "secrets.ini";

// 写完文件后必须严格 0600；其他权限视为不安全（其他用户能读 ssh 密码）。
bool ensure_owner_only_permissions(const QString& path, QString* detail) {
  QFile::Permissions perms = QFile::permissions(path);
  if (!(perms & QFileDevice::ReadOwner) || !(perms & QFileDevice::WriteOwner) ||
      (perms & QFileDevice::ReadGroup) || (perms & QFileDevice::WriteGroup) ||
      (perms & QFileDevice::ReadOther) || (perms & QFileDevice::WriteOther)) {
    if (!QFile::setPermissions(path, QFileDevice::ReadOwner | QFileDevice::WriteOwner)) {
      if (detail) {
        *detail = QStringLiteral("chmod 600 失败：%1").arg(path);
      }
      return false;
    }
  }
  return true;
}

}  // namespace

SecureSettings& SecureSettings::instance() {
  static SecureSettings inst;
  return inst;
}

QString SecureSettings::secretsFilePath() {
  const QString dir =
      QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
  // AppConfigLocation 已经返回 ~/.config/adas/adas_gui/，但若返回空就
  // 退到 $HOME/.config/adas/adas_gui/，再不行就用 /tmp。
  const QString base = dir.isEmpty()
                           ? QDir::homePath() + QStringLiteral("/.config/adas/adas_gui")
                           : dir;
  return base + QStringLiteral("/") + QString::fromLatin1(kSecretsFile);
}

QString SecureSettings::keyFor(const QString& host, const QString& user) const {
  // 用 / 分隔避免 host/user 含特殊字符冲撞 INI 节名
  return QStringLiteral("orin/%1/%2").arg(host, user);
}

SecureSettingsResult SecureSettings::loadOrinPassword(const QString& host,
                                                      const QString& user,
                                                      QString* password) const {
  SecureSettingsResult r;
  const QString path = secretsFilePath();
  if (!QFileInfo::exists(path)) {
    r.detail = QStringLiteral("secrets 文件不存在：%1\n请在 GUI 弹窗中填入 Orin ssh 密码")
                   .arg(path);
    return r;
  }
  if (!ensure_owner_only_permissions(path, &r.detail)) return r;

  QSettings ini(path, QSettings::IniFormat);
  const QString key = keyFor(host, user);
  const QString value = ini.value(key).toString();
  if (value.isEmpty()) {
    r.detail = QStringLiteral("secrets 中未配置 %1\n请在 GUI 弹窗中填入").arg(key);
    return r;
  }
  if (password) *password = value;
  r.ok = true;
  return r;
}

SecureSettingsResult SecureSettings::saveOrinPassword(const QString& host,
                                                      const QString& user,
                                                      const QString& password) {
  SecureSettingsResult r;
  const QString path = secretsFilePath();
  const QFileInfo fi(path);
  QDir().mkpath(fi.absolutePath());

  // 用 QSettings 自己管 INI 格式——避免手工拼装的格式与 QSettings 解析器
  // 兼容性踩坑（前者写 [orin] key=val，后者要求 key 走 beginGroup/setValue
  // 才有正确转义）。sync() 把内容落盘，再 chmod 600。
  QSettings ini(path, QSettings::IniFormat);
  ini.setValue(keyFor(host, user), password);
  ini.sync();
  if (ini.status() != QSettings::NoError) {
    r.detail = QStringLiteral("QSettings 同步失败：%1").arg(path);
    return r;
  }
  if (!ensure_owner_only_permissions(path, &r.detail)) return r;
  r.ok = true;
  return r;
}

}  // namespace adas::gui