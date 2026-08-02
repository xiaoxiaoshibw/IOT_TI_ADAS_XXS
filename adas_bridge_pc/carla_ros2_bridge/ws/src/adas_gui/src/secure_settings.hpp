#ifndef ADAS_GUI__SECURE_SETTINGS_HPP_
#define ADAS_GUI__SECURE_SETTINGS_HPP_

// 密码/路径等敏感信息的本地持久化封装。
// 设计取舍：
//   · 不加密（本地单用户文件 0600 即可，参考 ssh/sshpass 的做法）；
//     加密反而引入 KCM 钥匙链/口令解锁回路，对 GUI 一键启动是噪音。
//   · 配置文件：QStandardPaths::AppConfigLocation → ~/.config/adas/adas_gui/
//     secrets.ini，纯文本 INI（QSettings 格式），写完强制 chmod 600。
//   · 缺文件/权限不对：loadOrThrow 返回 false + 错误文案，让 GUI 弹窗让
//     用户填一次；不静默走默认值（默认密码 = 默认配置会让一键启动
//     风险变成"明文硬编码进代码"）。
//   · LaunchConfig 只持非敏感字段（host/user/firmware_path），密码只走
//     SecureSettings，避免 GUI 内部状态机到处拿密码。

#include <QString>

namespace adas::gui {

struct SecureSettingsResult {
  bool ok{false};
  QString detail;  // 失败原因（弹窗给用户看）
};

class SecureSettings {
 public:
  // 单例：secure_settings 状态稳定，没必要每次 new。
  static SecureSettings& instance();

  // 读取 Orin ssh 密码。失败：ok=false + detail。
  // 用 host+user 拼接 key，避免换 Orin 时残留密码被错用。
  SecureSettingsResult loadOrinPassword(const QString& host,
                                        const QString& user,
                                        QString* password) const;

  // 写密码并 chmod 600。失败：ok=false + detail。
  SecureSettingsResult saveOrinPassword(const QString& host,
                                        const QString& user,
                                        const QString& password);

  // 路径：~/.config/adas/adas_gui/secrets.ini（可被 GUI 弹窗展示）
  static QString secretsFilePath();

 private:
  SecureSettings() = default;
  QString keyFor(const QString& host, const QString& user) const;
};

}  // namespace adas::gui

#endif  // ADAS_GUI__SECURE_SETTINGS_HPP_