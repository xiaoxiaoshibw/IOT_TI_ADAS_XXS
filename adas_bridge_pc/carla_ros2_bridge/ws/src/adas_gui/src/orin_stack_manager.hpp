#ifndef ADAS_GUI__ORIN_STACK_MANAGER_HPP_
#define ADAS_GUI__ORIN_STACK_MANAGER_HPP_

// Orin 远程栈管理：CAN 链路 up/down + adas-hil.service start/ensure +
// 烧录 F280025C 固件。
// 设计原则：
//   · 一次只跑一条命令（serialized_ 标志）。sshpass + ssh 是阻塞子进程，
//     并发跑两条会令 ssh ControlMaster 误判、密码串扰、超时混乱。
//   · sudo 命令走 `echo $pwd | sudo -S <cmd>`，避免交互式 sudo 卡死。
//     这是 Linux/Orin 上事实标准（jetson 默认 password in /etc/sudoers
//     NOPASSWD 不一定开）。
//   · 烧录用 dslite.sh 而非 PowerShell DSLite GUI：GUI 跑在 Linux，
//     PowerShell 路径不可用（见 MCU 工具链 memory）。
//   · 命令拼装成 argv 列表（exec_format），不在 shell 字符串里手工拼
//     密码——即便密码含特殊字符也不会注入。
//   · 不持有密码明文：调用方把 password 临时塞进来，跑完即丢。
//
// 注意：Orin 上 `adas-can.service` 的 ExecStart=/usr/bin/true 是 inert
// 的（install_on_jetson.sh 不会 modprobe pcan 或 ip link set can1），
// 所以"启 adas-hil 之前必须先手工 ip link set can1 up type can bitrate
// 500000"是 GUI 的责任——这也是 setupCanLink 这个函数存在的原因。
// 如果跳过这一步 can_gateway_node 会在构造时抛异常、launch shutdown。

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTimer>

#include <functional>

namespace adas::gui {

class OrinStackManager : public QObject {
  Q_OBJECT

 public:
  enum class Op {
    SetupCanLink,   // ip link set can1 up type can bitrate <bitrate>
    StartHil,       // sudo systemctl start adas-hil.service
    EnsureHil,      // sudo systemctl start adas-hil.service（永不提供 stop）
    CheckCanLink,   // ip link show can1（只读，无副作用）
    FlashMcu,       // 调 ~/程序/ti/dslite.sh <firmware>
  };
  Q_ENUM(Op)

  explicit OrinStackManager(QObject* parent = nullptr);
  ~OrinStackManager() override;

  // 同步执行一条命令。返回 0 = 成功；非 0 = exit code。
  // 失败原因通过 opFinished 信号带回（detail = stderr 末尾几行）。
  // 命令正在跑时再调 start()：立即返回 -1 并 emit busy 信号。
  int start(Op op, const QString& host, const QString& user,
            const QString& password, const QString& firmware_path = {},
            int can_bitrate = 500000);

  bool busy() const { return serialized_; }
  void cancel();  // 发 SIGTERM 到当前进程；不会发 SIGKILL（避免烧录中途
                  // 强杀导致 F280025C brick）

  // 单条命令超时（毫秒）。sshpass 在某些 Orin 故障下（如 sudo -n 缺
  // NOPASSWD、RTNETLINK 失败但 ssh 会话没正常关闭）会卡住不退出，必须
  // 由超时强制 kill 并 emit finished（exit_code = -1）让上层状态机退出。
  // 烧录不走超时（destructive，宁可慢也不强杀）。
  static constexpr int kCommandTimeoutMs = 30000;

  // 命令行拼装：暴露给单元测试 / 上层 dry-run 验证；不真执行。
  static QStringList build_ssh_argv(const QString& host, const QString& user,
                                     const QString& command);
  static QStringList build_sshpass_argv(const QString& host, const QString& user,
                                        const QString& password,
                                        const QString& command);
  static QStringList build_flash_argv(const QString& firmware_path);

 signals:
  void started(Op op);
  void logLine(const QString& tag, const QString& line);
  void finished(Op op, int exit_code, const QString& detail);
  void busyChanged(bool busy);

 private slots:
  void on_stdout();
  void on_stderr();
  void on_finished(int exit_code, QProcess::ExitStatus status);

 private:
  void run_next();
  void set_busy(bool b);

  QProcess* proc_{nullptr};  // 指向 ssh_proc_ 或 sudo_proc_ 二者之一
  QProcess ssh_proc_;
  QProcess sudo_proc_;
  QTimer timeout_timer_;
  QString host_;
  QString user_;
  Op current_op_{Op::SetupCanLink};
  bool serialized_{false};
  QByteArray stdout_buffer_;
  QByteArray stderr_buffer_;
};

}  // namespace adas::gui

#endif  // ADAS_GUI__ORIN_STACK_MANAGER_HPP_
