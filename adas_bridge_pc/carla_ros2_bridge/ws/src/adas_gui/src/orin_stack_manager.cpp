#include "orin_stack_manager.hpp"

#include <QDir>
#include <QFileInfo>
#include <QTimer>

namespace adas::gui {

namespace {

// bash 单引号包裹的字符串字面：内含的 ' 替换为 '\''
// 仅用于把密码喂进远端 `echo 'PWD' | sudo -S`，前提是远端 shell 是 bash。
// 现实：jetson 默认 bash，OK。
QString bash_quote(const QString& s) {
  QString out;
  out.reserve(s.size() + 2);
  out.append(QChar('\''));
  for (const QChar c : s) {
    if (c == QChar('\'')) {
      out.append(QStringLiteral("'\\''"));
    } else {
      out.append(c);
    }
  }
  out.append(QChar('\''));
  return out;
}

QString op_tag(OrinStackManager::Op op) {
  switch (op) {
    case OrinStackManager::Op::SetupCanLink: return QStringLiteral("Orin CAN");
    case OrinStackManager::Op::StartHil:     return QStringLiteral("Orin adas-hil");
    case OrinStackManager::Op::EnsureHil:    return QStringLiteral("Orin adas-hil");
    case OrinStackManager::Op::CheckCanLink: return QStringLiteral("Orin CAN");
    case OrinStackManager::Op::FlashMcu:     return QStringLiteral("MCU");
  }
  return QStringLiteral("Orin");
}

}  // namespace

OrinStackManager::OrinStackManager(QObject* parent) : QObject(parent) {
  // ssh 跑完 → finished 信号 → 清理 busy → 通知上层
  connect(&ssh_proc_, &QProcess::finished, this,
          [this](int code, QProcess::ExitStatus status) { on_finished(code, status); });
  // stdout/stderr 按行转发 LogDrawer
  connect(&ssh_proc_, &QProcess::readyReadStandardOutput, this, &OrinStackManager::on_stdout);
  connect(&ssh_proc_, &QProcess::readyReadStandardError, this, &OrinStackManager::on_stderr);
  // 超时强制退出：某些 Orin 故障（sudo -n 缺 NOPASSWD、ssh 会话挂起）会让
  // sshpass 不退出，必须由 timer 触发 kill 让状态机推进。
  timeout_timer_.setSingleShot(true);
  timeout_timer_.setInterval(kCommandTimeoutMs);
  connect(&timeout_timer_, &QTimer::timeout, this, [this]() {
    if (!proc_ || proc_->state() == QProcess::NotRunning) return;
    emit logLine(op_tag(current_op_),
                 QStringLiteral("⚠ 命令超时 %1s，强制 kill")
                     .arg(kCommandTimeoutMs / 1000));
    proc_->kill();  // 强杀（无 SIGTERM 等待，避免挂起更久）
  });
}

OrinStackManager::~OrinStackManager() {
  cancel();
}

void OrinStackManager::set_busy(bool b) {
  if (serialized_ == b) return;
  serialized_ = b;
  emit busyChanged(b);
}

void OrinStackManager::cancel() {
  if (!serialized_) return;
  if (proc_) {
    proc_->terminate();
    if (!proc_->waitForFinished(2000)) proc_->kill();
  }
}

void OrinStackManager::kill() {
  if (!serialized_ || !proc_) return;
  timeout_timer_.stop();
  proc_->kill();
}

int OrinStackManager::start(Op op, const QString& host, const QString& user,
                            const QString& password, const QString& firmware_path,
                            int can_bitrate) {
  if (busy()) return -1;
  if (host.isEmpty() || user.isEmpty() || password.isEmpty()) return -2;
  if (op == Op::FlashMcu && firmware_path.isEmpty()) return -3;

  current_op_ = op;
  host_ = host;
  user_ = user;
  stdout_buffer_.clear();
  stderr_buffer_.clear();

  // 远端命令拼装。SetupCanLink / StartHil / EnsureHil 都走 sudo（Orin 上
  // jetson 用户对网络接口 / systemd 无 root 权限）；用 `echo pwd | sudo -S
  // -p ''` 把密码从 ssh stdin 喂给 sudo，避免交互卡死。
  QString inner_cmd;
  switch (op) {
    case Op::SetupCanLink:
      // ip link set 需要 CAP_NET_ADMIN：jetson 默认无此能力，必须 sudo。
      // 用 `echo pwd | sudo -S -p ''` 把密码从 stdin 喂给 sudo，避免 sudo
      // 交互卡死（-p '' 抑制 prompt，-S 走 stdin）。密码经 bash_quote 转义
      // 含特殊字符也不会注入；ssh 走 BatchMode=yes + StrictHostKeyChecking=
      // accept-new，密码链路不落 ssh 的 stdout/stderr（无明文泄露）。
      //
      // 先 down 再 up：保证 can1 已存在时（如上次残留、busy 状态）也能幂等
      // 重置，不会被 "Device or resource busy" 阻断。down 失败吞掉（设备
      // 不存在不算错），up 是真实成败。
      inner_cmd = QStringLiteral("echo %1 | sudo -S -p '' bash -c "
                                 "'ip link set can1 down 2>/dev/null; "
                                 "ip link set can1 up type can bitrate %2' 2>&1")
                      .arg(bash_quote(password))
                      .arg(can_bitrate);
      break;
    case Op::StartHil:
      // Fire-and-forget：nohup systemctl start ... & + disown，bash 子进
      // 程立即返回 exit 0（不等 systemctl 是否真正启动）。SSH 因此立即
      // 结束，sshpass 立即退出，GUI 顶层立刻推进到启 CARLA 阶段。
      //
      // 不再做 systemctl is-active 检查（is-active 在 activating 状态会
      // 阻塞等直到完成，又触发"永远等 ros2 launch"问题）；service 启动
      // 失败由绿灯 timer 通过 /adas/mcu/status 是否 fresh 兜底——节点没起
      // 来就不会发 /adas/mcu/status，全流程 60s 超时后 fail_full_stack
      // 给出明确错误。
      inner_cmd = QStringLiteral("echo %1 | sudo -S -p '' bash -c "
                                 "'nohup systemctl start adas-hil.service "
                                 "</dev/null >/dev/null 2>&1 & disown; exit 0' 2>&1")
                      .arg(bash_quote(password));
      break;
    case Op::EnsureHil:
      // 常驻策略不暴露 stop：即使旧调用点请求“停止”，也只会确保服务运行。
      inner_cmd = QStringLiteral("echo %1 | sudo -S -p '' bash -c "
                                 "'nohup systemctl start adas-hil.service "
                                 "</dev/null >/dev/null 2>&1 & disown; exit 0' 2>&1")
                      .arg(bash_quote(password));
      break;
    case Op::CheckCanLink:
      inner_cmd = QStringLiteral("ip -brief link show can1 2>&1");
      break;
    case Op::FlashMcu:
      // 本地操作，不走 ssh
      break;
  }

  if (op == Op::FlashMcu) {
    proc_ = &ssh_proc_;  // 复用成员
    const QStringList argv = build_flash_argv(firmware_path);
    proc_->start(QStringLiteral("bash"), argv);
  } else {
    proc_ = &ssh_proc_;
    const QStringList argv = build_sshpass_argv(host, user, password, inner_cmd);
    proc_->start(QStringLiteral("sshpass"), argv);
  }

  // 烧录不挂超时（destructive，宁可慢也不强杀）
  if (op != Op::FlashMcu) {
    timeout_timer_.start();
  }

  set_busy(true);
  emit started(op);
  return 0;
}

QStringList OrinStackManager::build_ssh_argv(const QString& host,
                                             const QString& user,
                                             const QString& command) {
  // 第一个必须是字面 "ssh"：当 argv 被 sshpass 透传给 ssh 时，ssh[0] 才是
  // 可执行文件名（execvp 行为）。否则 sshpass 会把 -o 当成自己的选项并
  // 报 "invalid option -- 'o'"。
  return {
      QStringLiteral("ssh"),
      QStringLiteral("-o"), QStringLiteral("BatchMode=yes"),
      QStringLiteral("-o"), QStringLiteral("ConnectTimeout=8"),
      QStringLiteral("-o"), QStringLiteral("StrictHostKeyChecking=accept-new"),
      QStringLiteral("%1@%2").arg(user, host),
      command,
  };
}

QStringList OrinStackManager::build_sshpass_argv(const QString& host,
                                                const QString& user,
                                                const QString& password,
                                                const QString& command) {
  QStringList argv;
  argv << QStringLiteral("-p") << password;
  argv += build_ssh_argv(host, user, command);
  return argv;
}

QStringList OrinStackManager::build_flash_argv(const QString& firmware_path) {
  // dslite.sh 不再传 flash 子命令（用户 memory 明确）。
  // 用 -c 让 bash 跑命令字符串，<path> 用单引号包一层防路径含空格。
  return {
      QStringLiteral("-c"),
      QStringLiteral("~/程序/ti/dslite.sh '%1' 2>&1").arg(firmware_path),
  };
}

void OrinStackManager::on_stdout() {
  if (!proc_) return;
  stdout_buffer_.append(proc_->readAllStandardOutput());
  int idx;
  while ((idx = stdout_buffer_.indexOf('\n')) >= 0) {
    const QByteArray line = stdout_buffer_.left(idx);
    stdout_buffer_.remove(0, idx + 1);
    emit logLine(op_tag(current_op_), QString::fromUtf8(line).trimmed());
  }
}

void OrinStackManager::on_stderr() {
  if (!proc_) return;
  stderr_buffer_.append(proc_->readAllStandardError());
  int idx;
  while ((idx = stderr_buffer_.indexOf('\n')) >= 0) {
    const QByteArray line = stderr_buffer_.left(idx);
    stderr_buffer_.remove(0, idx + 1);
    // sudo 自身的 [sudo] password for xxx: 也走这里；用 stderr 转发
    // 让 LogDrawer 标红，方便用户看出问题。
    emit logLine(op_tag(current_op_), QString::fromUtf8(line).trimmed());
  }
}

void OrinStackManager::on_finished(int exit_code, QProcess::ExitStatus status) {
  timeout_timer_.stop();  // 正常退出要关掉超时 timer
  // 把残余字节冲出来（最后一行可能没有换行）
  if (!stdout_buffer_.isEmpty()) {
    emit logLine(op_tag(current_op_), QString::fromUtf8(stdout_buffer_).trimmed());
    stdout_buffer_.clear();
  }
  if (!stderr_buffer_.isEmpty()) {
    emit logLine(op_tag(current_op_), QString::fromUtf8(stderr_buffer_).trimmed());
    stderr_buffer_.clear();
  }
  // 超时 kill 后 QProcess 把 exit_code 标记成 -1（或 137 for SIGKILL）；
  // 统一标 "timeout" 让 ProcessManager 排错更清晰。
  QString detail;
  if (status == QProcess::CrashExit || exit_code < 0 || exit_code == 137) {
    detail = QStringLiteral("超时 / 进程被 kill (exit=%1)").arg(exit_code);
  } else if (exit_code != 0) {
    detail = QStringLiteral("exit=%1").arg(exit_code);
  }
  emit finished(current_op_, exit_code, detail);
  set_busy(false);
  proc_ = nullptr;
}

}  // namespace adas::gui
