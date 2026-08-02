#ifndef ADAS_GUI__PROCESS_MANAGER_HPP_
#define ADAS_GUI__PROCESS_MANAGER_HPP_

// CARLA 与桥接节点的进程生命周期管理（一键启动面板的后端）。
// 关键点：
//   · CARLA 就绪不是"进程起来了"，甚至不是"RPC 端口可连接"——TCP listen
//     只是 UE4 RPC 服务器起了监听 socket，服务器本身可能还没跑到能正确应答
//     get_world()/load_world() 的地步；CARLA 0.9.16 在这个窗口期收到请求
//     有 Shipping/RHIThread SIGSEGV 的实测记录（见 start_pc_stack_clean.sh、
//     preflight_check.hpp classify_gpu_quality 注释）。真正就绪判据复用
//     scripts/carla_readiness.py——与 start_pc_stack_clean.sh 的
//     wait_carla_ready 完全同一份脚本、同一套 RPC/版本/世界/多帧稳定窗口
//     校验，不在 GUI 里另造一套更弱的判据。
//   · CarlaUE4.sh 会派生真正的 UE4 子进程，停止时必须对进程组发信号，
//     否则只杀掉壳脚本、CARLA 残留占端口。
//   · 一键启动 = 启 CARLA → readiness 脚本判定稳定就绪 → 启桥；桥异常退出/
//     CARLA 退出/readiness 校验失败都通过状态信号显式呈现，不静默、不把
//     "进程已拉起"当成"可以启动控制链"。
// 本管理器只拉起/停止进程，不发布任何 ROS 控制话题（DEF-ARCH-002 不变）。

#include <QObject>
#include <QProcess>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

#include <functional>
#include <utility>

#include "launch_config.hpp"
#include "orin_stack_manager.hpp"
#include "secure_settings.hpp"
#include "single_instance.hpp"

namespace adas::gui {

enum class ProcState { Stopped, Starting, Running, Stopping, Failed };

QString proc_state_name(ProcState state);

// 单个受管进程：启动、进程组终止、日志行转发。
class ManagedProcess : public QObject {
  Q_OBJECT

 public:
  explicit ManagedProcess(QString tag, QObject* parent = nullptr);

  void start(const QString& program, const QStringList& arguments);
  void stop();  // SIGTERM 进程组，超时 SIGKILL
  void kill() { process_.kill(); }
  bool waitForFinished(int ms) { return process_.waitForFinished(ms); }
  ProcState state() const { return state_; }
  bool active() const {
    return state_ == ProcState::Starting || state_ == ProcState::Running ||
           state_ == ProcState::Stopping;
  }
  qint64 startedAtMs() const { return started_at_ms_; }

 signals:
  void stateChanged(adas::gui::ProcState state, const QString& detail);
  void logLine(const QString& tag, const QString& line);

 private:
  void set_state(ProcState state, const QString& detail = {});
  // stderr=true 时把行打 "ERROR" tag，让 LogDrawer 自动升级为故障事件；
  // 否则按子进程的 tag（"CARLA"/"桥接"）。
  void forward_output(bool stderr);

  QString tag_;
  QProcess process_;
  QTimer kill_timer_;
  ProcState state_{ProcState::Stopped};
  qint64 started_at_ms_{-1};
  QByteArray line_buffer_;
};

// 编排：CARLA(含端口探测) + 桥，提供一键启动/全部停止。
class ProcessManager : public QObject {
  Q_OBJECT

 public:
  explicit ProcessManager(QObject* parent = nullptr);
  ~ProcessManager() override;

  void startCarla(const LaunchConfig& config);
  void startBridge(const LaunchConfig& config);
  void startAll(const LaunchConfig& config);  // CARLA 就绪后自动带起桥；
                                               // 当 LaunchConfig::start_full_stack
                                               // =true 时走 Orin+CARLA+bridge 全流程
  void stopAll();
  void stopBridge();
  // 烧录 F280025C 固件（独立操作，与启栈分离；GUI 不驱动电源/复位）。
  // 完成后用户回到主界面按"启动完整系统"即可。
  void flashMcuFirmware(const QString& firmware_path);
  // 上层（MainWindow）注入绿灯判据；lambda 内读 TelemetryFreshness +
  // RosBridge 的 ever_nav_ 等指标，全 true 才算"全栈就绪"。
  using GreenLightCheck = std::function<bool()>;
  void setGreenLightCheck(GreenLightCheck check) { green_light_check_ = std::move(check); }
  bool hilManagerActive() const { return hil_manager_.state() != QProcess::NotRunning; }
  bool hasManagedProcesses() const {
    return hilManagerActive() || carla_.active() || bridge_.active();
  }
  bool hasManagedCarla() const { return hilManagerActive() || carla_.active(); }
  bool hasManagedBridge() const { return hilManagerActive() || bridge_.active(); }
  bool externalCarlaDetected() const { return external_carla_; }
  bool externalBridgeDetected() const { return external_bridge_; }
  // 由 RosBridge 提供同步 ROS graph 探针；启动前必须再次检查，避免 GUI
  // 重启后把仍在运行的外部 carla_bridge 再启动一份。
  void setBridgeProbe(std::function<bool()> probe) { bridge_probe_ = std::move(probe); }
  void setExternalCarlaDetected(bool detected);
  void setExternalBridgeDetected(bool detected);

  ProcState carlaState() const {
    if (carla_.state() == ProcState::Stopped && external_carla_) return ProcState::Running;
    if (carla_.state() == ProcState::Stopped && hilManagerActive()) return ProcState::Starting;
    return carla_.state();
  }
  bool carlaReady() const { return carla_ready_; }
  ProcState bridgeState() const {
    if (bridge_.state() == ProcState::Stopped && external_bridge_) return ProcState::Running;
    if (bridge_.state() == ProcState::Stopped && hilManagerActive()) return ProcState::Starting;
    return bridge_.state();
  }
  qint64 bridgeStartedAtMs() const { return bridge_.startedAtMs(); }

 signals:
  void carlaChanged(adas::gui::ProcState state, bool port_ready,
                    const QString& detail);
  void bridgeChanged(adas::gui::ProcState state, const QString& detail);
  void logLine(const QString& tag, const QString& line);
  // 全流程进度（驱动 LaunchPanel/MainWindow 状态栏文案）：
  //   stage ∈ {"can_link","orin_hil","carla","bridge","green_light","complete","failed"}
  //   detail 是人类可读中文，会被 MainWindow 状态栏直接渲染
  void stackProgress(const QString& stage, const QString& detail);
  // 首次启动缺 Orin ssh 密码：让 LaunchPanel 弹 QInputDialog 让用户填；
  // 填完后用户重新按"一键启动全流程"即可（密码已写入 secrets.ini）。
  void needsOrinCredentials(const QString& host, const QString& user);

 private slots:
  void on_orin_command_finished(OrinStackManager::Op op, int exit_code,
                                const QString& detail);
  void on_carla_state_changed_for_topology(adas::gui::ProcState state, bool ready,
                                           const QString& detail);
  void on_bridge_state_changed_for_topology(adas::gui::ProcState state,
                                             const QString& detail);
  void check_green_light();

 private:
  // 全流程阶段机（无栈压，用一个 enum + 上下文变量）
  enum class FullStage {
    Idle,
    SetupCanLink,
    StartOrinHil,
    StartCarla,
    WaitReadiness,
    StartBridge,
    WaitGreenLight,
    Complete,
    Failed,
  };
  void start_full_stack(const LaunchConfig& config);
  void fail_full_stack(const QString& stage, const QString& detail);
  bool full_stack_running() const;
  void advance_after_orin(OrinStackManager::Op op, int exit_code);
  void start_readiness_probe();
  void stop_readiness_probe();
  void on_readiness_finished(int exit_code, QProcess::ExitStatus exit_status);
  void start_release_stack(const LaunchConfig& config, const QString& script);
  void stop_release_stack(const QString& script);
  void forward_hil_output(QProcess& process, QByteArray& buffer, bool stderr);

  ManagedProcess carla_;
  ManagedProcess bridge_;
  QProcess hil_manager_;
  QProcess stop_helper_;
  OrinStackManager orin_;  // ssh 远端命令 + 本地 dslite.sh 烧录
  QByteArray hil_output_buffer_;
  QByteArray hil_error_buffer_;
  QByteArray stop_output_buffer_;
  QByteArray stop_error_buffer_;
  bool hil_stop_requested_{false};
  // 系统级单实例锁：本机（含 CLI 脚本）同一时刻只允许一个 CARLA 在启动/运行。
  SingleInstanceLock carla_lock_{carla_lock_path()};
  // carla_readiness.py 子进程：一次性校验 RPC/版本/世界/多帧稳定窗口，
  // 不是常驻轮询（脚本内部自带 interval/stabilization 重试逻辑）。
  QProcess readiness_probe_;
  QTimer readiness_timeout_timer_;
  QString readiness_output_path_;
  QByteArray readiness_stdout_;
  QByteArray readiness_stderr_;
  bool readiness_running_{false};
  quint64 startup_generation_{0};
  quint64 readiness_generation_{0};
  qint64 readiness_started_ms_{-1};
  LaunchConfig pending_config_;
  bool carla_ready_{false};
  bool bridge_pending_{false};  // 等 CARLA 就绪后自动启桥
  bool external_carla_{false};
  bool external_bridge_{false};
  std::function<bool()> bridge_probe_;

  // ---- 全流程编排上下文 ----
  FullStage full_stage_{FullStage::Idle};
  LaunchConfig full_config_;
  QString full_orin_password_;  // 临时持有，跑完即清
  QString full_orin_host_;
  QString full_orin_user_;
  GreenLightCheck green_light_check_;
  QTimer green_light_timer_;
  qint64 green_light_started_ms_{-1};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__PROCESS_MANAGER_HPP_
