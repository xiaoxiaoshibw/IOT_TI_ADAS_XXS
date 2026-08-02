#ifndef ADAS_GUI__LOG_DRAWER_HPP_
#define ADAS_GUI__LOG_DRAWER_HPP_

// 底部可折叠日志抽屉。"按钮栏 + 三 Tab"结构：
//   进程日志 | CAN 报文 | 故障事件
// 折叠时仅留头部标题栏；展开时高度由 MainWindow 的 vertical splitter 控制，
// 默认约 180px，可拖拽。日志不再长期占用左侧大块空间。
//
// 信号槽口径与 ProcessManager::logLine(tag, line) 兼容。

#include <QListWidget>
#include <QMap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QWidget>

namespace adas::gui {

class RealtimePlot;

class LogDrawer : public QWidget {
  Q_OBJECT
 public:
  explicit LogDrawer(QWidget* parent = nullptr);

  // ProcessManager::logLine 的落点：tag 区分 CARLA / 桥接 / ERROR / WARN。
  void appendLog(const QString& tag, const QString& line);

  // 折叠 / 展开：仅切换内层 QTabWidget 显隐；高度由外层 splitter 自行决定。
  bool isExpanded() const { return expanded_; }

 public slots:
  void setExpanded(bool expanded);
  void toggleExpanded();
  void onEgoTelemetry(double x, double y, double yaw_rad,
                      double speed_mps, double yaw_rate_rps);
  void onBehaviorTelemetry(int state, double target_speed_mps, int target_lane);
  void onActuationTelemetry(double steer, double throttle, double brake);
  void onAebTelemetry(int state, double ttc_s, double required_decel_mps2);
  void onLaneTelemetry(double lateral_offset_m, bool valid);
  void onDtcHistory(const QString& json);
  void appendFaultEvent(qint64 timestamp_ms, int severity, const QString& source,
                        const QString& title, const QString& detail,
                        bool recovered = false);

 signals:
  void expandedChanged(bool expanded);

 private:
  void build_ui();
  void update_chevron();

  QTabWidget* tabs_;
  QPlainTextEdit* proc_log_;
  QPlainTextEdit* can_log_;            // CAN 原始帧调试输出（当前桥无原始帧话题）
  QListWidget* fault_log_;             // DTC / 状态迁移时间线
  RealtimePlot* speed_plot_;
  RealtimePlot* target_speed_plot_;
  RealtimePlot* yaw_rate_plot_;
  RealtimePlot* steer_plot_;
  RealtimePlot* throttle_plot_;
  RealtimePlot* brake_plot_;
  RealtimePlot* ttc_plot_;
  RealtimePlot* lateral_plot_;
  RealtimePlot* required_decel_plot_;
  QPushButton* toggle_button_;
  bool expanded_{true};
  QMap<QString, bool> dtc_active_;
  QMap<QString, int> dtc_occurrences_;
  bool have_dtc_snapshot_{false};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__LOG_DRAWER_HPP_
