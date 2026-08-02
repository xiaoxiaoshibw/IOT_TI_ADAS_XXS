#ifndef ADAS_GUI__SAFETY_PANEL_HPP_
#define ADAS_GUI__SAFETY_PANEL_HPP_

// 右侧安全状态栏：汇总 MCU 安全状态机、主源健康度、AEB/TTC、车速/横误差/
// 转角/制动量。独立 widget，由 MainWindow 透传各 ROS 信号槽更新。
//
// 视觉层（从上到下）：
//   1. 大号"安全状态标签 + 状态机名 + 控制源"卡片（与顶部 banner 类似但更紧凑）
//   2. 主源健康度：PRIMARY fresh（备源不在 GUI 展示）
//   3. 链路状态：MCU / 反馈年龄 / 协议 / CAN bus 状态
//   4. AEB + TTC
//   5. 车速、横摆角速度、转角、油门、制动、横误差（如有）
//
// 所有阈值与 main_window 旧版保持一致（kMcuStaleMs 等），遥测断流时显式置灰。

#include <QLabel>
#include <QListWidget>
#include <QProgressBar>
#include <QWidget>

#include "ros_bridge.hpp"  // GuiMcuStatus / GuiActuation
#include "telemetry_freshness.hpp"

class QCheckBox;

namespace adas::gui {

class StatusBanner;
class SegmentedBar;

class SafetyPanel : public QWidget {
  Q_OBJECT
 public:
  explicit SafetyPanel(QWidget* parent = nullptr);

  // 遥测各自新鲜度阈值 ms（与 MainWindow::onStaleCheck 同口径）。
  static constexpr qint64 kMcuStaleMs = 500;
  static constexpr qint64 kActuationStaleMs = 500;
  static constexpr qint64 kEgoStaleMs = 500;

 public slots:
  void onMcuStatus(const GuiMcuStatus& status);
  void onActuation(const GuiActuation& actuation);
  void onBehavior(int state, double target_speed_mps, int target_lane);
  void onGate(int source, bool limited, const QString& reason);
  void onAeb(int state, double ttc_s, double required_decel_mps2);
  void onSafety(int overall, const QString& failed);
  void onEgo(double x, double y, double yaw_rad, double speed_mps,
             double yaw_rate_rps);
  void onLaneState(double lateral_offset_m, bool valid);
  // DTC 历史（从 /mcu/dtc_history JSON 解析，仅展示 active 记录）
  void onDtcHistory(const QString& json);
  // 主窗口定时调用：刷新遥测新鲜度灰显
  void onStaleCheck(qint64 now_ms);

 signals:
  // 高危安全事件被点亮时通知主窗口告警条
  void alertChanged(const QString& key, const QString& text, bool active);
  // DTC 列表点击时主窗口可弹详情
  void dtcHistoryRequested();
  void faultEvent(int severity, const QString& source, const QString& title,
                  const QString& detail, bool recovered);

 private:
  void build_ui();
  void update_link_lights(const GuiMcuStatus& status, bool fresh);
  void set_value_label(QLabel* label, const QString& text, const char* color = nullptr);

  StatusBanner* banner_;
  QLabel* source_value_;
  QLabel* primary_value_;
  QLabel* mcu_value_;
  QLabel* can_value_;
  QLabel* protocol_value_;
  QLabel* handover_value_;
  QLabel* manual_value_;
  QLabel* aeb_value_;
  QLabel* ttc_value_;
  QLabel* safety_value_;
  QLabel* speed_value_;
  QLabel* behavior_value_;
  QLabel* gate_value_;
  QLabel* steer_value_;
  QLabel* lateral_value_;
  SegmentedBar* throttle_bar_;
  SegmentedBar* brake_bar_;
  QLabel* pose_value_;
  QListWidget* dtc_list_;

  GuiMcuStatus last_status_;
  bool have_status_{false};
  // mcu / actuation / ego 的新鲜度走 TelemetryFreshness 单例；这里只保留
  // 业务态字段。
  int last_aeb_state_{-1};
  int last_safety_level_{-1};
  // 接管流程可视化：控制源切换与驾驶员手动接管的时间轴（只读展示，GUI 不触发接管）
  int last_active_source_{-1};
  qint64 last_source_change_ms_{-1};
  bool last_manual_override_{false};
  qint64 manual_override_since_ms_{-1};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__SAFETY_PANEL_HPP_
