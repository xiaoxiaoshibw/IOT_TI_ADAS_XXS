#ifndef ADAS_GUI__MAIN_WINDOW_HPP_
#define ADAS_GUI__MAIN_WINDOW_HPP_

#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QMap>
#include <QPushButton>
#include <QSplitter>
#include <QTimer>

#include "fault_inject_panel.hpp"
#include "launch_panel.hpp"
#include "log_drawer.hpp"
#include "map_view.hpp"
#include "ros_bridge.hpp"
#include "safety_panel.hpp"
#include "scenario_run_panel.hpp"
#include "telemetry_freshness.hpp"

namespace adas::gui {

class BusyButton;

// 主窗口：三栏一底部抽屉布局。GUI 只承担启动、导航和故障操控；
// 场景、路线、目标物与车辆状态全部在 CARLA 世界内展示。
//   ┌──────────────┬─────────────────────┬───────────────┐
//   │ 系统控制       │     导航操控         │  安全状态       │
//   │ (LaunchPanel) │   (control only)   │ (SafetyPanel) │
//   ├──────────────┴─────────────────────┴───────────────┤
//   │             日志抽屉 (LogDrawer)                    │
//   └────────────────────────────────────────────────────┘
//
// 顶部 alert_bar 仍保留聚合闪烁提示；状态栏文字统一中文 + LED 颜色规则：
// 绿=正常、黄=降级/预警、红=故障、灰=未启动。
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(RosBridge* bridge, QWidget* parent = nullptr);
  bool startConfiguredSystem(bool interactive = true);

 protected:
  void closeEvent(QCloseEvent* event) override;

 private slots:
  void onGoalRequested(double world_x, double world_y);
  void onLeadObject(const GuiLeadObject& lead);
  void onEgo(double x, double y, double yaw_rad, double speed_mps,
             double yaw_rate_rps);
  // 演示预设已触发：仅做操控反馈，场景画面交给 CARLA。
  void onDemoPresetLaunched(const QString& scenario, const QString& town);
  void onStaleCheck();

 private:
  QWidget* build_control_panel();
  void set_goal_controls_enabled(bool enabled);
  bool navigationInputsReady() const;
  bool submitGoal(double world_x, double world_y, bool require_confirmation);
  void tryAutoNavigation();
  void update_alert_bar();
  void update_hud_mode();
  void update_hud_fault();
  void set_alert(const QString& key, const QString& text, bool active);

  RosBridge* bridge_;
  QSplitter* columns_;          // 三栏水平 splitter
  QSplitter* rows_;             // 三栏 + 底部抽屉 垂直 splitter
  LaunchPanel* launch_panel_;
  SafetyPanel* safety_panel_;
  FaultInjectPanel* fault_inject_panel_;
  LogDrawer* log_drawer_;
  MapView* map_view_;
  ScenarioRunPanel* scenario_run_panel_;
  QDoubleSpinBox* goal_x_input_;
  QDoubleSpinBox* goal_y_input_;
  QDoubleSpinBox* forward_distance_input_;
  QPushButton* send_coordinate_goal_button_;
  QPushButton* send_forward_goal_button_;
  BusyButton* cancel_button_;
  QLabel* nav_status_value_;
  QLabel* alert_bar_;
  QLabel* hud_speed_value_;
  QLabel* hud_target_speed_value_;
  QLabel* hud_mode_value_;
  QLabel* hud_ttc_value_;
  QLabel* hud_lead_gap_value_;
  QLabel* hud_objects_value_;
  QLabel* hud_fault_value_;
  QLabel* scenario_label_;
  QLabel* config_label_;
  QLabel* stack_progress_label_;  // 全流程进度文案（来自 stackProgress）
  // 状态栏新鲜度圆点
  QLabel* dot_mcu_;
  QLabel* dot_actuation_;
  QLabel* dot_ego_;
  QLabel* dot_nav_;
  QTimer stale_timer_;
  QTimer alert_flash_timer_;
  bool alert_flash_on_{false};
  // 各遥测通道的新鲜度时间戳统一在 TelemetryFreshness 单例中维护，
  // 这里只保留需要 UI 状态机的 last_behavior_state_ / last_aeb_state_ /
  // last_safety_level_ 这类业务态字段。
  int last_behavior_state_{-1};
  int last_aeb_state_{0};
  int last_safety_level_{0};
  QStringList alert_order_;               // 告警插入顺序
  QMap<QString, QString> active_alerts_;  // key → 文本
  QMap<QString, HealthState> health_states_;
  bool have_health_snapshot_{false};
  QString active_goal_id_;
  QString pending_goal_request_id_;
  QString pending_cancel_request_id_;
  double latest_ego_x_{0.0};
  double latest_ego_y_{0.0};
  double latest_ego_yaw_{0.0};
  bool latest_ego_valid_{false};
  QVector<GuiLane> latest_lanes_;
  QString active_scenario_{QStringLiteral("free")};
  bool bridge_running_{false};
  bool auto_navigation_armed_{false};
  bool auto_navigation_retry_pending_{false};
  int auto_navigation_attempts_{0};
  quint64 auto_navigation_generation_{0};
  // P1.H: 一旦本会话已经自动下发过目标，立刻置为当前 run_id，阻止
  // tryAutoNavigation 二次发送；run_id 切换后该值被覆盖。
  QString auto_navigation_submitted_for_run_id_;
  quint64 latest_map_generation_{0};
  quint64 latest_ego_generation_{0};
  double auto_navigation_distance_m_{0.0};
  QString requested_town_;
  QString latest_map_id_;
  QString latest_map_hash_;
};

}  // namespace adas::gui

#endif  // ADAS_GUI__MAIN_WINDOW_HPP_
