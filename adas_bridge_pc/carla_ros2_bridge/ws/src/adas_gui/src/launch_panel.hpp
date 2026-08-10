#ifndef ADAS_GUI__LAUNCH_PANEL_HPP_
#define ADAS_GUI__LAUNCH_PANEL_HPP_

// 左侧启动控制面板（经压缩）。仅保留：
//   · 运行配置（场景/地图/控制源/CARLA 目录/低画质·无窗口）
//   · 进程状态灯（CARLA / 桥接 / 桥运行时长）
//   · 主操作：[启动完整系统] / [停止系统]（带确认弹窗）
//   · 高级操作（折叠）：仅 CARLA / 仅桥接 / 停桥接
//
// 进程日志不再在本面板渲染，由 MainWindow 将 ProcessManager::logLine 转接到
// 底部 LogDrawer。本面板只发出运行态变化信号。

#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QMap>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "process_manager.hpp"
#include "demo_presets.hpp"
#include "health_model.hpp"
#include "preflight_check.hpp"
#include "ros_bridge.hpp"
#include "widgets.hpp"  // LedIndicator

namespace adas::gui {

class LaunchPanel : public QWidget {
  Q_OBJECT

 public:
  explicit LaunchPanel(QWidget* parent = nullptr);
  ~LaunchPanel() override;

  bool eventFilter(QObject* watched, QEvent* event) override;

  LaunchConfig currentConfig() const;
  // 暴露 ProcessManager 供 MainWindow 转发 logLine 至底部日志抽屉。
  ProcessManager* processManager() { return &manager_; }

 public slots:
  void onHealthSnapshot(const QVector<adas::gui::GuiHealthStatus>& statuses);

 signals:
  // 桥进入运行中/停止时通知主窗口（用于状态栏当前场景显示 + 状态栏告警）
  void runStateChanged(bool bridge_running, const QString& scenario,
                        const QString& town);
  // 演示预设已触发：纯场景启动器，不再自动 arm 导航目标。
  // 导航目标必须由用户在地图上手动点击（MapView::goalRequested），
  // 保证"场景"与"导航"两个概念分离，避免 arm 期内的手动选点被覆盖。
  void demoPresetLaunched(const QString& scenario, const QString& town);

 private:
  void build_ui();
  void build_preset_card(QVBoxLayout* root);
  void applyPreset(const DemoPreset& preset);  // 设配置 + 一键启动（仅场景，无导航 arm）
  void save_settings() const;
  void restore_settings();
  void update_buttons();
  bool confirmFlashMcu(const QString& firmware_path);  // 烧录前二次确认
  void onFlashMcuClicked();     // 选文件 + 弹确认 + 调 manager_
  void onNeedsOrinCredentials(const QString& host, const QString& user);  // 弹密码框
  void setAdvancedVisible(bool visible);
  void setHealthRow(const QString& id, HealthState state, const QString& detail);
  // 启动前一键体检：Fail 项阻断（返回 false），Warn 项弹窗确认是否继续。
  bool runPreflightGate();

  ProcessManager manager_;
  QComboBox* scenario_combo_;
  QComboBox* town_combo_;
  QComboBox* source_combo_;
  QLineEdit* carla_root_edit_;
  QCheckBox* low_quality_check_;
  QCheckBox* offscreen_check_;
  QCheckBox* start_full_stack_check_;  // 默认不勾：只跑 PC 本地栈
  BusyButton* start_all_button_;
  BusyButton* stop_all_button_;
  QList<BusyButton*> preset_buttons_;  // 演示预设按钮，桥运行期间禁用
  BusyButton* active_start_button_{nullptr};
  // 高级调试折叠
  QPushButton* advanced_toggle_;
  QWidget* advanced_box_;
  BusyButton* start_carla_button_;
  BusyButton* start_bridge_button_;
  BusyButton* stop_bridge_button_;
  BusyButton* flash_mcu_button_;  // 高级折叠：调 dslite.sh 烧 F280025C
  // 状态灯
  LedIndicator* carla_light_;
  QLabel* carla_state_label_;
  LedIndicator* bridge_light_;
  QLabel* bridge_state_label_;
  QLabel* runtime_label_;
  QTimer runtime_timer_;
  QMap<QString, LedIndicator*> health_lights_;
  QMap<QString, QLabel*> health_values_;
};

}  // namespace adas::gui

#endif  // ADAS_GUI__LAUNCH_PANEL_HPP_
