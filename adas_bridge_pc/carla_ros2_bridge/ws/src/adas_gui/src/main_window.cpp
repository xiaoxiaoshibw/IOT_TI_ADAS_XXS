#include "main_window.hpp"

#include <algorithm>
#include <cmath>

#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QSettings>
#include <QSizePolicy>
#include <QStatusBar>
#include <QVBoxLayout>

#include "demo_presets.hpp"
#include "format.hpp"
#include "icons.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace adas::gui {
namespace {

constexpr qint64 kMcuStaleMs = 500;
constexpr qint64 kActuationStaleMs = 500;
constexpr qint64 kEgoStaleMs = 500;
constexpr qint64 kNavStaleMs = 3000;

LedIndicator* make_dot(const QString& label_text) {
  auto* led = new LedIndicator();
  led->setText(label_text);
  led->setState(LedState::Stale);
  return led;
}

void set_dot(QLabel* dot, bool fresh) {
  if (auto* led = qobject_cast<LedIndicator*>(dot)) {
    led->setState(fresh ? LedState::Ok : LedState::Stale);
  }
}

QPushButton* make_tool(const QString& icon_name, const QString& text,
                       const QString& object_name = QStringLiteral("iconButton")) {
  auto* b = new QPushButton();
  b->setIcon(icons::get(icon_name));
  b->setIconSize(QSize(16, 16));
  b->setObjectName(object_name);
  b->setFixedSize(34, 34);
  b->setToolTip(text);
  b->setAccessibleName(text);
  return b;
}

QFrame* make_hud_metric(const QString& icon_name, const QString& title,
                        QLabel** value_label, const QString& initial) {
  auto* card = new QFrame();
  card->setObjectName(QStringLiteral("hudMetric"));
  card->setMinimumHeight(62);
  card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  card->setStyleSheet(
      QStringLiteral("QFrame#hudMetric{background:%1;border:1px solid %2;"
                     "border-radius:8px;}QFrame#hudMetric:hover{border-color:%3;}")
          .arg(theme::kMdSurfaceContainerHigh, theme::kCardBorder,
               theme::kMdPrimary));

  auto* layout = new QVBoxLayout(card);
  layout->setContentsMargins(10, 8, 10, 8);
  layout->setSpacing(3);
  auto* label = new IconLabel(icon_name, title);
  label->setStyleSheet(QStringLiteral("color:%1;font-size:10px;font-weight:600;"
                                      "letter-spacing:0px;")
                           .arg(theme::kTextSecondary));
  *value_label = new QLabel(initial);
  (*value_label)->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  (*value_label)->setMinimumHeight(23);
  (*value_label)->setTextInteractionFlags(Qt::TextSelectableByMouse);
  (*value_label)->setStyleSheet(QStringLiteral("color:%1;font-size:18px;"
                                               "font-weight:700;letter-spacing:0px;")
                                    .arg(theme::kTextPrimary));
  layout->addWidget(label);
  layout->addWidget(*value_label);
  return card;
}

void set_hud_value(QLabel* label, const QString& text, const QString& color,
                   bool prominent = false) {
  if (!label) return;
  label->setText(text);
  label->setStyleSheet(QStringLiteral("color:%1;font-size:%2px;font-weight:%3;"
                                      "letter-spacing:0px;")
                           .arg(color)
                           .arg(prominent ? 20 : 18)
                           .arg(prominent ? 800 : 700));
}

}  // namespace

MainWindow::MainWindow(RosBridge* bridge, QWidget* parent)
    : QMainWindow(parent), bridge_(bridge) {
  setWindowTitle(QStringLiteral("ADAS 异构双控故障验证平台"));

  launch_panel_ = new LaunchPanel();
  safety_panel_ = new SafetyPanel();
  map_view_ = new MapView();
  log_drawer_ = new LogDrawer();
  alert_bar_ = new QLabel();
  alert_bar_->setObjectName(QStringLiteral("alertBar"));
  alert_bar_->setAlignment(Qt::AlignCenter);
  alert_bar_->setWordWrap(true);
  alert_bar_->hide();
  // 故障注入面板（审计整改 TOP10-2）：通过 ROS2 publish `/adas/_debug/fault_inject_cmd`
  // 把 0x301 命令发给桥节点；桥节点订阅后通过 PC CANalyst-II 发送 0x301 帧到 MCU。
  fault_inject_panel_ = new FaultInjectPanel(
      [bridge](int cmd, int param, const QString& label) {
        bridge->publishFaultInjectCommand(cmd, param, label);
      });
  launch_panel_->processManager()->setBridgeProbe(
      [bridge]() { return bridge->hasCarlaBridgeNode(); });

  // 三栏水平 splitter：左紧凑、中拉伸、右垂直可拖（安全 + 故障注入）。
  // 右栏内部用 QSplitter(Qt::Vertical) 而不是写死的 QVBoxLayout，让操作员在
  // bench 上能手动把故障注入区拖大；同时给 safety / fault 各自 minHeight
  // 防止被另一侧完全挤没。
  auto* right_column = new QSplitter(Qt::Vertical, this);
  right_column->setContentsMargins(0, 0, 0, 0);
  right_column->setChildrenCollapsible(false);
  right_column->setHandleWidth(2);
  right_column->addWidget(safety_panel_);
  right_column->addWidget(fault_inject_panel_);
  right_column->setStretchFactor(0, 1);  // safety 优先伸展
  right_column->setStretchFactor(1, 0);  // fault 最小
  right_column->setSizes({520, 280});
  safety_panel_->setMinimumHeight(220);
  fault_inject_panel_->setMinimumHeight(200);

  columns_ = new QSplitter(Qt::Horizontal, this);
  columns_->addWidget(launch_panel_);
  columns_->addWidget(build_map_panel());
  columns_->addWidget(right_column);
  columns_->setStretchFactor(0, 0);
  columns_->setStretchFactor(1, 1);   // map 主伸展
  columns_->setStretchFactor(2, 1);   // 右栏也分一份额外宽度，按钮不再被挤
  columns_->setChildrenCollapsible(false);
  columns_->setHandleWidth(2);
  columns_->setSizes({310, 820, 310});

  // 垂直 splitter：上面三栏，下面日志抽屉
  rows_ = new QSplitter(Qt::Vertical, this);
  rows_->addWidget(columns_);
  rows_->addWidget(log_drawer_);
  rows_->setStretchFactor(0, 1);
  rows_->setStretchFactor(1, 0);
  rows_->setChildrenCollapsible(false);
  rows_->setHandleWidth(2);
  rows_->setSizes({680, 150});
  setCentralWidget(rows_);

  // 状态栏：中文 + LED 颜色规则（绿/黄/红/灰）
  dot_mcu_ = make_dot(QStringLiteral("MCU"));
  dot_actuation_ = make_dot(QStringLiteral("执行器"));
  dot_ego_ = make_dot(QStringLiteral("里程计"));
  dot_nav_ = make_dot(QStringLiteral("导航"));
  scenario_label_ = new QLabel(QStringLiteral("场景: --"));
  scenario_label_->setStyleSheet(
      QStringLiteral("color:%1;font-size:11px;padding:3px 8px;border-radius:5px;"
                     "background:%2;border:1px solid %3;")
          .arg(theme::kMdOnPrimaryContainer, theme::kMdPrimaryContainer,
               theme::kCardBorder));
  config_label_ = new QLabel(QStringLiteral("配置: --"));
  config_label_->setStyleSheet(scenario_label_->styleSheet());
  // 全流程进度文案（来自 ProcessManager::stackProgress）。初始空，运行
  // 全流程时被 stage 文案覆盖（complete/failed 时变绿/红）。
  stack_progress_label_ = new QLabel(QString());
  stack_progress_label_->setStyleSheet(
      QStringLiteral("color:%1;font-size:11px;padding:3px 8px;border-radius:5px;"
                     "background:%2;border:1px solid %3;")
          .arg(theme::kMdOnPrimaryContainer, theme::kMdSurfaceContainerLow,
               theme::kCardBorder));
  stack_progress_label_->hide();

  statusBar()->addWidget(dot_mcu_);
  statusBar()->addWidget(dot_actuation_);
  statusBar()->addWidget(dot_ego_);
  statusBar()->addWidget(dot_nav_);
  statusBar()->addPermanentWidget(stack_progress_label_);
  statusBar()->addPermanentWidget(scenario_label_);
  statusBar()->addPermanentWidget(config_label_);
  statusBar()->setSizeGripEnabled(false);

  // ===== 信号绑定 =====
  // MCU / actuation / safety 在 SafetyPanel 内部展示；状态栏 LED 在 MainWindow 维护
  connect(bridge, &RosBridge::mcuStatusChanged, safety_panel_, &SafetyPanel::onMcuStatus);
  connect(bridge, &RosBridge::actuationChanged, safety_panel_, &SafetyPanel::onActuation);
  connect(bridge, &RosBridge::actuationChanged, this,
          [this](const GuiActuation& actuation) {
            log_drawer_->onActuationTelemetry(actuation.steer, actuation.throttle,
                                              actuation.brake);
          });
  connect(bridge, &RosBridge::dtcHistoryChanged, safety_panel_, &SafetyPanel::onDtcHistory);
  connect(bridge, &RosBridge::dtcHistoryChanged, log_drawer_, &LogDrawer::onDtcHistory);
  connect(bridge, &RosBridge::behaviorChanged, safety_panel_, &SafetyPanel::onBehavior);
  connect(bridge, &RosBridge::behaviorChanged,
          log_drawer_, &LogDrawer::onBehaviorTelemetry);
  connect(bridge, &RosBridge::behaviorChanged, this,
          [this](int state, double target_speed_mps, int) {
            TelemetryFreshness::instance().markFresh(TelemetryFreshness::Behavior);
            last_behavior_state_ = state;
            if (std::isfinite(target_speed_mps)) {
              set_hud_value(hud_target_speed_value_,
                            QStringLiteral("%1 km/h")
                                .arg(std::max(0.0, target_speed_mps) * 3.6,
                                     0, 'f', 0),
                            theme::kTextPrimary);
            }
            update_hud_mode();
          });
  connect(bridge, &RosBridge::gateChanged, safety_panel_, &SafetyPanel::onGate);
  connect(bridge, &RosBridge::aebChanged, safety_panel_, &SafetyPanel::onAeb);
  connect(bridge, &RosBridge::aebChanged, log_drawer_, &LogDrawer::onAebTelemetry);
  connect(bridge, &RosBridge::aebChanged, this,
          [this](int state, double ttc_s, double) {
            TelemetryFreshness::instance().markFresh(TelemetryFreshness::Aeb);
            last_aeb_state_ = state;
            if (std::isfinite(ttc_s) && ttc_s > 0.0 && ttc_s < 1.0e5) {
              const QString color = ttc_s < 1.5 ? theme::kDanger
                                  : ttc_s < 3.0 ? theme::kWarn
                                                 : theme::kOk;
              set_hud_value(hud_ttc_value_,
                            QStringLiteral("%1 s").arg(ttc_s, 0, 'f', 1),
                            color, ttc_s < 3.0);
            } else {
              set_hud_value(hud_ttc_value_, QStringLiteral("-- s"), theme::kTextSecondary);
            }
            update_hud_mode();
          });
  connect(bridge, &RosBridge::safetyChanged, safety_panel_, &SafetyPanel::onSafety);
  connect(bridge, &RosBridge::safetyChanged, this,
          [this](int overall, const QString&) {
            last_safety_level_ = overall;
            update_hud_mode();
            update_hud_fault();
          });
  connect(bridge, &RosBridge::laneStateChanged, safety_panel_, &SafetyPanel::onLaneState);
  connect(bridge, &RosBridge::laneStateChanged, log_drawer_, &LogDrawer::onLaneTelemetry);
  connect(bridge, &RosBridge::healthSnapshotChanged,
          launch_panel_, &LaunchPanel::onHealthSnapshot);
  connect(safety_panel_, &SafetyPanel::alertChanged, this, &MainWindow::set_alert);
  connect(safety_panel_, &SafetyPanel::faultEvent, this,
          [this](int severity, const QString& source, const QString& title,
                 const QString& detail, bool recovered) {
            log_drawer_->appendFaultEvent(QDateTime::currentMSecsSinceEpoch(), severity,
                                          source, title, detail, recovered);
          });
  connect(bridge, &RosBridge::healthSnapshotChanged, this,
          [this](const QVector<GuiHealthStatus>& statuses) {
            for (const auto& status : statuses) {
              if (!health_states_.contains(status.id)) {
                health_states_.insert(status.id, status.state);
                continue;
              }
              const HealthState previous = health_states_.value(status.id);
              if (previous == status.state) continue;
              if (health_is_problem(status.state)) {
                log_drawer_->appendFaultEvent(
                    QDateTime::currentMSecsSinceEpoch(), health_severity(status.state),
                    status.display_name,
                    QStringLiteral("健康状态变为 %1")
                        .arg(QString::fromUtf8(health_state_name(status.state))),
                    status.detail, false);
              } else if (health_is_problem(previous)) {
                log_drawer_->appendFaultEvent(
                    QDateTime::currentMSecsSinceEpoch(), 0, status.display_name,
                    QStringLiteral("健康状态恢复为 %1")
                        .arg(QString::fromUtf8(health_state_name(status.state))),
                    status.detail, true);
              }
              health_states_.insert(status.id, status.state);
            }
            have_health_snapshot_ = true;
          });

  // 各通道的新鲜度统一记入 TelemetryFreshness；SafetyPanel::onMcuStatus /
  // onActuation 已经 markFresh(Mcu/Actuation)，这里不再重复。
  connect(bridge, &RosBridge::egoChanged, this, &MainWindow::onEgo);
  connect(bridge, &RosBridge::egoChanged, log_drawer_, &LogDrawer::onEgoTelemetry);
  connect(bridge, &RosBridge::leadObjectChanged, this, &MainWindow::onLeadObject);
  // Lead 信号到达也视为"前车信息"通道新鲜一次，便于 HUD 状态栏新鲜度指示
  // 与 onLeadObject 内部独立 last_lead_ms_ 解耦。
  connect(bridge, &RosBridge::leadObjectChanged, this,
          [](const GuiLeadObject&) {
            TelemetryFreshness::instance().markFresh(TelemetryFreshness::Lead);
          });

  connect(bridge, &RosBridge::laneGraphChanged, map_view_, &MapView::setLanes);
  connect(bridge, &RosBridge::routeChanged, map_view_, &MapView::setRoute);
  connect(bridge, &RosBridge::navStatusChanged, this,
          [this](const GuiNavStatus& status) {
            TelemetryFreshness::instance().markFresh(TelemetryFreshness::Nav);
            QString text = QString("导航: %1").arg(QString::fromLatin1(nav_state_name(status.state)));
            if (status.state == 3U) {
              text += QString("  剩余 %1 m").arg(status.remaining_distance_m, 0, 'f', 0);
            }
            if (!status.detail.isEmpty()) text += QString("  (%1)").arg(status.detail);
            nav_status_value_->setText(text);
            if (status.state >= 4U) map_view_->setGoal(0.0, 0.0, false);
          });

  connect(map_view_, &MapView::goalRequested, this, &MainWindow::onGoalRequested);
  connect(map_view_, &MapView::goalRejected, this, [this](double distance) {
    nav_status_value_->setText(
        QStringLiteral("导航: 目标离车道 %1 m（>%2 m），请点击车道附近")
            .arg(distance, 0, 'f', 1)
            .arg(MapView::kSnapMaxDistanceM, 0, 'f', 0));
  });
  connect(map_view_, &MapView::followBroken, this,
          [this]() { follow_check_->setChecked(false); });
  connect(follow_check_, &QCheckBox::toggled, map_view_, &MapView::setFollowEnabled);
  connect(fit_button_, &QPushButton::clicked, this, [this]() {
    follow_check_->setChecked(false);
    map_view_->fitToLanes();
  });
  connect(clear_trail_button_, &QPushButton::clicked, map_view_, &MapView::clearTrail);
  connect(layer_button_, &QPushButton::clicked, this, [this]() {
    // 循环切换调试图层：默认关 → grid+trail → grid+trail+halo+snap → 仅 grid
    static int stage = 0;
    stage = (stage + 1) % 4;
    quint32 flags = 0;
    switch (stage) {
      case 0: flags = MapView::kLayerGrid; break;
      case 1: flags = MapView::kLayerGrid | MapView::kLayerTrail; break;
      case 2: flags = MapView::kLayerGrid | MapView::kLayerTrail |
                       MapView::kLayerEgoHalo | MapView::kLayerSnapRadius; break;
      case 3: flags = 0; break;
    }
    map_view_->setLayers(flags);
  });
  connect(cancel_button_, &QPushButton::clicked, this, [this]() {
    bridge_->publishCancel();
    map_view_->setGoal(0.0, 0.0, false);
    nav_status_value_->setText(QStringLiteral("已请求取消导航"));
  });

  // 进程日志全部转接到底部抽屉
  connect(launch_panel_->processManager(), &ProcessManager::logLine,
          log_drawer_, &LogDrawer::appendLog);
  // 全流程进度信号：把 stage+detail 显示到状态栏
  connect(launch_panel_->processManager(), &ProcessManager::stackProgress,
          this, [this](const QString& stage, const QString& detail) {
            stack_progress_label_->setText(detail);
            stack_progress_label_->show();
            QString bg = theme::kMdSurfaceContainerLow;
            if (stage == QStringLiteral("complete")) {
              bg = QStringLiteral("#1f3b1f");  // 绿
            } else if (stage == QStringLiteral("failed")) {
              bg = QStringLiteral("#3b1f1f");  // 红
            } else if (stage == QStringLiteral("stopped")) {
              bg = theme::kMdSurfaceContainerLow;
            }
            stack_progress_label_->setStyleSheet(
                QStringLiteral("color:%1;font-size:11px;padding:3px 8px;"
                               "border-radius:5px;background:%2;border:1px solid %3;")
                    .arg(theme::kMdOnPrimaryContainer, bg, theme::kCardBorder));
            // complete 后 30s 隐藏（避免长期占据状态栏）；失败不隐藏（用户要看到）
            if (stage == QStringLiteral("complete")) {
              QTimer::singleShot(30000, this, [this]() {
                stack_progress_label_->hide();
              });
            }
          });
  connect(launch_panel_->processManager(), &ProcessManager::carlaChanged, this,
          [this](ProcState state, bool, const QString& detail) {
            if (state == ProcState::Failed) {
              log_drawer_->appendFaultEvent(QDateTime::currentMSecsSinceEpoch(), 2,
                                            QStringLiteral("CARLA"),
                                            QStringLiteral("进程异常退出"), detail);
            }
          });
  connect(launch_panel_->processManager(), &ProcessManager::bridgeChanged, this,
          [this](ProcState state, const QString& detail) {
            if (state == ProcState::Failed) {
              log_drawer_->appendFaultEvent(QDateTime::currentMSecsSinceEpoch(), 2,
                                            QStringLiteral("ROS2 Bridge"),
                                            QStringLiteral("进程异常退出"), detail);
            }
          });

  connect(launch_panel_, &LaunchPanel::runStateChanged, this,
          [this](bool running, const QString& scenario, const QString& town) {
            scenario_label_->setText(
                running ? QStringLiteral("场景: %1 @ %2").arg(scenario, town)
                         : QStringLiteral("场景: --"));
            config_label_->setText(
                running ? QStringLiteral("配置: 控制=%1").arg(launch_panel_->currentConfig().control_source)
                         : QStringLiteral("配置: --"));
            if (running) map_view_->clearTrail();
          });

  connect(launch_panel_, &LaunchPanel::demoPresetLaunched, this,
          &MainWindow::onDemoPresetLaunched);

  connect(&stale_timer_, &QTimer::timeout, this, &MainWindow::onStaleCheck);
  stale_timer_.start(200);
  alert_flash_timer_.setInterval(500);
  connect(&alert_flash_timer_, &QTimer::timeout, this, [this]() {
    alert_flash_on_ = !alert_flash_on_;
    update_alert_bar();
  });

  // 恢复窗口几何与分栏比例
  QSettings settings(QStringLiteral("adas"), QStringLiteral("adas_gui"));
  if (!restoreGeometry(
          settings.value(QStringLiteral("window/geometry_v2")).toByteArray())) {
    resize(1400, 860);
  }
  setMinimumSize(1180, 720);
  columns_->restoreState(
      settings.value(QStringLiteral("window/columns_v2")).toByteArray());
  rows_->restoreState(
      settings.value(QStringLiteral("window/rows_v2")).toByteArray());
}

void MainWindow::closeEvent(QCloseEvent* event) {
  if (launch_panel_ && launch_panel_->processManager() &&
      launch_panel_->processManager()->hasManagedProcesses()) {
    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    box.setWindowTitle(QStringLiteral("退出 ADAS 验证台"));
    box.setText(QStringLiteral("<b>完整 HIL 会话仍在运行</b><br>"
                               "关闭 GUI 将停止 bridge、CARLA 与 Orin HIL/CAN 服务。"));
    box.setInformativeText(QStringLiteral("确认停止完整系统并退出 GUI？"));
    box.setStandardButtons(QMessageBox::Cancel | QMessageBox::Yes);
    box.setDefaultButton(QMessageBox::Cancel);
    box.button(QMessageBox::Yes)->setText(QStringLiteral("确认退出"));
    box.button(QMessageBox::Cancel)->setText(QStringLiteral("取消"));
    if (box.exec() != QMessageBox::Yes) {
      event->ignore();
      return;
    }
  }
  QSettings settings(QStringLiteral("adas"), QStringLiteral("adas_gui"));
  settings.setValue(QStringLiteral("window/geometry_v2"), saveGeometry());
  settings.setValue(QStringLiteral("window/columns_v2"), columns_->saveState());
  settings.setValue(QStringLiteral("window/rows_v2"), rows_->saveState());
  QMainWindow::closeEvent(event);
}

QWidget* MainWindow::build_map_panel() {
  auto* panel = new QWidget();
  panel->setObjectName(QStringLiteral("root"));
  auto* root = new QVBoxLayout(panel);
  root->setContentsMargins(8, 10, 8, 8);
  root->setSpacing(7);

  root->addWidget(alert_bar_);

  auto* hud = new QHBoxLayout();
  hud->setSpacing(6);
  hud->addWidget(make_hud_metric(QStringLiteral("gauge-high"), QStringLiteral("当前车速"),
                                 &hud_speed_value_, QStringLiteral("-- km/h")), 2);
  hud->addWidget(make_hud_metric(QStringLiteral("flag"), QStringLiteral("目标车速"),
                                 &hud_target_speed_value_, QStringLiteral("-- km/h")), 1);
  hud->addWidget(make_hud_metric(QStringLiteral("route"), QStringLiteral("驾驶模式"),
                                 &hud_mode_value_, QStringLiteral("等待数据")), 2);
  hud->addWidget(make_hud_metric(QStringLiteral("triangle-exclamation"), QStringLiteral("TTC"),
                                 &hud_ttc_value_, QStringLiteral("-- s")), 1);
  hud->addWidget(make_hud_metric(QStringLiteral("car"), QStringLiteral("前车距离"),
                                 &hud_lead_gap_value_, QStringLiteral("-- m")), 1);
  hud->addWidget(make_hud_metric(QStringLiteral("shield"), QStringLiteral("故障状态"),
                                 &hud_fault_value_, QStringLiteral("正常")), 2);
  root->addLayout(hud);
  update_hud_mode();
  update_hud_fault();

  // 地图工具栏：导航摘要 + 紧凑图标操作。
  auto* toolbar = new QHBoxLayout();
  toolbar->setSpacing(5);
  auto* title = new IconLabel(QStringLiteral("map"), QStringLiteral("实时导航"));
  title->setStyleSheet(QStringLiteral(
      "color:%1;font-size:14px;font-weight:700;letter-spacing:0px;")
      .arg(theme::kTextPrimary));
  nav_status_value_ = new QLabel(QStringLiteral("等待导航状态"));
  nav_status_value_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  nav_status_value_->setStyleSheet(
      QStringLiteral("color:%1;background:%2;border:1px solid %3;"
                     "border-radius:5px;font-size:11px;padding:4px 8px;")
          .arg(theme::kTextSecondary, theme::kMdSurfaceContainerLow,
               theme::kCardBorder));
  follow_check_ = new QCheckBox(QStringLiteral("跟车视角"));
  fit_button_ = make_tool(QStringLiteral("expand"), QStringLiteral("适配全图"));
  clear_trail_button_ = make_tool(QStringLiteral("eraser"), QStringLiteral("清除尾迹"));
  layer_button_ = make_tool(QStringLiteral("sliders"), QStringLiteral("切换地图图层"));
  cancel_button_ = make_tool(QStringLiteral("ban"), QStringLiteral("取消导航"));
  toolbar->addWidget(title);
  toolbar->addWidget(nav_status_value_, 1);
  toolbar->addWidget(follow_check_);
  toolbar->addWidget(fit_button_);
  toolbar->addWidget(clear_trail_button_);
  toolbar->addWidget(layer_button_);
  toolbar->addWidget(cancel_button_);
  root->addLayout(toolbar);

  root->addWidget(map_view_, 1);

  return panel;
}

void MainWindow::update_hud_mode() {
  if (!hud_mode_value_) return;
  QString mode = QStringLiteral("等待数据");
  QString color = theme::kTextSecondary;
  bool prominent = false;

  if (last_aeb_state_ >= 3) {
    mode = QStringLiteral("AEB 紧急制动");
    color = theme::kDanger;
    prominent = true;
  } else if (last_aeb_state_ == 2) {
    mode = QStringLiteral("AEB 预警");
    color = theme::kWarn;
    prominent = true;
  } else if (last_safety_level_ >= 2) {
    mode = QStringLiteral("安全链故障");
    color = theme::kDanger;
    prominent = true;
  } else if (last_safety_level_ == 1) {
    mode = QStringLiteral("降级运行");
    color = theme::kWarn;
    prominent = true;
  } else if (last_behavior_state_ >= 0) {
    mode = QString::fromUtf8(
        behavior_state_name(static_cast<std::uint8_t>(last_behavior_state_)));
    color = mode == QStringLiteral("紧急") ? theme::kDanger : theme::kOk;
    prominent = mode == QStringLiteral("紧急");
  }

  set_hud_value(hud_mode_value_, mode, color, prominent);
}

void MainWindow::update_hud_fault() {
  if (!hud_fault_value_) return;
  if (!active_alerts_.isEmpty()) {
    const QString text =
        active_alerts_.size() == 1
            ? active_alerts_.value(alert_order_.isEmpty() ? active_alerts_.firstKey()
                                                          : alert_order_.first())
            : QStringLiteral("%1 项告警").arg(active_alerts_.size());
    set_hud_value(hud_fault_value_, text, theme::kDanger, true);
    return;
  }
  if (last_safety_level_ >= 2) {
    set_hud_value(hud_fault_value_,
                  QString::fromUtf8(safety_level_name(
                      static_cast<std::uint8_t>(last_safety_level_))),
                  theme::kDanger, true);
  } else if (last_safety_level_ == 1) {
    set_hud_value(hud_fault_value_, QStringLiteral("WARN"), theme::kWarn, true);
  } else {
    set_hud_value(hud_fault_value_, QStringLiteral("正常"), theme::kOk);
  }
}

void MainWindow::set_alert(const QString& key, const QString& text, bool active) {
  const bool changed = active ? (active_alerts_.value(key) != text)
                                : active_alerts_.contains(key);
  if (!changed) return;
  if (active) {
    if (!active_alerts_.contains(key)) alert_order_.append(key);
    active_alerts_.insert(key, text);
  } else {
    active_alerts_.remove(key);
    alert_order_.removeAll(key);
  }
  if (active_alerts_.isEmpty()) {
    alert_flash_timer_.stop();
    alert_flash_on_ = false;
  } else if (!alert_flash_timer_.isActive()) {
    alert_flash_on_ = true;
    alert_flash_timer_.start();
  }
  update_alert_bar();
  update_hud_fault();
}

void MainWindow::update_alert_bar() {
  if (active_alerts_.isEmpty()) {
    alert_bar_->hide();
    return;
  }
  QStringList lines;
  for (const auto& key : alert_order_) lines << active_alerts_.value(key);
  alert_bar_->setText(lines.join(QStringLiteral("   ·   ")));
  alert_bar_->setStyleSheet(
       QStringLiteral("background:%1;color:%2;font-weight:600;"
                      "border:1px solid %3;border-radius:6px;padding:6px 14px;"
                     "font-size:13px;")
          .arg(alert_flash_on_ ? theme::kMdErrorContainer : QStringLiteral("#5a1010"),
               QStringLiteral("#ffdad6"),
               QStringLiteral("#7a1f1f")));
  alert_bar_->show();
}

void MainWindow::onLeadObject(const GuiLeadObject& lead) {
  if (!lead.valid || !std::isfinite(lead.gap_m)) {
    set_hud_value(hud_lead_gap_value_, QStringLiteral("-- m"), theme::kTextSecondary);
    return;
  }
  const QString color = lead.gap_m < 10.0f ? theme::kDanger
                      : lead.gap_m < 25.0f ? theme::kWarn
                                           : theme::kOk;
  QString text = QStringLiteral("%1 m").arg(lead.gap_m, 0, 'f', 1);
  if (std::isfinite(lead.speed_mps)) {
    text += QStringLiteral(" / %1").arg(lead.speed_mps * 3.6f, 0, 'f', 0);
  }
  set_hud_value(hud_lead_gap_value_, text, color, lead.gap_m < 25.0f);
}

void MainWindow::onEgo(double x, double y, double yaw_rad, double speed_mps,
                       double yaw_rate_rps) {
  TelemetryFreshness::instance().markFresh(TelemetryFreshness::Ego);
  // NaN/Inf 守卫：bridge 在异常情况下可能发布 NaN/Inf odom，绝不让相机和 HUD 被污染
  // （一旦 setVehicle 拿到 NaN，map 拖拽/缩放立刻坏掉）。
  const bool pose_ok = std::isfinite(x) && std::isfinite(y) &&
                       std::isfinite(yaw_rad);
  if (pose_ok) {
    map_view_->setVehicle(x, y, yaw_rad, true);
    safety_panel_->onEgo(x, y, yaw_rad, speed_mps, yaw_rate_rps);
  } else {
    map_view_->setVehicle(0.0, 0.0, 0.0, false);
  }
  if (std::isfinite(speed_mps)) {
    set_hud_value(hud_speed_value_,
                  QStringLiteral("%1 km/h")
                      .arg(std::max(0.0, speed_mps) * 3.6, 0, 'f', 1),
                  theme::kTextPrimary, true);
  }
}

void MainWindow::onDemoPresetLaunched(const QString& scenario,
                                      const QString& town) {
  // 仅提示：预设是"启动哪个场景"，不挂起导航目标。如要导航，请点击地图选点。
  nav_status_value_->setText(
      QStringLiteral("演示场景: %1 @ %2（导航请手动点击地图）")
          .arg(scenario, town));
}

void MainWindow::onGoalRequested(double world_x, double world_y) {
  bridge_->publishGoal(world_x, world_y);
  map_view_->setGoal(world_x, world_y, true);
  nav_status_value_->setText(QString("导航: 目标已发送 (%1, %2)")
                                  .arg(world_x, 0, 'f', 1)
                                  .arg(world_y, 0, 'f', 1));
}

void MainWindow::onStaleCheck() {
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  const auto& f = TelemetryFreshness::instance();
  set_dot(dot_mcu_, f.isFresh(TelemetryFreshness::Mcu, kMcuStaleMs));
  set_dot(dot_actuation_, f.isFresh(TelemetryFreshness::Actuation, kActuationStaleMs));
  set_dot(dot_ego_, f.isFresh(TelemetryFreshness::Ego, kEgoStaleMs));
  set_dot(dot_nav_, f.isFresh(TelemetryFreshness::Nav, kNavStaleMs));
  if (!f.isFresh(TelemetryFreshness::Ego, kEgoStaleMs)) {
    set_hud_value(hud_speed_value_, QStringLiteral("-- km/h"), theme::kTextSecondary);
  }
  if (!f.isFresh(TelemetryFreshness::Behavior, 1200)) {
    last_behavior_state_ = -1;
    set_hud_value(hud_target_speed_value_, QStringLiteral("-- km/h"),
                  theme::kTextSecondary);
    update_hud_mode();
  }
  if (!f.isFresh(TelemetryFreshness::Aeb, 1200)) {
    last_aeb_state_ = 0;
    set_hud_value(hud_ttc_value_, QStringLiteral("-- s"), theme::kTextSecondary);
    update_hud_mode();
  }
  if (!f.isFresh(TelemetryFreshness::Lead, 1200)) {
    set_hud_value(hud_lead_gap_value_, QStringLiteral("-- m"), theme::kTextSecondary);
  }
  safety_panel_->onStaleCheck(now);
}

}  // namespace adas::gui
