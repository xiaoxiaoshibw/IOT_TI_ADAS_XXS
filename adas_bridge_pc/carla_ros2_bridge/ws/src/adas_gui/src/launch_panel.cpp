#include "launch_panel.hpp"

#include "run_id_session.hpp"

#include <QDateTime>
#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QScrollArea>
#include <QSettings>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QVBoxLayout>
#include <QWheelEvent>

#include "icons.hpp"
#include "preflight_check.hpp"
#include "scenario_workflow.hpp"
#include "ros_bridge.hpp"
#include "secure_settings.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace adas::gui {
namespace {

LedState to_led_state(ProcState state, bool ready) {
  switch (state) {
    case ProcState::Running: return ready ? LedState::Ok : LedState::Warn;
    case ProcState::Starting:
    case ProcState::Stopping: return LedState::Warn;
    case ProcState::Failed: return LedState::Danger;
    case ProcState::Stopped: return LedState::Stale;
  }
  return LedState::Stale;
}

LedState to_led_state(HealthState state) {
  switch (state) {
    case HealthState::Healthy: return LedState::Ok;
    case HealthState::Starting:
    case HealthState::Degraded: return LedState::Warn;
    case HealthState::Fault: return LedState::Danger;
    case HealthState::Unknown:
    case HealthState::Offline: return LedState::Stale;
  }
  return LedState::Stale;
}

void set_led_state(QLabel* holder, LedState state) {
  if (auto* led = qobject_cast<LedIndicator*>(holder)) led->setState(state);
}

QString default_carla_root() {
  return discover_carla_root();
}

QString environment_or(const char* name, const QString& fallback) {
  const QByteArray value = qgetenv(name);
  return value.isEmpty() ? fallback : QString::fromLocal8Bit(value);
}

QGroupBox* make_card(const QString& icon_name, const QString& title, QWidget* parent = nullptr) {
  auto* box = new QGroupBox(parent);
  box->setTitle(QString());
  box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
  auto* root = new QVBoxLayout(box);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(8);
  auto* header = new QHBoxLayout();
  header->setSpacing(8);
  auto* icon = new IconLabel(icon_name, title);
  icon->setStyleSheet(QStringLiteral("color:%1;font-size:12px;font-weight:600;"
                                     "letter-spacing:0px;")
                          .arg(theme::kMdPrimary));
  header->addWidget(icon);
  header->addStretch(1);
  root->addLayout(header);
  return box;
}

BusyButton* make_button(const QString& icon_name, const QString& text,
                        const QString& object_name, QWidget* parent = nullptr) {
  auto* b = new BusyButton(text, parent);
  b->setIcon(icons::get(icon_name));
  b->setIconSize(QSize(16, 16));
  b->setObjectName(object_name);
  return b;
}

}  // namespace

LaunchPanel::LaunchPanel(QWidget* parent) : QWidget(parent) {
  build_ui();
  restore_settings();

  connect(&manager_, &ProcessManager::carlaChanged, this,
          [this](ProcState state, bool ready, const QString& detail) {
            set_led_state(carla_light_, to_led_state(state, ready));
            QColor text_color(state == ProcState::Running && ready ? theme::kOk
                          : state == ProcState::Running ? theme::kWarn
                          : state == ProcState::Failed  ? theme::kDanger
                          : theme::kStale);
            QString text = ready && state != ProcState::Running
                               ? QStringLiteral("外部实例")
                               : proc_state_name(state);
            if (ready) text += QStringLiteral(" · 端口就绪");
            if (!detail.isEmpty()) text += QStringLiteral("  (%1)").arg(detail);
            carla_state_label_->setText(text);
            carla_state_label_->setStyleSheet(
                QStringLiteral("color:%1;font-size:12px;font-weight:600;")
                    .arg(text_color.name()));
            const HealthState health = ready ? HealthState::Healthy
                : state == ProcState::Starting ? HealthState::Starting
                : state == ProcState::Failed ? HealthState::Fault
                                             : HealthState::Offline;
            setHealthRow(QStringLiteral("carla"), health, detail);
            if (state == ProcState::Running || state == ProcState::Failed) {
              start_carla_button_->setBusy(false);
              if (state == ProcState::Failed && active_start_button_) {
                active_start_button_->setBusy(false);
                active_start_button_ = nullptr;
              }
            }
            if (!manager_.hasManagedProcesses()) stop_all_button_->setBusy(false);
            update_buttons();
          });
  connect(&manager_, &ProcessManager::bridgeChanged, this,
          [this](ProcState state, const QString& detail) {
            set_led_state(bridge_light_, to_led_state(state, true));
            QColor text_color(state == ProcState::Running ? theme::kOk
                          : state == ProcState::Failed   ? theme::kDanger
                          : state == ProcState::Starting || state == ProcState::Stopping
                                                         ? theme::kWarn
                                                         : theme::kStale);
            QString text = proc_state_name(state);
            if (!detail.isEmpty()) text += QStringLiteral("  (%1)").arg(detail);
            bridge_state_label_->setText(text);
            bridge_state_label_->setStyleSheet(
                QStringLiteral("color:%1;font-size:12px;font-weight:600;")
                    .arg(text_color.name()));
            if (state != ProcState::Running) {
              const HealthState health = state == ProcState::Starting ? HealthState::Starting
                  : state == ProcState::Failed ? HealthState::Fault
                                               : HealthState::Offline;
              setHealthRow(QStringLiteral("bridge"), health, detail);
            }
            const bool running = state == ProcState::Running;
            if (running) runtime_timer_.start();
            else {
              runtime_timer_.stop();
              if (state == ProcState::Stopped || state == ProcState::Failed)
                runtime_label_->setText(QStringLiteral("--"));
            }
            emit runStateChanged(running, selectedScenario(),
                                 town_combo_->currentText());
            if (state == ProcState::Running || state == ProcState::Failed) {
              start_bridge_button_->setBusy(false);
              if (active_start_button_) {
                active_start_button_->setBusy(false);
                active_start_button_ = nullptr;
              }
            }
            if (state == ProcState::Stopped || state == ProcState::Failed ||
                (state == ProcState::Running &&
                 detail.contains(QStringLiteral("未停止")))) {
              stop_bridge_button_->setBusy(false);
            }
            if (!manager_.hasManagedProcesses()) stop_all_button_->setBusy(false);
            update_buttons();
          });

  runtime_timer_.setInterval(1000);
  connect(&runtime_timer_, &QTimer::timeout, this, [this]() {
    const qint64 started = manager_.bridgeStartedAtMs();
    if (started < 0) return;
    const qint64 seconds =
        (QDateTime::currentMSecsSinceEpoch() - started) / 1000;
    runtime_label_->setText(QStringLiteral("%1:%2:%3")
                                .arg(seconds / 3600, 2, 10, QChar('0'))
                                .arg((seconds / 60) % 60, 2, 10, QChar('0'))
                                .arg(seconds % 60, 2, 10, QChar('0')));
  });

  connect(start_all_button_, &QPushButton::clicked, this, [this]() {
    startConfiguredSystem(/*interactive=*/true);
  });
  connect(stop_all_button_, &QPushButton::clicked, this, [this]() {
    if (!confirm_action(this, QStringLiteral("确认停止完整系统"),
                        QStringLiteral("停止本机受管的 bridge 与 CARLA/SIL"),
                        QStringLiteral("Orin HIL/CAN 常驻服务不会被停止；外部实例也不会被终止。"),
                        ConfirmSeverity::Danger)) return;
    stop_all_button_->setBusy(true, QStringLiteral("停止中"),
                              QStringLiteral("等待受管进程退出"));
    manager_.stopAll();
    update_buttons();
  });
  connect(start_carla_button_, &QPushButton::clicked, this, [this]() {
    if (!confirm_action(this, QStringLiteral("确认启动仿真"),
                        QStringLiteral("仅启动 CARLA/SIL 仿真"),
                        QStringLiteral("不会自动启动 bridge。"))) return;
    start_carla_button_->setBusy(true, QStringLiteral("启动中"));
    save_settings();
    manager_.startCarla(currentConfig());
    update_buttons();
  });
  connect(start_bridge_button_, &QPushButton::clicked, this, [this]() {
    if (!confirm_action(this, QStringLiteral("确认启动桥接"),
                        QStringLiteral("仅启动 ROS2 bridge"),
                        QStringLiteral("请确认仿真与控制源已经就绪。"))) return;
    start_bridge_button_->setBusy(true, QStringLiteral("启动中"));
    save_settings();
    manager_.startBridge(currentConfig());
    update_buttons();
  });
  connect(stop_bridge_button_, &QPushButton::clicked, this, [this]() {
    if (!confirm_action(this, QStringLiteral("确认停止桥接"),
                        QStringLiteral("停止本 GUI 管理的 ROS2 bridge"),
                        QStringLiteral("控制数据将中断；外部 bridge 不会被终止。"),
                        ConfirmSeverity::Danger)) return;
    stop_bridge_button_->setBusy(true, QStringLiteral("停止中"));
    manager_.stopBridge();
    update_buttons();
  });
  connect(flash_mcu_button_, &QPushButton::clicked, this,
          &LaunchPanel::onFlashMcuClicked);
  // 首次启动缺 Orin ssh 密码：弹 QInputDialog 让用户填；存到 secrets.ini 后
  // 用户再次按"一键启动全流程"即可（不再弹）。
  connect(&manager_, &ProcessManager::needsOrinCredentials, this,
          &LaunchPanel::onNeedsOrinCredentials);
  connect(&manager_, &ProcessManager::needsOrinCredentials, this,
          [this](const QString&, const QString&) {
            if (active_start_button_) active_start_button_->setBusy(false);
            active_start_button_ = nullptr;
          });
  connect(&manager_, &ProcessManager::stackProgress, this,
          [this](const QString& stage, const QString&) {
            if (stage == QStringLiteral("failed") ||
                stage == QStringLiteral("complete")) {
              if (active_start_button_) active_start_button_->setBusy(false);
              active_start_button_ = nullptr;
            }
          });
  connect(&manager_, &ProcessManager::flashFinished, this,
          [this](bool success, const QString& detail) {
            flash_mcu_button_->setBusy(false);
            flash_mcu_button_->setToolTip(detail);
            if (!success) {
              QMessageBox::warning(this, QStringLiteral("MCU 烧录失败"), detail);
            }
            update_buttons();
          });
  connect(advanced_toggle_, &QPushButton::toggled, this,
          &LaunchPanel::setAdvancedVisible);
  update_buttons();
}

LaunchPanel::~LaunchPanel() {
  save_settings();
  manager_.stopAll();
}

bool LaunchPanel::startConfiguredSystem(bool interactive) {
  auto config = currentConfig();
  if (config.scenario.isEmpty() || config.scenario_file.isEmpty() ||
      config.acceptance_profile.isEmpty() || config.nav_distance_m <= 0.0) {
    const QString detail = QStringLiteral(
        "场景 catalog 缺失或无效；已 fail-closed，禁止启动。请运行 tools/validate_scenario.py");
    emit manager_.logLine(QStringLiteral("PREFLIGHT"), detail);
    if (interactive) QMessageBox::critical(this, QStringLiteral("场景 catalog 无效"), detail);
    return false;
  }
  if (interactive) {
    if (!runPreflightGate()) return false;
    if (!confirm_action(this, QStringLiteral("确认启动完整系统"),
                        QStringLiteral("%1 · %2 @ %3")
                            .arg(backend_display_name(config.backend), config.scenario,
                                 config.town),
                        QStringLiteral("场景文件：%1\nseed=%2，预计目标=%3\n"
                                       "%4\n运行中配置将锁定，停止后可切换场景。")
                            .arg(config.scenario_file)
                            .arg(config.seed)
                            .arg(config.expected_actor_count)
                            .arg(config.auto_navigation
                                     ? QStringLiteral("将按场景 profile 自动生成并下发车道导航目标")
                                     : QStringLiteral("不会自动导航，请在 Town 地图手动选点")),
                        ConfirmSeverity::Warning)) return false;
  } else {
    const auto items = run_preflight(config);
    for (const auto& item : items) {
      if (item.level == PreflightLevel::Fail) {
        emit manager_.logLine(QStringLiteral("PREFLIGHT"),
                              QStringLiteral("[FAIL] %1：%2").arg(item.label, item.detail));
        return false;
      }
      if (item.level == PreflightLevel::Warn) {
        emit manager_.logLine(QStringLiteral("PREFLIGHT"),
                              QStringLiteral("[WARN] %1：%2；--autostart 继续")
                                  .arg(item.label, item.detail));
      }
    }
  }
  // 预检与确认完成才建立会话；取消或预检失败不会轮换 run_id。
  config.run_id = adas_gui::RunIdSession::begin();
  start_all_button_->setBusy(true, QStringLiteral("启动中"),
                             QStringLiteral("等待 bridge 运行或启动失败"));
  active_start_button_ = start_all_button_;
  save_settings();
  const auto profile = scenario_workflow_profile(config.acceptance_profile);
  emit scenarioRunRequested(config.scenario, config.town,
                            config.acceptance_profile,
                            config.auto_navigation && profile.requires_navigation,
                            config.nav_distance_m);
  manager_.startAll(config);
  update_buttons();
  return true;
}

QString LaunchPanel::selectedBackend() const {
  return backend_combo_->currentData().toString();
}

QString LaunchPanel::selectedScenario() const {
  return scenario_combo_->currentData().toString();
}

void LaunchPanel::rebuildScenarioChoices(const QString& preferred) {
  const QString wanted = preferred.isEmpty() ? selectedScenario() : preferred;
  const QSignalBlocker blocker(scenario_combo_);
  scenario_combo_->clear();
  const auto catalog = load_scenario_catalog();
  if (catalog.isEmpty()) {
    scenario_combo_->addItem(QStringLiteral("场景 catalog 无效，禁止启动"), QString());
    scenario_combo_->setEnabled(false);
  } else {
    scenario_combo_->setEnabled(true);
    for (const auto& entry : catalog) {
      if (!scenario_supports_backend(entry, selectedBackend())) continue;
      scenario_combo_->addItem(
          QStringLiteral("%1  ·  %2").arg(entry.display_name, entry.id), entry.id);
    }
  }
  int index = scenario_combo_->findData(wanted);
  if (index < 0) index = scenario_combo_->findData(QStringLiteral("free"));
  if (index < 0 && scenario_combo_->count() > 0) index = 0;
  scenario_combo_->setCurrentIndex(index);
  updateScenarioSummary();
}

void LaunchPanel::updateScenarioSummary() {
  const auto entry = scenario_catalog_entry(selectedScenario());
  if (entry.id.isEmpty() || entry.file.isEmpty()) {
    scenario_summary_->setText(QStringLiteral("兼容场景 · 无 catalog 元数据"));
    return;
  }
  scenario_summary_->setText(
      QStringLiteral("%1\n预计目标 %2 · seed=%3 · %4")
          .arg(entry.description)
          .arg(entry.expected_actor_count)
          .arg(entry.default_seed)
          .arg(QFileInfo(entry.file).fileName()));
  const int town = town_combo_->findText(entry.default_town);
  if (town >= 0) town_combo_->setCurrentIndex(town);
}

LaunchConfig LaunchPanel::currentConfig() const {
  LaunchConfig config;
  config.backend = selectedBackend();
  config.repo_root = find_repo_root();
  config.carla_root = carla_root_edit_->text().trimmed();
  config.scenario = selectedScenario();
  const auto entry = scenario_catalog_entry(config.scenario, config.repo_root);
  config.scenario_file = entry.file;
  config.seed = entry.default_seed;
  config.expected_actor_count = entry.expected_actor_count;
  config.acceptance_profile = entry.acceptance_profile;
  config.nav_distance_m = entry.nav_distance_m;
  config.town = entry.default_town.isEmpty() ? town_combo_->currentText()
                                              : entry.default_town;
  config.control_source = source_combo_->currentText();
  config.low_quality = low_quality_check_->isChecked();
  // The product contract requires CARLA to be the authoritative visible
  // scene.  Headless rendering is intentionally unavailable from this GUI.
  config.render_offscreen = false;
  config.auto_navigation = auto_navigation_check_->isChecked();
  // Orin 远程启动默认不勾（比赛前一天手动 systemctl start 更稳）。
  // 勾上时 GUI 会 SSH 上 Orin 跑 CAN 链路 + adas-hil 编排，需要 SecureSettings
  // 里有密码。
  config.start_full_stack = start_full_stack_check_ && start_full_stack_check_->isChecked();
  // 只返回已确认会话；startConfiguredSystem 在所有 gate 通过后覆盖为新 UUID。
  config.run_id = adas_gui::RunIdSession::current();
  return config;
}

void LaunchPanel::build_ui() {
  setObjectName(QStringLiteral("root"));
  setMinimumWidth(292);
  setMaximumWidth(360);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(10, 10, 8, 8);
  outer->setSpacing(8);

  auto* heading = new QHBoxLayout();
  auto* product = new IconLabel(QStringLiteral("microchip"), QStringLiteral("ADAS 验证台"));
  product->setStyleSheet(QStringLiteral(
      "color:%1;font-size:15px;font-weight:700;letter-spacing:0px;")
      .arg(theme::kTextPrimary));
  auto* mode = new QLabel(manager_.milMode() ? QStringLiteral("MIL")
                              : manager_.carlaLocalMode() ? QStringLiteral("CARLA")
                                                          : QStringLiteral("HIL"));
  mode->setStyleSheet(QStringLiteral(
      "color:%1;background:%2;border:1px solid %3;border-radius:5px;"
      "padding:3px 7px;font-size:10px;font-weight:700;")
      .arg(theme::kMdPrimary, theme::kMdPrimaryContainer, theme::kCardBorder));
  heading->addWidget(product);
  heading->addStretch(1);
  heading->addWidget(mode);
  outer->addLayout(heading);

  auto* scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* content = new QWidget();
  content->setObjectName(QStringLiteral("sidebarContent"));
  auto* root = new QVBoxLayout(content);
  root->setContentsMargins(0, 0, 3, 0);
  root->setSpacing(8);
  root->setSizeConstraint(QLayout::SetMinAndMaxSize);
  scroll->setWidget(content);

  // ===== 卡片 0: 演示预设（一键起终点+场景） =====
  build_preset_card(root);

  // ===== 卡片 1: 运行配置（折叠到 form） =====
  auto* config_box = make_card(QStringLiteral("sliders"), QStringLiteral("运行配置"));
  auto* config_body = new QWidget();
  auto* form = new QFormLayout(config_body);
  form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
  form->setHorizontalSpacing(8);
  form->setVerticalSpacing(6);
  form->setContentsMargins(0, 0, 0, 0);
  backend_combo_ = new QComboBox();
  for (const QString& backend : known_backends()) {
    backend_combo_->addItem(backend_display_name(backend), backend);
  }
  backend_combo_->setToolTip(QStringLiteral(
      "MIL=本机 sim_vehicle + SoC + vcan + F280025C host-runner 模拟硬件闭环；"
      "CARLA 本机联合=CARLA 窗口展示 + 本机 SoC 控制闭环；"
      "未完成的 HIL 适配不作为可选运行配置。后端决定 ROS_DOMAIN_ID，"
      "如需切换必须用 --backend 重启 GUI，"
      "运行中不能在下拉框热切换。"));
  scenario_combo_ = new QComboBox();
  scenario_summary_ = new QLabel();
  scenario_summary_->setWordWrap(true);
  scenario_summary_->setStyleSheet(QStringLiteral("color:%1;font-size:10px;")
                                       .arg(theme::kTextSecondary));
  town_combo_ = new QComboBox();
  town_combo_->addItems(known_towns());
  town_combo_->setEnabled(false);
  town_combo_->installEventFilter(this);
  source_combo_ = new QComboBox();
  source_combo_->addItems(known_control_sources());
  source_combo_->setEnabled(false);
  source_combo_->installEventFilter(this);
  carla_root_edit_ = new QLineEdit(default_carla_root());
  carla_browse_button_ = new QPushButton();
  carla_browse_button_->setIcon(icons::get(QStringLiteral("folder-open")));
  carla_browse_button_->setIconSize(QSize(14, 14));
  carla_browse_button_->setFixedWidth(32);
  carla_browse_button_->setObjectName(QStringLiteral("textButton"));
  connect(carla_browse_button_, &QPushButton::clicked, this, [this]() {
    const QString dir = QFileDialog::getExistingDirectory(
        this, QStringLiteral("选择 CARLA 目录"), carla_root_edit_->text());
    if (!dir.isEmpty()) carla_root_edit_->setText(dir);
  });
  auto* root_row = new QHBoxLayout();
  root_row->setSpacing(4);
  root_row->addWidget(carla_root_edit_, 1);
  root_row->addWidget(carla_browse_button_);
  low_quality_check_ = new QCheckBox(QStringLiteral("低画质"));
  low_quality_check_->setToolTip(QStringLiteral("降低 CARLA 画质，适合较弱 GPU"));
  offscreen_check_ = new QCheckBox(QStringLiteral("离屏渲染"));
  offscreen_check_->setChecked(false);
  offscreen_check_->setEnabled(false);
  offscreen_check_->setVisible(false);
  offscreen_check_->setToolTip(QStringLiteral("GUI 模式强制显示 CARLA 窗口"));
  auto_navigation_check_ = new QCheckBox(QStringLiteral("自动导航"));
  auto_navigation_check_->setToolTip(QStringLiteral(
      "场景启动且 Town 地图/自车位姿就绪后，沿有向车道图自动生成安全目标；"
      "关闭后可只用中间地图手动选点。"));
  start_full_stack_check_ = new QCheckBox(QStringLiteral("包含 Orin 远程启动"));
  start_full_stack_check_->setToolTip(QStringLiteral(
      "勾上后 GUI 会 SSH 上 Orin（%1@%2）跑 CAN 链路 + adas-hil.service。"
      "比赛前不推荐，比赛当天调试 OK 后再勾。\n"
      "不勾：GUI 只跑 PC 本地（CARLA + bridge），Orin 端由操作员提前手动起。")
          .arg(QStringLiteral("jetson"), QStringLiteral("192.168.100.32")));
  auto* option_row = new QHBoxLayout();
  option_row->setSpacing(8);
  option_row->addWidget(low_quality_check_);
  option_row->addWidget(offscreen_check_);
  option_row->addWidget(auto_navigation_check_);
  option_row->addStretch(1);
  form->addRow(QStringLiteral("后端"), backend_combo_);
  form->addRow(QStringLiteral("场景"), scenario_combo_);
  form->addRow(QString(), scenario_summary_);
  form->addRow(QStringLiteral("地图"), town_combo_);
  form->addRow(QStringLiteral("控制源"), source_combo_);
  form->addRow(QStringLiteral("CARLA 目录"), root_row);
  form->addRow(QStringLiteral("选项"), option_row);
  form->addRow(QStringLiteral("远程"), start_full_stack_check_);
  config_box->layout()->addWidget(config_body);
  root->addWidget(config_box);

  connect(backend_combo_, &QComboBox::currentIndexChanged, this, [this]() {
    rebuildScenarioChoices(selectedScenario());
    update_buttons();
  });
  connect(scenario_combo_, &QComboBox::currentIndexChanged, this, [this]() {
    updateScenarioSummary();
    emit scenarioSelectionChanged(selectedScenario(), town_combo_->currentText());
  });
  rebuildScenarioChoices(QStringLiteral("free"));

  // ===== 卡片 2: 进程状态 =====
  auto* status_box = make_card(QStringLiteral("signal"), QStringLiteral("进程状态"));
  auto* status_body = new QWidget();
  auto* status_grid = new QFormLayout(status_body);
  status_grid->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_grid->setHorizontalSpacing(8);
  status_grid->setVerticalSpacing(6);
  status_grid->setContentsMargins(0, 0, 0, 0);
  carla_light_ = new LedIndicator();
  carla_light_->setState(LedState::Stale);
  carla_state_label_ = new QLabel(QStringLiteral("未运行"));
  carla_state_label_->setStyleSheet(
      QStringLiteral("color:%1;font-size:12px;font-weight:600;").arg(theme::kStale));
  bridge_light_ = new LedIndicator();
  bridge_light_->setState(LedState::Stale);
  bridge_state_label_ = new QLabel(QStringLiteral("未运行"));
  bridge_state_label_->setStyleSheet(
      QStringLiteral("color:%1;font-size:12px;font-weight:600;").arg(theme::kStale));
  runtime_label_ = new QLabel(QStringLiteral("--"));
  runtime_label_->setStyleSheet(
      QStringLiteral("color:%1;font-size:13px;font-weight:600;padding:3px 10px;"
                     "border-radius:6px;background:%2;border:1px solid %3;")
          .arg(theme::kTextPrimary, theme::kMdSurfaceContainerHigh,
               theme::kCardBorder));
  auto* carla_row = new QHBoxLayout();
  carla_row->setSpacing(6);
  carla_row->addWidget(carla_light_);
  carla_row->addWidget(carla_state_label_, 1);
  auto* bridge_row = new QHBoxLayout();
  bridge_row->setSpacing(6);
  bridge_row->addWidget(bridge_light_);
  bridge_row->addWidget(bridge_state_label_, 1);
  status_grid->addRow(manager_.milMode() ? QStringLiteral("MIL 仿真")
                          : manager_.carlaLocalMode() ? QStringLiteral("CARLA")
                                                      : QStringLiteral("HIL"),
                      carla_row);
  status_grid->addRow(manager_.milMode() ? QStringLiteral("模拟硬件栈")
                          : manager_.carlaLocalMode() ? QStringLiteral("本机 SoC/Bridge")
                                                      : QStringLiteral("HIL 桥接"),
                      bridge_row);
  status_grid->addRow(QStringLiteral("时长"), runtime_label_);
  status_box->layout()->addWidget(status_body);
  root->addWidget(status_box);

  // ===== 卡片 3: ROS graph + 真实话题新鲜度健康状态 =====
  auto* health_box = make_card(QStringLiteral("signal"), QStringLiteral("系统健康"));
  auto* health_body = new QWidget();
  auto* health_grid = new QGridLayout(health_body);
  health_grid->setContentsMargins(0, 0, 0, 0);
  health_grid->setHorizontalSpacing(5);
  health_grid->setVerticalSpacing(2);
  health_grid->setColumnStretch(1, 1);
  const QList<QPair<QString, QString>> health_rows = {
      {QStringLiteral("carla"), manager_.milMode() ? QStringLiteral("MIL 仿真")
                              : manager_.carlaLocalMode() ? QStringLiteral("CARLA 世界")
                                                          : QStringLiteral("HIL")},
      {QStringLiteral("bridge"), manager_.milMode() ? QStringLiteral("MIL 控制栈")
                               : manager_.carlaLocalMode() ? QStringLiteral("CARLA Bridge")
                                                           : QStringLiteral("HIL Bridge")},
      {QStringLiteral("orin_stack"), manager_.milMode() ? QStringLiteral("本机 SoC 栈")
                                   : manager_.carlaLocalMode() ? QStringLiteral("本机 SoC 栈")
                                                               : QStringLiteral("Orin 控制栈")},
      {QStringLiteral("can_gateway"), manager_.milMode() ? QStringLiteral("虚拟 CAN Gateway")
                                                           : QStringLiteral("CAN Gateway")},
      {QStringLiteral("safety_monitor"), QStringLiteral("Safety Monitor")},
      {QStringLiteral("mcu"), manager_.milMode() ? QStringLiteral("模拟 F280025C")
                                                   : QStringLiteral("F280025C MCU")},
      {QStringLiteral("can_link"), manager_.milMode() ? QStringLiteral("vcan 模拟链路")
                                                        : QStringLiteral("CAN链路")},
      {QStringLiteral("odometry"), QStringLiteral("里程计")},
      {QStringLiteral("navigation"), QStringLiteral("导航模块")},
  };
  int health_row = 0;
  for (const auto& entry : health_rows) {
    auto* light = new LedIndicator();
    light->setState(LedState::Stale);
    auto* name = new QLabel(entry.second);
    name->setStyleSheet(QStringLiteral("color:%1;font-size:11px;").arg(theme::kTextSecondary));
    auto* value = new QLabel(QStringLiteral("未知"));
    value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value->setStyleSheet(QStringLiteral("color:%1;font-size:11px;font-weight:600;")
                             .arg(theme::kStale));
    health_grid->addWidget(light, health_row, 0);
    health_grid->addWidget(name, health_row, 1);
    health_grid->addWidget(value, health_row, 2);
    health_lights_.insert(entry.first, light);
    health_values_.insert(entry.first, value);
    ++health_row;
  }
  health_box->layout()->addWidget(health_body);
  root->addWidget(health_box);
  root->addStretch(1);
  outer->addWidget(scroll, 1);

  // ===== 主操作：仅 [启动完整系统] / [停止系统] =====
  auto* primary_row = new QHBoxLayout();
  primary_row->setSpacing(6);
  start_all_button_ = make_button(QStringLiteral("play"), QStringLiteral("启动完整系统"),
                                  QStringLiteral("primaryButton"));
  start_all_button_->setToolTip(manager_.milMode()
      ? QStringLiteral("启动 MIL 本机闭环：sim_vehicle → SoC 控制栈 → vcan → 模拟 F280025C。")
      : QStringLiteral("HIL 当前仅保留接口，尚未适配，不会执行 SSH/CAN/MCU 动作。"));
  stop_all_button_  = make_button(QStringLiteral("stop"), QStringLiteral("停止完整系统"),
                                   QStringLiteral("dangerButton"));
  primary_row->addWidget(start_all_button_, 2);
  primary_row->addWidget(stop_all_button_, 1);
  outer->addLayout(primary_row);

  // ===== 高级操作折叠 =====
  advanced_toggle_ = new QPushButton(QStringLiteral("高级操作"));
  advanced_toggle_->setObjectName(QStringLiteral("textButton"));
  advanced_toggle_->setCheckable(true);
  advanced_toggle_->setChecked(false);
  advanced_toggle_->setIcon(icons::get(QStringLiteral("chevron-right")));
  advanced_toggle_->setIconSize(QSize(12, 12));
  // setIcon 仅显示一张图标，无法按状态切换；用 paint-time toggle 通过 connect 处理
  connect(advanced_toggle_, &QPushButton::toggled, this, [this](bool checked) {
    advanced_toggle_->setIcon(icons::get(checked ? QStringLiteral("chevron-down")
                                                  : QStringLiteral("chevron-right")));
  });
  outer->addWidget(advanced_toggle_);
  // MIL 是一个整体闭环，拆开启动仿真/桥/烧录没有有效语义；HIL 尚未适配。
  // 两种后端当前都只暴露受控的「启动/停止完整系统」。
  advanced_toggle_->hide();

  advanced_box_ = new QWidget();
  auto* adv_layout = new QVBoxLayout(advanced_box_);
  adv_layout->setContentsMargins(0, 0, 0, 0);
  adv_layout->setSpacing(6);
  start_carla_button_  = make_button(QStringLiteral("car"), QStringLiteral("仅启动 CARLA"),
                                     QStringLiteral("secondaryButton"));
  start_bridge_button_ = make_button(QStringLiteral("sitemap"), QStringLiteral("仅启动桥接"),
                                     QStringLiteral("secondaryButton"));
  stop_bridge_button_  = make_button(QStringLiteral("circle-stop"), QStringLiteral("停止桥接"),
                                     QStringLiteral("outlinedButton"));
  flash_mcu_button_ = make_button(QStringLiteral("microchip"),
                                  QStringLiteral("烧录 MCU 固件（F280025C）"),
                                  QStringLiteral("outlinedButton"));
  flash_mcu_button_->setToolTip(QStringLiteral(
      "调 ~/程序/ti/dslite.sh 烧录 F280025C。\n"
      "不会控制 MCU 电源 / 复位；烧完后请人工按 LAUNCHXL S1。"));
  adv_layout->addWidget(start_carla_button_);
  adv_layout->addWidget(start_bridge_button_);
  adv_layout->addWidget(stop_bridge_button_);
  adv_layout->addWidget(flash_mcu_button_);
  advanced_box_->setVisible(false);
  outer->addWidget(advanced_box_);
}

void LaunchPanel::build_preset_card(QVBoxLayout* root) {
  // 卡片改名 + 副标题：明确这是「场景预设」（只启动哪个 CARLA 场景），与
  // 导航目标（地图手动选点）是两个独立概念。导航不会在按预设时被自动
  // 设置——按完预设若要导航，请在地图上点击。
  auto* preset_box =
      make_card(QStringLiteral("bolt"), QStringLiteral("演示场景 · 一键启动"));
  auto* preset_body = new QWidget();
  auto* body_layout = new QVBoxLayout(preset_body);
  body_layout->setContentsMargins(0, 0, 0, 0);
  body_layout->setSpacing(4);

  auto* hint = new QLabel(manager_.milMode()
      ? QStringLiteral("选择本机模拟场景并一键启动 MIL 闭环。\n"
                       "导航目标可在中间地图的车道附近点击设置。")
      : manager_.carlaLocalMode()
          ? QStringLiteral("catalog 全场景入口；CARLA 展示，Town 地图负责导航选点。")
          : QStringLiteral("HIL 场景入口已保留，当前版本尚未适配硬件。"));
  hint->setWordWrap(true);
  hint->setStyleSheet(QStringLiteral("color:%1;font-size:10px;")
                           .arg(theme::kTextSecondary));
  body_layout->addWidget(hint);

  auto* grid = new QGridLayout();
  grid->setContentsMargins(0, 0, 0, 0);
  grid->setHorizontalSpacing(6);
  grid->setVerticalSpacing(6);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  const auto presets = demo_presets();
  int index = 0;
  for (const auto& preset : presets) {
    auto* button = make_button(QStringLiteral("play"), preset.label,
                               QStringLiteral("secondaryButton"));
    QFont compact_font = button->font();
    compact_font.setPointSize(9);
    button->setFont(compact_font);
    button->setMinimumWidth(0);
    button->setToolTip(preset.note);
    connect(button, &QPushButton::clicked, this,
            [this, preset]() { applyPreset(preset); });
    grid->addWidget(button, index / 2, index % 2);
    preset_buttons_.append(button);
    ++index;
  }
  body_layout->addLayout(grid);
  preset_box->layout()->addWidget(preset_body);
  root->addWidget(preset_box);
}

void LaunchPanel::applyPreset(const DemoPreset& preset) {
  // 桥运行期间预设被禁用（见 update_buttons），此处只在停止态触发。
  const auto pick = [](QComboBox* combo, const QString& value) {
    const int index = combo->findData(value);
    if (index >= 0) combo->setCurrentIndex(index);
  };
  pick(scenario_combo_, preset.scenario);
  pick(town_combo_, preset.town);
  if (!runPreflightGate()) return;
  if (!confirm_action(this, QStringLiteral("确认启动演示场景"),
                      QStringLiteral("启动 %1").arg(preset.label),
                      QStringLiteral("场景：%1，地图：%2；%3")
                          .arg(preset.scenario, preset.town,
                               auto_navigation_check_->isChecked()
                                   ? QStringLiteral("将自动生成并下发车道导航目标。")
                                   : QStringLiteral("请在 Town 地图手动选择目标。")),
                      ConfirmSeverity::Warning)) return;
  emit demoPresetLaunched(preset.scenario, preset.town);
  // 预设与普通/autostart 复用同一会话创建入口；前面的交互确认不创建 UUID。
  startConfiguredSystem(/*interactive=*/false);
}

void LaunchPanel::setAdvancedVisible(bool visible) {
  advanced_box_->setVisible(visible);
}

void LaunchPanel::setHealthRow(const QString& id, HealthState state,
                               const QString& detail) {
  auto* light = health_lights_.value(id, nullptr);
  auto* value = health_values_.value(id, nullptr);
  if (!light || !value) return;
  light->setState(to_led_state(state));
  value->setText(QString::fromUtf8(health_state_name(state)));
  value->setStyleSheet(QStringLiteral("color:%1;font-size:11px;font-weight:600;")
                           .arg(QString::fromLatin1(health_state_color(state))));
  value->setToolTip(detail);
  light->setToolTip(detail);
}

void LaunchPanel::onHealthSnapshot(const QVector<GuiHealthStatus>& statuses) {
  for (const auto& status : statuses) {
    // 健康快照只负责观测显示，绝不能反向改写进程所有权/运行会话。
    // 旧逻辑把任意 ROS graph 残留当成 bridge Running，进而永久锁死场景框。
    // 受管 CARLA 的 RPC 探测比 ROS graph 推断更直接；端口就绪时不被覆盖。
    if (status.id == QStringLiteral("carla") && manager_.carlaReady()) continue;
    if (status.id == QStringLiteral("bridge") &&
        manager_.bridgeState() == ProcState::Starting) continue;
    setHealthRow(status.id, status.state, status.detail);
  }
}

bool LaunchPanel::confirmFlashMcu(const QString& firmware_path) {
  return confirm_action(
      this, QStringLiteral("确认烧录 F280025C 固件"),
      QStringLiteral("擦除并重写 MCU：%1").arg(firmware_path),
      QStringLiteral("GUI 不控制电源/复位；完成后需人工按 LAUNCHXL S1。"),
      ConfirmSeverity::Critical);
}

void LaunchPanel::onFlashMcuClicked() {
  // 让用户选 .out/.hex
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("选择 F280025C 固件"),
      QDir::homePath(),
      QStringLiteral("F280025C 固件 (*.out *.hex);;所有文件 (*)"));
  if (path.isEmpty()) return;
  if (!confirmFlashMcu(path)) return;
  flash_mcu_button_->setBusy(true, QStringLiteral("烧录中"),
                             QStringLiteral("等待 dslite 进程完成"));
  manager_.flashMcuFirmware(path);
  update_buttons();
}

void LaunchPanel::onNeedsOrinCredentials(const QString& host,
                                         const QString& user) {
  // QInputDialog::getText：Ok 后用 saveOrinPassword 落盘 0600。Ok=继续；
  // Cancel=用户放弃本次启动（stackProgress 会停在 "Orin auth" 但状态机
  // 已经回到 Idle，下一次点启动会再次触发 needsOrinCredentials）。
  bool ok = false;
  const QString pwd = QInputDialog::getText(
      this, QStringLiteral("首次启动：填入 Orin ssh 密码"),
      QStringLiteral("Orin 主机：%1@%2\n\n"
                     "密码存到 ~/.config/adas/adas_gui/secrets.ini (chmod 600)。\n"
                     "不会再弹此框（除非密码错误或换 Orin 主机）。")
          .arg(user, host),
      QLineEdit::Password, QString(), &ok);
  if (!ok || pwd.isEmpty()) return;
  const auto r = SecureSettings::instance().saveOrinPassword(host, user, pwd);
  if (!r.ok) {
    QMessageBox::warning(this, QStringLiteral("保存密码失败"),
                         QStringLiteral("无法写入 secrets.ini：%1").arg(r.detail));
    return;
  }
  // 提示用户：现在按"一键启动全流程"即可
  QMessageBox::information(
      this, QStringLiteral("Orin 密码已保存"),
      QStringLiteral("密码已写入 secrets.ini (chmod 600)。\n"
                     "请再次点击「一键启动全流程」按钮继续。"));
}

bool LaunchPanel::runPreflightGate() {
  const QVector<PreflightItem> items = run_preflight(currentConfig());

  bool has_fail = false;
  bool has_warn = false;
  QString report;
  for (const auto& item : items) {
    if (item.level == PreflightLevel::Fail) has_fail = true;
    if (item.level == PreflightLevel::Warn) has_warn = true;
    if (item.level == PreflightLevel::Ok) continue;
    report += QStringLiteral("[%1] %2：%3\n")
                  .arg(preflight_level_name(item.level), item.label, item.detail);
  }

  if (has_fail) {
    QMessageBox box(this);
    box.setIcon(QMessageBox::Critical);
    box.setWindowTitle(QStringLiteral("启动前体检未通过"));
    box.setText(QStringLiteral("以下问题会导致启动失败，已阻断本次启动："));
    box.setInformativeText(report);
    box.setStandardButtons(QMessageBox::Ok);
    box.exec();
    return false;
  }

  if (has_warn) {
    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    box.setWindowTitle(QStringLiteral("启动前体检"));
    box.setText(QStringLiteral("体检发现以下潜在问题，可以继续但请留意："));
    box.setInformativeText(report);
    box.setStandardButtons(QMessageBox::Cancel | QMessageBox::Yes);
    box.setDefaultButton(QMessageBox::Cancel);
    box.button(QMessageBox::Yes)->setText(QStringLiteral("仍要启动"));
    box.button(QMessageBox::Cancel)->setText(QStringLiteral("取消"));
    return box.exec() == QMessageBox::Yes;
  }

  return true;
}

void LaunchPanel::save_settings() const {
  QSettings settings(QStringLiteral("adas"), QStringLiteral("adas_gui"));
  // backend 决定 rclcpp 初始化前的 DDS profile，不跨启动持久化；通过
  // --backend/ADAS_GUI_BACKEND 显式选择，避免一次本地测试污染下次 CARLA HIL。
  settings.remove(QStringLiteral("launch/backend"));
  settings.setValue(QStringLiteral("launch/scenario"), selectedScenario());
  settings.setValue(QStringLiteral("launch/town"), town_combo_->currentText());
  settings.setValue(QStringLiteral("launch/control_source"), source_combo_->currentText());
  settings.setValue(QStringLiteral("launch/carla_root"), carla_root_edit_->text());
  settings.setValue(QStringLiteral("launch/low_quality"), low_quality_check_->isChecked());
  settings.setValue(QStringLiteral("launch/offscreen"), offscreen_check_->isChecked());
  settings.setValue(QStringLiteral("launch/auto_navigation"),
                    auto_navigation_check_->isChecked());
}

void LaunchPanel::restore_settings() {
  QSettings settings(QStringLiteral("adas"), QStringLiteral("adas_gui"));
  const auto pick_text = [](QComboBox* combo, const QString& value) {
    const int index = combo->findText(value);
    if (index >= 0) combo->setCurrentIndex(index);
  };
  QString default_backend =
      environment_or("ADAS_GUI_BACKEND", QStringLiteral("carla_local_soc"));
  if (default_backend.isEmpty()) {
    default_backend = QStringLiteral("carla_local_soc");
  }
  // 旧配置迁移只影响本次启动，不再向 UI 暴露旧四后端。
  if (!known_backends().contains(default_backend)) {
    default_backend = QStringLiteral("carla_local_soc");
  }
  const int backend_index = backend_combo_->findData(default_backend);
  if (backend_index >= 0) backend_combo_->setCurrentIndex(backend_index);
  const QString requested_scenario = qEnvironmentVariableIsSet("ADAS_GUI_SCENARIO")
      ? qEnvironmentVariable("ADAS_GUI_SCENARIO")
      : settings.value(QStringLiteral("launch/scenario"), QStringLiteral("free")).toString();
  rebuildScenarioChoices(requested_scenario);
  pick_text(town_combo_,
       settings.value(QStringLiteral("launch/town"), QStringLiteral("Town04")).toString());
  pick_text(source_combo_,
       settings.value(QStringLiteral("launch/control_source"),
                      environment_or("ADAS_GUI_CONTROL_SOURCE",
                                     QStringLiteral("ros2"))).toString());
  QString saved_root = qEnvironmentVariableIsSet("CARLA_ROOT")
      ? default_carla_root()
      : settings.value(QStringLiteral("launch/carla_root")).toString();
  if (!QFileInfo::exists(QDir(saved_root).filePath(QStringLiteral("CarlaUE4.sh")))) {
    saved_root = default_carla_root();
  }
  carla_root_edit_->setText(saved_root);
  low_quality_check_->setChecked(
      settings.value(QStringLiteral("launch/low_quality"), false).toBool());
  offscreen_check_->setChecked(false);
  auto_navigation_check_->setChecked(
      settings.value(QStringLiteral("launch/auto_navigation"), true).toBool());
}

void LaunchPanel::update_buttons() {
  const bool observer = manager_.localThreeMachineObserver();
  const bool config_locked = manager_.configurationLocked();
  const bool carla_busy = manager_.carlaState() == ProcState::Starting ||
                          manager_.carlaState() == ProcState::Running;
  const bool bridge_busy = manager_.bridgeState() == ProcState::Starting ||
                           manager_.bridgeState() == ProcState::Running;
  start_all_button_->setEnabled(!observer && !start_all_button_->isBusy() && !config_locked);
  start_all_button_->setToolTip(observer
      ? QStringLiteral("外部本地三机编排器已持有进程；当前 GUI 仅观察")
      : config_locked
      ? QStringLiteral("GUI-owned 会话正在启动或运行，请先停止当前系统")
      : QStringLiteral("启动前将执行体检并要求二次确认"));
  for (auto* button : preset_buttons_) {
    button->setEnabled(!observer && !button->isBusy() && !config_locked);
    if (config_locked) button->setToolTip(QStringLiteral("当前 GUI-owned 会话运行中，不能切换场景"));
  }
  start_carla_button_->setEnabled(!observer && !start_carla_button_->isBusy() &&
                                  !carla_busy && !manager_.carlaReady());
  if (carla_busy || manager_.carlaReady()) {
    start_carla_button_->setToolTip(QStringLiteral("CARLA/SIL 已启动或正在启动"));
  }
  start_bridge_button_->setEnabled(!observer && !start_bridge_button_->isBusy() && !bridge_busy);
  if (bridge_busy) start_bridge_button_->setToolTip(QStringLiteral("bridge 已运行或正在启动"));
  stop_bridge_button_->setEnabled(!observer && !stop_bridge_button_->isBusy() &&
                                  manager_.hasManagedBridge());
  if (!manager_.hasManagedBridge()) {
    stop_bridge_button_->setToolTip(QStringLiteral("没有由本 GUI 管理的 bridge 可停止"));
  }
  const bool can_stop = manager_.hasManagedProcesses();
  stop_all_button_->setEnabled(!observer && !stop_all_button_->isBusy() && can_stop);
  stop_all_button_->setToolTip(
      can_stop
          ? QStringLiteral("停止本机受管 CARLA/bridge；Orin HIL/CAN 常驻服务与外部实例保持运行")
          : QStringLiteral("没有受管进程可停止"));
  // 运行配置在桥/CARLA 运行期间锁定，避免界面显示与实际运行场景不一致
  scenario_combo_->setEnabled(!observer && !config_locked);
  carla_root_edit_->setEnabled(!observer && !config_locked);
  carla_browse_button_->setEnabled(!observer && !config_locked);
  low_quality_check_->setEnabled(!observer && !config_locked);
  auto_navigation_check_->setEnabled(!observer && !config_locked);
  start_full_stack_check_->setEnabled(!observer && !config_locked);
  // 后端在 rclcpp::init 前决定 DDS domain/RMW，GUI 内热切换必然造成“界面是
  // local_three_machine、进程仍在 domain43”这类不可用组合。这里只展示当前
  // 启动后端；场景仍可在停止态自由切换。
  backend_combo_->setEnabled(false);
}

bool LaunchPanel::eventFilter(QObject* watched, QEvent* event) {
  if (event->type() == QEvent::Wheel &&
      (watched == town_combo_ || watched == source_combo_)) {
    return true;
  }
  return QWidget::eventFilter(watched, event);
}

}  // namespace adas::gui
