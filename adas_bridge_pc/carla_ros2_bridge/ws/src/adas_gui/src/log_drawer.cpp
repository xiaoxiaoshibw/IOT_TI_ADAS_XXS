#include "log_drawer.hpp"

#include <QDateTime>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QVBoxLayout>

#include <cmath>

#include "icons.hpp"
#include "realtime_plot.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace adas::gui {

namespace {
constexpr int kLogMaxBlocks = 1200;
constexpr int kFaultMaxItems = 500;
}  // namespace

LogDrawer::LogDrawer(QWidget* parent) : QWidget(parent) {
  build_ui();
}

void LogDrawer::build_ui() {
  setObjectName(QStringLiteral("root"));
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(10, 6, 10, 8);
  root->setSpacing(5);

  // 头部：标题 + 折叠按钮（chevron-up / chevron-down）
  auto* header = new QHBoxLayout();
  header->setSpacing(8);
  auto* icon = new IconLabel(QStringLiteral("list-check"), QStringLiteral("运行日志 / CAN / 故障事件"));
  icon->setStyleSheet(QStringLiteral("color:%1;font-size:12px;font-weight:700;letter-spacing:0px;")
                          .arg(theme::kMdPrimary));
  header->addWidget(icon);
  header->addStretch(1);
  toggle_button_ = new QPushButton();
  toggle_button_->setObjectName(QStringLiteral("iconButton"));
  toggle_button_->setIcon(icons::get(QStringLiteral("chevron-down")));
  toggle_button_->setIconSize(QSize(16, 16));
  toggle_button_->setFixedSize(30, 30);
  toggle_button_->setToolTip(QStringLiteral("折叠 / 展开"));
  connect(toggle_button_, &QPushButton::clicked, this, &LogDrawer::toggleExpanded);
  header->addWidget(toggle_button_);
  root->addLayout(header);

  tabs_ = new QTabWidget();
  proc_log_ = new QPlainTextEdit();
  proc_log_->setReadOnly(true);
  proc_log_->setMaximumBlockCount(kLogMaxBlocks);
  proc_log_->setStyleSheet(QStringLiteral(
      "QPlainTextEdit{background:%1;border:1px solid %2;border-radius:6px;"
      "padding:6px;color:%3;font-family:'JetBrains Mono','Cascadia Mono',Consolas,"
      "'DejaVu Sans Mono','Courier New',monospace;font-size:11px;}")
      .arg(theme::kMdSurfaceContainerLow, theme::kCardBorder, theme::kMdOnSurface));
  tabs_->addTab(proc_log_, QStringLiteral("进程日志"));

  can_log_ = new QPlainTextEdit();
  can_log_->setReadOnly(true);
  can_log_->setStyleSheet(proc_log_->styleSheet());
  can_log_->appendPlainText(QStringLiteral("(等待桥接节点经 --can 反馈上报 0x201 帧 ...)"));
  tabs_->addTab(can_log_, QStringLiteral("CAN 报文"));

  fault_log_ = new QListWidget();
  fault_log_->addItem(QStringLiteral("(等待真实故障与状态变化事件 ...)"));
  tabs_->addTab(fault_log_, QStringLiteral("故障事件"));

  auto setup_chart_layout = [](QWidget* panel) {
    auto* layout = new QHBoxLayout(panel);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(8);
    return layout;
  };

  auto* vehicle_panel = new QWidget();
  auto* vehicle_layout = setup_chart_layout(vehicle_panel);
  speed_plot_ = new RealtimePlot(QStringLiteral("实际车速"), QStringLiteral("km/h"),
                                 QColor(theme::kMdPrimary));
  speed_plot_->setRange(0.0, 120.0);
  target_speed_plot_ = new RealtimePlot(QStringLiteral("目标车速"), QStringLiteral("km/h"),
                                        QColor(theme::kMdTertiary));
  target_speed_plot_->setRange(0.0, 120.0);
  yaw_rate_plot_ = new RealtimePlot(QStringLiteral("横摆角速度"), QStringLiteral("deg/s"),
                                    QColor(QStringLiteral("#8bb9ff")));
  yaw_rate_plot_->setRange(-60.0, 60.0);
  vehicle_layout->addWidget(speed_plot_, 1);
  vehicle_layout->addWidget(target_speed_plot_, 1);
  vehicle_layout->addWidget(yaw_rate_plot_, 1);
  tabs_->addTab(vehicle_panel, QStringLiteral("车辆曲线"));

  auto* control_panel = new QWidget();
  auto* control_layout = setup_chart_layout(control_panel);
  steer_plot_ = new RealtimePlot(QStringLiteral("转向指令"), QStringLiteral("%"),
                                 QColor(QStringLiteral("#8bb9ff")));
  steer_plot_->setRange(-100.0, 100.0);
  throttle_plot_ = new RealtimePlot(QStringLiteral("油门指令"), QStringLiteral("%"),
                                    QColor(QStringLiteral("#66d69a")));
  throttle_plot_->setRange(0.0, 100.0);
  brake_plot_ = new RealtimePlot(QStringLiteral("制动指令"), QStringLiteral("%"),
                                 QColor(QStringLiteral("#ff8a8a")));
  brake_plot_->setRange(0.0, 100.0);
  control_layout->addWidget(steer_plot_, 1);
  control_layout->addWidget(throttle_plot_, 1);
  control_layout->addWidget(brake_plot_, 1);
  tabs_->addTab(control_panel, QStringLiteral("控制曲线"));

  auto* safety_panel = new QWidget();
  auto* safety_layout = setup_chart_layout(safety_panel);
  ttc_plot_ = new RealtimePlot(QStringLiteral("TTC"), QStringLiteral("s"),
                               QColor(theme::kWarn));
  ttc_plot_->setRange(0.0, 10.0);
  lateral_plot_ = new RealtimePlot(QStringLiteral("横向误差"), QStringLiteral("m"),
                                   QColor(theme::kAccent));
  lateral_plot_->setRange(-2.0, 2.0);
  required_decel_plot_ = new RealtimePlot(QStringLiteral("AEB 需求减速度"),
                                          QStringLiteral("m/s²"),
                                          QColor(theme::kMdError));
  required_decel_plot_->setRange(0.0, 12.0);
  safety_layout->addWidget(ttc_plot_, 1);
  safety_layout->addWidget(lateral_plot_, 1);
  safety_layout->addWidget(required_decel_plot_, 1);
  tabs_->addTab(safety_panel, QStringLiteral("安全曲线"));

  tabs_->setStyleSheet(QStringLiteral(
      "QTabWidget::pane{border:1px solid %1;border-radius:8px;background:%2;}"
      "QTabBar::tab{padding:6px 12px;}")
      .arg(theme::kCardBorder, theme::kMdSurfaceContainerLow));

  root->addWidget(tabs_, 1);
}

void LogDrawer::update_chevron() {
  toggle_button_->setIcon(icons::get(expanded_ ? QStringLiteral("chevron-down")
                                                : QStringLiteral("chevron-up")));
}

void LogDrawer::setExpanded(bool expanded) {
  if (expanded_ == expanded) return;
  expanded_ = expanded;
  tabs_->setVisible(expanded);
  update_chevron();
  emit expandedChanged(expanded);
}

void LogDrawer::toggleExpanded() {
  setExpanded(!expanded_);
}

void LogDrawer::appendLog(const QString& tag, const QString& line) {
  static const QMap<QString, QString> tag_color = {
      {QStringLiteral("CARLA"),  QStringLiteral("#8bb0d4")},
      {QStringLiteral("BRIDGE"), QStringLiteral("#7bc5a6")},
      {QStringLiteral("ERROR"),  QStringLiteral("#ef9a9a")},
      {QStringLiteral("WARN"),   QStringLiteral("#ffe082")},
  };
  const QColor color = QColor(tag_color.value(tag, theme::kTextPrimary));
  const QString html = QStringLiteral(
      "<span style=\"color:%1;font-weight:600;\">[%2]</span> "
      "<span style=\"color:%3;\">%4</span>")
      .arg(color.name(), tag, theme::kMdOnSurface, line.toHtmlEscaped());
  proc_log_->appendHtml(html);
  // 进程管理器显式标为 ERROR 的输出也进入结构化故障时间线。
  if (tag == QStringLiteral("ERROR")) {
    appendFaultEvent(QDateTime::currentMSecsSinceEpoch(), 2,
                     QStringLiteral("进程"), QStringLiteral("进程异常"), line);
  }
}

void LogDrawer::onEgoTelemetry(double, double, double, double speed_mps,
                               double yaw_rate_rps) {
  if (std::isfinite(speed_mps)) {
    speed_plot_->appendValue(std::max(0.0, speed_mps) * 3.6);
  }
  if (std::isfinite(yaw_rate_rps)) {
    constexpr double kRadiansToDegrees = 57.29577951308232;
    yaw_rate_plot_->appendValue(yaw_rate_rps * kRadiansToDegrees);
  }
}

void LogDrawer::onBehaviorTelemetry(int, double target_speed_mps, int) {
  if (std::isfinite(target_speed_mps)) {
    target_speed_plot_->appendValue(std::max(0.0, target_speed_mps) * 3.6);
  }
}

void LogDrawer::onActuationTelemetry(double steer, double throttle, double brake) {
  if (std::isfinite(steer)) steer_plot_->appendValue(steer * 100.0);
  if (std::isfinite(throttle)) throttle_plot_->appendValue(throttle * 100.0);
  if (std::isfinite(brake)) brake_plot_->appendValue(brake * 100.0);
}

void LogDrawer::onAebTelemetry(int, double ttc_s, double required_decel_mps2) {
  // 发布方用大有限值表达 +inf（无碰撞风险）；仍保留该真实采样，绘制时按
  // 0..10 s 视窗裁剪到顶部，风险出现后曲线会从顶部下降。
  if (std::isfinite(ttc_s) && ttc_s > 0.0) {
    ttc_plot_->appendValue(ttc_s);
  }
  if (std::isfinite(required_decel_mps2)) {
    required_decel_plot_->appendValue(std::abs(required_decel_mps2));
  }
}

void LogDrawer::onLaneTelemetry(double lateral_offset_m, bool valid) {
  if (valid && std::isfinite(lateral_offset_m)) {
    lateral_plot_->appendValue(lateral_offset_m);
  }
}

void LogDrawer::appendFaultEvent(qint64 timestamp_ms, int severity,
                                 const QString& source, const QString& title,
                                 const QString& detail, bool recovered) {
  if (fault_log_->count() == 1 &&
      fault_log_->item(0)->text().startsWith(QStringLiteral("(等待"))) {
    delete fault_log_->takeItem(0);
  }
  const QDateTime when = QDateTime::fromMSecsSinceEpoch(timestamp_ms);
  const QString state = recovered ? QStringLiteral("恢复")
      : severity >= 3 ? QStringLiteral("故障")
      : severity == 2 ? QStringLiteral("严重")
      : severity == 1 ? QStringLiteral("告警") : QStringLiteral("信息");
  QString text = QStringLiteral("%1   [%2] %3 · %4")
                     .arg(when.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")),
                          source, state, title);
  if (!detail.isEmpty()) text += QStringLiteral("  —  %1").arg(detail);
  auto* item = new QListWidgetItem(text);
  if (recovered) {
    item->setIcon(icons::get(QStringLiteral("circle-check")));
    item->setForeground(QColor(QStringLiteral("#a5d6a7")));
  } else if (severity >= 2) {
    item->setIcon(icons::get(QStringLiteral("triangle-exclamation")));
    item->setForeground(QColor(QStringLiteral("#ef9a9a")));
    item->setBackground(QColor(QStringLiteral("#3a2020")));
  } else if (severity == 1) {
    item->setIcon(icons::get(QStringLiteral("circle-exclamation")));
    item->setForeground(QColor(QStringLiteral("#ffe082")));
  } else {
    item->setIcon(icons::get(QStringLiteral("clock")));
  }
  item->setToolTip(text);
  fault_log_->insertItem(0, item);
  while (fault_log_->count() > kFaultMaxItems) {
    delete fault_log_->takeItem(fault_log_->count() - 1);
  }
}

void LogDrawer::onDtcHistory(const QString& json) {
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  const auto records = document.object().value(QStringLiteral("records")).toArray();
  for (int i = 0; i < records.size(); ++i) {
    const auto record = records.at(i).toObject();
    const QString code = record.value(QStringLiteral("code")).toString();
    if (code.isEmpty()) continue;
    const bool active = record.value(QStringLiteral("active")).toBool();
    const int occurrences = record.value(QStringLiteral("occurrences")).toInt();
    if (have_dtc_snapshot_) {
      const bool new_occurrence = occurrences > dtc_occurrences_.value(code);
      const bool changed = active != dtc_active_.value(code);
      if (new_occurrence || changed) {
        const qint64 timestamp_ms = static_cast<qint64>(
            record.value(QStringLiteral("last_seen_unix")).toDouble() * 1000.0);
        const QString severity_text = record.value(QStringLiteral("severity")).toString();
        const bool warning = severity_text.compare(QStringLiteral("warning"), Qt::CaseInsensitive) == 0;
        const bool error = severity_text.compare(QStringLiteral("error"), Qt::CaseInsensitive) == 0;
        // fault_catalog 的 CRITICAL / EMERGENCY / LOCKED 均属于红色高危级。
        const int severity = warning ? 1 : error ? 2 : 3;
        const QString detail = QStringLiteral("%1 · 次数 %2%3")
            .arg(record.value(QStringLiteral("safety_action")).toString())
            .arg(occurrences)
            .arg(record.value(QStringLiteral("latched")).toBool()
                     ? QStringLiteral(" · 闭锁") : QString());
        appendFaultEvent(timestamp_ms > 0 ? timestamp_ms : QDateTime::currentMSecsSinceEpoch(),
                         severity, record.value(QStringLiteral("source")).toString(),
                         code + QStringLiteral(" ") + record.value(QStringLiteral("name")).toString(),
                         detail, !active);
      }
    }
    // 始终同步状态：首帧用于初始化，后续用于 diff 比较。
    dtc_active_.insert(code, active);
    dtc_occurrences_.insert(code, occurrences);
  }
  // 首帧只初始化状态、不发事件：当前活动 DTC 已由 SafetyPanel::onDtcHistory
  // 渲染在"活动 DTC"列表里，避免日志时间线被一次性几十条同质事件淹没。
  have_dtc_snapshot_ = true;
}

}  // namespace adas::gui
