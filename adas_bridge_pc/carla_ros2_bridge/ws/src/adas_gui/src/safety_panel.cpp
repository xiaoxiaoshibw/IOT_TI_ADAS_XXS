#include "safety_panel.hpp"

#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>

#include <cmath>

#include "format.hpp"
#include "icons.hpp"
#include "theme.hpp"
#include "widgets.hpp"

namespace adas::gui {
namespace {

constexpr qint64 kMcuStaleMs = 500;
constexpr qint64 kActuationStaleMs = 500;
constexpr qint64 kEgoStaleMs = 500;

QLabel* make_value_label() {
  auto* label = new QLabel(QStringLiteral("--"));
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setStyleSheet(QStringLiteral("color:%1;font-size:13px;")
                           .arg(theme::kTextPrimary));
  label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  label->setMinimumWidth(48);
  return label;
}

QLabel* make_key_label(const QString& text) {
  auto* label = new QLabel(text);
  label->setStyleSheet(QStringLiteral("color:%1;font-size:11px;letter-spacing:0px;")
                           .arg(theme::kTextSecondary));
  return label;
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

void add_kv(QGridLayout* grid, int row, const QString& key, QLabel* value) {
  grid->addWidget(make_key_label(key), row, 0);
  grid->addWidget(value, row, 1);
}

}  // namespace

SafetyPanel::SafetyPanel(QWidget* parent) : QWidget(parent) {
  build_ui();
}

void SafetyPanel::build_ui() {
  setObjectName(QStringLiteral("root"));
  setMinimumWidth(292);
  setMaximumWidth(360);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(8, 10, 10, 8);
  outer->setSpacing(8);

  auto* heading = new QHBoxLayout();
  auto* title = new IconLabel(QStringLiteral("shield"), QStringLiteral("实时安全态势"));
  title->setStyleSheet(QStringLiteral(
      "color:%1;font-size:15px;font-weight:700;letter-spacing:0px;")
      .arg(theme::kTextPrimary));
  auto* live = new QLabel(QStringLiteral("LIVE"));
  live->setStyleSheet(QStringLiteral(
      "color:%1;background:%2;border:1px solid %3;border-radius:5px;"
      "padding:3px 7px;font-size:10px;font-weight:700;")
      .arg(QStringLiteral("#8fe3ad"), theme::color_with_alpha(theme::kOk, 45),
           theme::kCardBorder));
  heading->addWidget(title);
  heading->addStretch(1);
  heading->addWidget(live);
  outer->addLayout(heading);

  banner_ = new StatusBanner();
  banner_->setText(QStringLiteral("等待遥测"));
  banner_->setIconName(QStringLiteral("microchip"));
  banner_->setBannerColor(QColor(theme::kStale));
  outer->addWidget(banner_);

  auto* scroll = new QScrollArea();
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* content = new QWidget();
  content->setObjectName(QStringLiteral("sidebarContent"));
  auto* root = new QVBoxLayout(content);
  root->setContentsMargins(3, 0, 0, 0);
  root->setSpacing(8);
  root->setSizeConstraint(QLayout::SetMinAndMaxSize);
  scroll->setWidget(content);

  // ===== 卡片 1: 控制源 & 主源健康度 =====
  auto* source_card = make_card(QStringLiteral("sitemap"), QStringLiteral("控制源 / 链路"));
  auto* source_body = new QWidget();
  auto* source_grid = new QGridLayout(source_body);
  source_grid->setHorizontalSpacing(10);
  source_grid->setVerticalSpacing(4);
  source_grid->setColumnStretch(1, 1);
  source_value_   = make_value_label();
  primary_value_  = make_value_label();
  mcu_value_      = make_value_label();
  can_value_      = make_value_label();
  protocol_value_ = make_value_label();
  handover_value_ = make_value_label();
  manual_value_   = make_value_label();
  add_kv(source_grid, 0, QStringLiteral("当前控制源"), source_value_);
  add_kv(source_grid, 1, QStringLiteral("主源健康"),  primary_value_);
  add_kv(source_grid, 2, QStringLiteral("MCU 在线"),   mcu_value_);
  add_kv(source_grid, 3, QStringLiteral("CAN 链路"),   can_value_);
  add_kv(source_grid, 4, QStringLiteral("协议版本"),   protocol_value_);
  add_kv(source_grid, 5, QStringLiteral("当前源持续"), handover_value_);
  add_kv(source_grid, 6, QStringLiteral("驾驶员接管"), manual_value_);
  source_card->layout()->addWidget(source_body);
  root->addWidget(source_card);

  // ===== 卡片 2: AEB / 安全链 =====
  auto* aeb_card = make_card(QStringLiteral("bolt"), QStringLiteral("AEB / 安全链"));
  auto* aeb_body = new QWidget();
  auto* aeb_grid = new QGridLayout(aeb_body);
  aeb_grid->setHorizontalSpacing(10);
  aeb_grid->setVerticalSpacing(4);
  aeb_grid->setColumnStretch(1, 1);
  aeb_value_  = make_value_label();
  ttc_value_  = make_value_label();
  safety_value_ = make_value_label();
  add_kv(aeb_grid, 0, QStringLiteral("AEB 状态"), aeb_value_);
  add_kv(aeb_grid, 1, QStringLiteral("TTC"),      ttc_value_);
  add_kv(aeb_grid, 2, QStringLiteral("安全链等级"), safety_value_);
  aeb_card->layout()->addWidget(aeb_body);
  root->addWidget(aeb_card);

  // ===== 卡片 3: 车辆动力学 =====
  auto* veh_card = make_card(QStringLiteral("gauge-high"), QStringLiteral("车辆/执行"));
  auto* veh_body = new QWidget();
  auto* veh_grid = new QGridLayout(veh_body);
  veh_grid->setHorizontalSpacing(10);
  veh_grid->setVerticalSpacing(4);
  veh_grid->setColumnStretch(1, 1);
  speed_value_    = make_value_label();
  speed_value_->setStyleSheet(QStringLiteral("color:%1;font-size:18px;font-weight:700;")
                                  .arg(theme::kTextPrimary));
  behavior_value_ = make_value_label();
  // 大号车速独占第一行
  veh_grid->addWidget(make_key_label(QStringLiteral("车速")), 0, 0, Qt::AlignTop);
  veh_grid->addWidget(speed_value_, 0, 1);
  gate_value_     = make_value_label();
  steer_value_    = make_value_label();
  add_kv(veh_grid, 1, QStringLiteral("行为/Gate"), behavior_value_);
  add_kv(veh_grid, 2, QStringLiteral("Gate 源"),    gate_value_);
  add_kv(veh_grid, 3, QStringLiteral("位置/航向"), pose_value_ = make_value_label());
  add_kv(veh_grid, 4, QStringLiteral("方向盘"),    steer_value_);
  lateral_value_ = make_value_label();
  add_kv(veh_grid, 5, QStringLiteral("横向误差"), lateral_value_);

  auto* bar_row = new QHBoxLayout();
  bar_row->setSpacing(8);
  throttle_bar_ = new SegmentedBar();
  throttle_bar_->setRange(0, 100);
  throttle_bar_->setColor(QColor("#66bb6a"));
  throttle_bar_->setIconName(QStringLiteral("bolt"));
  brake_bar_ = new SegmentedBar();
  brake_bar_->setRange(0, 100);
  brake_bar_->setColor(QColor(theme::kDanger));
  brake_bar_->setIconName(QStringLiteral("circle-stop"));
  bar_row->addWidget(throttle_bar_, 1);
  bar_row->addWidget(brake_bar_, 1);
  veh_grid->addWidget(make_key_label(QStringLiteral("油门/制动")), 6, 0, Qt::AlignTop);
  veh_grid->addLayout(bar_row, 6, 1);
  veh_card->layout()->addWidget(veh_body);
  root->addWidget(veh_card);

  // ===== 卡片 4: 活动 DTC =====
  auto* dtc_card = make_card(QStringLiteral("list-check"), QStringLiteral("活动 DTC"));
  auto* dtc_body = new QWidget();
  auto* dtc_layout = new QVBoxLayout(dtc_body);
  dtc_layout->setSpacing(4);
  dtc_list_ = new QListWidget();
  dtc_list_->setMinimumHeight(110);
  dtc_list_->setMaximumHeight(180);
  dtc_list_->addItem(QStringLiteral("(等待 /mcu/dtc_history ..."));
  dtc_layout->addWidget(dtc_list_);
  dtc_card->layout()->addWidget(dtc_body);
  root->addWidget(dtc_card);
  root->addStretch(1);
  outer->addWidget(scroll, 1);
}

void SafetyPanel::set_value_label(QLabel* label, const QString& text, const char* color) {
  label->setText(text);
  label->setStyleSheet(
      QStringLiteral("color:%1;font-size:13px;%2")
          .arg(color ? QString::fromLatin1(color) : QString::fromLatin1(theme::kTextPrimary),
               color ? QStringLiteral("font-weight:600;") : QString()));
}

void SafetyPanel::update_link_lights(const GuiMcuStatus& status, bool fresh) {
  // MCU 在线：状态遥测在 500ms 内有效即"在线"；否则按状态机退化
  set_value_label(mcu_value_, fresh ? QStringLiteral("● 在线")
                                     : QStringLiteral("○ 离线"),
                  fresh ? theme::kOk : theme::kStale);
  // CAN 链路：用反馈年龄 + protocol_version_ok 推断
  const bool can_ok = status.feedback_age_s >= 0.0f && status.feedback_age_s < 0.5f
                          && status.protocol_version_ok;
  set_value_label(can_value_, can_ok ? QStringLiteral("● 正常")
                                     : QStringLiteral("○ 异常"),
                  can_ok ? theme::kOk : theme::kWarn);
}

void SafetyPanel::onMcuStatus(const GuiMcuStatus& status) {
  const bool first_status = !have_status_;
  const GuiMcuStatus previous = last_status_;
  last_status_ = status;
  // 标记 MCU 通道新鲜；onStaleCheck 与 MainWindow 状态栏 LED 都从这里取。
  TelemetryFreshness::instance().markFresh(TelemetryFreshness::Mcu);
  have_status_ = true;
  // 用于 source-change / manual-override 持续时间计算的参考时间。
  // 之前用 last_status_ms_（MCU 消息到达时刻），语义是"MCU 停则计时冻结"。
  // 改为当前 wall-clock：MCU 停了 banner 会显式说"遥测断流"，duration 显示
  // 不再冻结更符合直觉。
  const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();

  banner_->setText(QString("%1 · %2")
                       .arg(QString::fromLatin1(state_name(status.system_state)))
                       .arg(QString::fromLatin1(source_name(status.active_source))));
  banner_->setBannerColor(QColor(state_color(status.system_state)));
  const auto s = status.system_state;
  const QString icon_key =
      (s == 5U || s == 6U || s == 7U) ? QStringLiteral("triangle-exclamation")
      : (s == 3U || s == 4U)          ? QStringLiteral("circle-exclamation")
      : (s == 2U)                    ? QStringLiteral("circle-check")
                                      : QStringLiteral("microchip");
  banner_->setIconName(icon_key);
  if (first_status || previous.system_state != status.system_state) {
    banner_->pulse(QColor(state_color(status.system_state)));
    if (status.system_state >= 3U) {
      emit faultEvent(status.system_state >= 5U ? 3 : 1,
                      QStringLiteral("MCU"),
                      QStringLiteral("安全状态切换为 %1")
                          .arg(QString::fromLatin1(state_name(status.system_state))),
                      status.degrade_reason, false);
    } else if (!first_status && previous.system_state >= 3U) {
      emit faultEvent(0, QStringLiteral("MCU"),
                      QStringLiteral("安全状态恢复为 %1")
                          .arg(QString::fromLatin1(state_name(status.system_state))),
                      QString(), true);
    }
  }
  if (!first_status && previous.fault_code != status.fault_code) {
    const quint16 raised = static_cast<quint16>(status.fault_code & ~previous.fault_code);
    const quint16 cleared = static_cast<quint16>(previous.fault_code & ~status.fault_code);
    if (raised != 0U) {
      emit faultEvent(2, QStringLiteral("MCU DTC"), QStringLiteral("新增故障位"),
                      QString::fromStdString(fault_bits_text(raised)), false);
    }
    if (cleared != 0U) {
      emit faultEvent(0, QStringLiteral("MCU DTC"), QStringLiteral("故障位清除"),
                      QString::fromStdString(fault_bits_text(cleared)), true);
    }
  } else if (first_status && status.fault_code != 0U &&
             !(status.system_state == 1U &&
               (status.fault_code & static_cast<quint16>(~0x0011U)) == 0U)) {
    emit faultEvent(2, QStringLiteral("MCU DTC"), QStringLiteral("当前活动故障"),
                    QString::fromStdString(fault_bits_text(status.fault_code)), false);
  }

  // 接管流程可视化：控制源切换（PRIMARY/BACKUP/看门狗接管）计时与事件
  if (first_status) {
    last_source_change_ms_ = now_ms;
    last_active_source_ = status.active_source;
  } else if (status.active_source != last_active_source_) {
    const qint64 held_ms = last_source_change_ms_ >= 0
                               ? now_ms - last_source_change_ms_ : -1;
    emit faultEvent(status.active_source == 9U ? 3 : 2,
                    QStringLiteral("接管流程"),
                    QString("控制源切换 %1 → %2")
                        .arg(QString::fromLatin1(source_name(last_active_source_)))
                        .arg(QString::fromLatin1(source_name(status.active_source))),
                    held_ms >= 0 ? QStringLiteral("原源持续 %1").arg(
                                       QString::fromStdString(format_age(held_ms / 1000.0f)))
                                 : QString(),
                    false);
    last_source_change_ms_ = now_ms;
    last_active_source_ = status.active_source;
  }
  // 驾驶员手动接管（HIL 台架物理按钮/踏板触发，非 GUI 下发）
  if (status.manual_override != last_manual_override_) {
    if (status.manual_override) {
      manual_override_since_ms_ = now_ms;
      emit faultEvent(2, QStringLiteral("接管流程"),
                      QStringLiteral("驾驶员手动接管"), QString(), false);
      banner_->pulse(QColor(theme::kWarn));
    } else {
      const qint64 held_ms = manual_override_since_ms_ >= 0
                                 ? now_ms - manual_override_since_ms_ : -1;
      emit faultEvent(0, QStringLiteral("接管流程"),
                      QStringLiteral("驾驶员接管解除"),
                      held_ms >= 0 ? QStringLiteral("接管持续 %1").arg(
                                         QString::fromStdString(format_age(held_ms / 1000.0f)))
                                       : QString(),
                      true);
      manual_override_since_ms_ = -1;
    }
    last_manual_override_ = status.manual_override;
  }
  set_value_label(manual_value_,
                  status.manual_override ? QStringLiteral("● 接管中") : QStringLiteral("--"),
                  status.manual_override ? theme::kWarn : nullptr);

  source_value_->setText(QString::fromLatin1(source_name(status.active_source)));
  set_value_label(primary_value_, status.primary_fresh ? QStringLiteral("● 健康")
                                                        : QStringLiteral("○ 超时"),
                   status.primary_fresh ? theme::kOk : theme::kDanger);
  protocol_value_->setText(QString("v%1 %2%3")
                               .arg(status.protocol_version)
                               .arg(status.protocol_version_ok ? "OK" : "MISMATCH")
                               .arg(status.test_build ? "  [TEST]" : ""));
  update_link_lights(status, /*fresh=*/true);

  // 故障码 DTC 行：主源超时即告警
  if (status.system_state != 1U && !status.primary_fresh) {
    emit alertChanged(QStringLiteral("sources"),
                      QStringLiteral("主源超时"), true);
  } else {
    emit alertChanged(QStringLiteral("sources"), QString(), false);
  }
  emit alertChanged(QStringLiteral("estop"),
                    QStringLiteral("MCU ESTOP 触发"), status.estop);
}

void SafetyPanel::onActuation(const GuiActuation& actuation) {
  TelemetryFreshness::instance().markFresh(TelemetryFreshness::Actuation);
  steer_value_->setText(QString("%1 %").arg(actuation.steer * 100.0f, 0, 'f', 0));
  throttle_bar_->setValue(static_cast<int>(actuation.throttle * 100.0f + 0.5f));
  brake_bar_->setValue(static_cast<int>(actuation.brake * 100.0f + 0.5f));
}

void SafetyPanel::onBehavior(int state, double target_speed_mps, int target_lane) {
  QString text = QString::fromLatin1(behavior_state_name(static_cast<std::uint8_t>(state)));
  text += QString(" %1 km/h").arg(target_speed_mps * 3.6, 0, 'f', 0);
  if (target_lane != 0) text += target_lane < 0 ? QStringLiteral(" ←左邻道")
                                                : QStringLiteral(" →右邻道");
  behavior_value_->setText(text);
}

void SafetyPanel::onGate(int source, bool limited, const QString& reason) {
  QString text = QString::fromLatin1(gate_source_name(static_cast<std::uint8_t>(source)));
  if (limited) text += QStringLiteral(" 限幅");
  if (!reason.isEmpty()) text += QString(" (%1)").arg(reason);
  gate_value_->setText(text);
}

void SafetyPanel::onAeb(int state, double ttc_s, double required_decel_mps2) {
  QString text = QString::fromLatin1(aeb_state_name(static_cast<std::uint8_t>(state)));
  if (state >= 2 && ttc_s < 1.0e5) {
    text += QString(" 需 %1 m/s²").arg(required_decel_mps2, 0, 'f', 1);
  }
  aeb_value_->setText(text);
  aeb_value_->setStyleSheet(
      state >= 3 ? theme::value_style(theme::kDanger, true)
      : state == 2 ? theme::value_style(theme::kWarn, true)
                   : theme::value_style(theme::kTextPrimary));
  if (ttc_s > 0.0 && ttc_s < 1.0e5) {
    ttc_value_->setText(QString("%1 s").arg(ttc_s, 0, 'f', 2));
    if (ttc_s < 1.5) ttc_value_->setStyleSheet(theme::value_style(theme::kDanger, true));
    else if (ttc_s < 3.0) ttc_value_->setStyleSheet(theme::value_style(theme::kWarn, true));
    else ttc_value_->setStyleSheet(theme::value_style(theme::kOk));
  } else {
    ttc_value_->setText(QStringLiteral("-- s"));
    ttc_value_->setStyleSheet(theme::value_style(theme::kTextPrimary));
  }
  emit alertChanged(QStringLiteral("aeb"),
                    QStringLiteral("AEB 制动中 (TTC %1 s)").arg(ttc_s, 0, 'f', 1),
                    state >= 3);
  emit alertChanged(QStringLiteral("aeb_warn"),
                    QStringLiteral("AEB 预警 (TTC %1 s)").arg(ttc_s, 0, 'f', 1),
                    state == 2);
  if (state != last_aeb_state_) {
    if (state >= 2) {
      banner_->pulse(QColor(state >= 3 ? theme::kDanger : theme::kWarn));
      emit faultEvent(state >= 3 ? 3 : 1, QStringLiteral("AEB"),
                      QStringLiteral("AEB %1")
                          .arg(QString::fromLatin1(aeb_state_name(
                              static_cast<std::uint8_t>(state)))),
                      std::isfinite(ttc_s) && ttc_s < 1.0e5
                          ? QStringLiteral("TTC %1 s").arg(ttc_s, 0, 'f', 2)
                          : QString(), false);
    } else if (last_aeb_state_ >= 2) {
      emit faultEvent(0, QStringLiteral("AEB"), QStringLiteral("AEB 风险解除"),
                      QString(), true);
    }
    last_aeb_state_ = state;
  }
}

void SafetyPanel::onSafety(int overall, const QString& failed) {
  QString text = QString::fromLatin1(safety_level_name(static_cast<std::uint8_t>(overall)));
  if (!failed.isEmpty()) text += QString(" (%1)").arg(failed);
  safety_value_->setText(text);
  safety_value_->setStyleSheet(
      overall >= 2 ? theme::value_style(theme::kDanger, true)
      : overall == 1 ? theme::value_style(theme::kWarn, true)
                      : theme::value_style(theme::kOk));
  emit alertChanged(QStringLiteral("safety"),
                    QStringLiteral("安全链故障: %1").arg(failed), overall >= 2);
  if (overall != last_safety_level_) {
    if (overall >= 1) {
      banner_->pulse(QColor(overall >= 2 ? theme::kDanger : theme::kWarn));
      emit faultEvent(overall >= 2 ? 3 : 1, QStringLiteral("Safety Monitor"),
                      QStringLiteral("安全链切换为 %1")
                          .arg(QString::fromLatin1(safety_level_name(
                              static_cast<std::uint8_t>(overall)))),
                      failed, false);
    } else if (last_safety_level_ >= 1) {
      emit faultEvent(0, QStringLiteral("Safety Monitor"),
                      QStringLiteral("安全链恢复正常"), failed, true);
    }
    last_safety_level_ = overall;
  }
}

void SafetyPanel::onEgo(double x, double y, double yaw_rad, double speed_mps,
                       double yaw_rate_rps) {
  TelemetryFreshness::instance().markFresh(TelemetryFreshness::Ego);
  // 非有限值守卫：bridge 在异常情况下可能发布 NaN/Inf odom，避免显示 "nan km/h"
  // 并让 stale 检查按预期工作。
  if (std::isfinite(speed_mps)) {
    speed_value_->setText(QString("%1 km/h").arg(speed_mps * 3.6, 0, 'f', 1));
  } else {
    speed_value_->setText(QStringLiteral("-- km/h"));
  }
  if (std::isfinite(x) && std::isfinite(y) &&
      std::isfinite(yaw_rad) && std::isfinite(yaw_rate_rps)) {
    pose_value_->setText(QString("(%1, %2) m  %3°/%4°/s")
                             .arg(x, 0, 'f', 1)
                             .arg(y, 0, 'f', 1)
                             .arg(yaw_rad * 180.0 / M_PI, 0, 'f', 0)
                             .arg(yaw_rate_rps * 180.0 / M_PI, 0, 'f', 0));
  } else {
    pose_value_->setText(QStringLiteral("--"));
  }
}

void SafetyPanel::onLaneState(double lateral_offset_m, bool valid) {
  if (!valid || !std::isfinite(lateral_offset_m)) {
    set_value_label(lateral_value_, QStringLiteral("-- m"), theme::kStale);
    return;
  }
  const double absolute = std::abs(lateral_offset_m);
  const char* color = absolute >= 0.8 ? theme::kDanger
                    : absolute >= 0.4 ? theme::kWarn : theme::kOk;
  set_value_label(lateral_value_,
                  QStringLiteral("%1 m").arg(lateral_offset_m, 0, 'f', 3), color);
}

void SafetyPanel::onDtcHistory(const QString& json) {
  dtc_list_->clear();
  const auto document = QJsonDocument::fromJson(json.toUtf8());
  for (const auto& value : document.object().value(QStringLiteral("records")).toArray()) {
    const auto record = value.toObject();
    if (!record.value(QStringLiteral("active")).toBool()) continue;
    auto* item = new QListWidgetItem(
        QString("%1  %2  x%3  %4  %5")
            .arg(record.value(QStringLiteral("code")).toString())
            .arg(record.value(QStringLiteral("name")).toString())
            .arg(record.value(QStringLiteral("occurrences")).toInt())
            .arg(record.value(QStringLiteral("safety_action")).toString())
            .arg(record.value(QStringLiteral("latched")).toBool()
                     ? QStringLiteral("  [闭锁]") : QString()));
    if (record.value(QStringLiteral("latched")).toBool()) {
      item->setForeground(QColor(theme::kDanger));
      item->setBackground(QColor::fromString(QStringLiteral("#3a2020")));
    }
    dtc_list_->addItem(item);
  }
  if (dtc_list_->count() == 0) {
    dtc_list_->addItem(QStringLiteral("(无活动 DTC)"));
  }
}

void SafetyPanel::onStaleCheck(qint64 now_ms) {
  const auto& f = TelemetryFreshness::instance();
  if (have_status_) {
    const bool mcu_fresh = f.isFresh(TelemetryFreshness::Mcu, kMcuStaleMs);
    if (!mcu_fresh) {
      banner_->setText(QStringLiteral("遥测断流"));
      banner_->setBannerColor(QColor(theme::kStale));
      banner_->setIconName(QStringLiteral("wifi"));
      update_link_lights(last_status_, false);
    } else {
      update_link_lights(last_status_, true);
    }
  }
  if (!f.isFresh(TelemetryFreshness::Actuation, kActuationStaleMs)) {
    throttle_bar_->setValue(0);
    brake_bar_->setValue(100);
  }
  if (!f.isFresh(TelemetryFreshness::Ego, kEgoStaleMs)) {
    speed_value_->setText(QStringLiteral("-- km/h"));
  }
  if (have_status_ && last_source_change_ms_ >= 0) {
    handover_value_->setText(QString::fromStdString(
        format_age(static_cast<float>(now_ms - last_source_change_ms_) / 1000.0f)));
  }
  if (last_manual_override_ && manual_override_since_ms_ >= 0) {
    set_value_label(manual_value_,
                    QStringLiteral("● 接管中 %1").arg(QString::fromStdString(
                        format_age(static_cast<float>(now_ms - manual_override_since_ms_) / 1000.0f))),
                    theme::kWarn);
  }
}

}  // namespace adas::gui
