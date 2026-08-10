#include "fault_inject_panel.hpp"

#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QScrollArea>
#include <QSizePolicy>
#include <QVBoxLayout>

#include "request_tracker.hpp"
#include "theme.hpp"

namespace adas::gui {

FaultInjectPanel::FaultInjectPanel(InjectCallback callback, QWidget* parent)
    : QWidget(parent), callback_(std::move(callback)) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto* title_box = new QGroupBox(QStringLiteral("故障注入（CAN v3 / 0x301）"), this);
  auto* title_layout = new QVBoxLayout(title_box);
  auto* title = new QLabel(QStringLiteral(
      "命令经 PC CAN 接口下发；仅 ADAS_TEST_BUILD=1 的 MCU 接受。"
      "每次操作均需确认，并等待 bridge 传输确认。"), title_box);
  title->setWordWrap(true);
  title->setStyleSheet(QStringLiteral("color:%1;font-size:10px;")
                           .arg(theme::kTextSecondary));
  title_layout->addWidget(title);
  root->addWidget(title_box);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* content = new QWidget(scroll);
  auto* content_layout = new QVBoxLayout(content);
  content_layout->setContentsMargins(0, 0, 0, 0);
  auto* grid = new QGridLayout();
  grid->setSpacing(4);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);

  // Values are defined by adas_mcu/include/adas_can_protocol.h. Keep this
  // table explicit so the GUI cannot silently drift back to the legacy map.
  grid->addWidget(makeButton(QStringLiteral("清除故障"),
                             QStringLiteral("INJ_CMD_CLEAR = 0"), 0,
                             ConfirmSeverity::Warning), 0, 0);
  grid->addWidget(makeButton(QStringLiteral("强制降级"),
                             QStringLiteral("INJ_CMD_FORCE_DEGRADE = 1"), 1,
                             ConfirmSeverity::Warning), 0, 1);
  grid->addWidget(makeButton(QStringLiteral("强制急停"),
                             QStringLiteral("INJ_CMD_FORCE_ESTOP = 2"), 2,
                             ConfirmSeverity::Critical), 1, 0);
  grid->addWidget(makeButton(QStringLiteral("主源掉线"),
                             QStringLiteral("INJ_CMD_DROP_PRIMARY = 3"), 3,
                             ConfirmSeverity::Danger), 1, 1);
  grid->addWidget(makeButton(QStringLiteral("备源掉线"),
                             QStringLiteral("INJ_CMD_DROP_BACKUP = 4"), 4,
                             ConfirmSeverity::Danger), 2, 0);
  grid->addWidget(makeButton(QStringLiteral("强制锁定"),
                             QStringLiteral("INJ_CMD_FORCE_LOCK = 5"), 5,
                             ConfirmSeverity::Critical), 2, 1);
  grid->addWidget(makeButton(QStringLiteral("双源掉线"),
                             QStringLiteral("INJ_CMD_DROP_ALL = 6"), 6,
                             ConfirmSeverity::Critical), 3, 0);
  grid->addWidget(makeButton(QStringLiteral("强制 MRM"),
                             QStringLiteral("INJ_CMD_FORCE_MRM = 7"), 7,
                             ConfirmSeverity::Critical), 3, 1);
  grid->addWidget(makeButton(QStringLiteral("CAN 耗尽"),
                             QStringLiteral("INJ_CMD_CAN_EXHAUSTED = 8"), 8,
                             ConfirmSeverity::Danger), 4, 0);
  grid->addWidget(makeButton(QStringLiteral("自检失败"),
                             QStringLiteral("INJ_CMD_SELF_TEST_FAIL = 9"), 9,
                             ConfirmSeverity::Critical), 4, 1);
  content_layout->addLayout(grid);
  scroll->setWidget(content);
  root->addWidget(scroll, 1);

  log_view_ = new QTextEdit(this);
  log_view_->setReadOnly(true);
  log_view_->setMinimumHeight(80);
  log_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  log_view_->setStyleSheet(QStringLiteral(
      "QTextEdit{background:%1;color:%2;border:1px solid %3;border-radius:6px;"
      "font-family:'Courier New',monospace;font-size:10px;}")
      .arg(theme::kMdSurfaceContainerHigh, theme::kMdOnSurface,
           theme::kCardBorder));
  root->addWidget(log_view_, 1);
  logEvent(QStringLiteral("面板就绪 — 等待 bridge/CAN ack"));
}

BusyButton* FaultInjectPanel::makeButton(const QString& text,
                                         const QString& tooltip, int command,
                                         ConfirmSeverity severity) {
  auto* button = new BusyButton(text, this);
  button->setToolTip(tooltip);
  button->setMinimumHeight(34);
  button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  // Phase 2 hardening：注册到 all_buttons_,请求在飞行时统一禁用/启用。
  all_buttons_.append(button);
  connect(button, &QPushButton::clicked, this,
          [this, button, command, text, severity]() {
            requestInjection(button, command, text, severity);
          });
  return button;
}

void FaultInjectPanel::requestInjection(BusyButton* button, int command,
                                        const QString& label,
                                        ConfirmSeverity severity) {
  if (pending_button_ != nullptr) {
    // 已经有请求在飞行：禁用其他按钮,而不是静默丢弃第二次点击。
    logEvent(QStringLiteral(
        "! 拒绝重复点击：上一条 cmd=%1（request_id=%2）尚未收到 ack,等待响应中")
                 .arg(button == pending_button_ ? command : -1)
                 .arg(pending_request_id_));
    return;
  }
  const QString action = QStringLiteral("下发 %1（命令 %2）").arg(label).arg(command);
  const QString impact = command == 0
      ? QStringLiteral("将清除 MCU 可恢复故障状态。")
      : QStringLiteral("可能立即改变 MCU 安全状态或车辆控制输出，仅应在受控测试环境执行。");
  if (!confirm_action(this, QStringLiteral("确认故障注入"), action, impact,
                      severity)) {
    logEvent(QStringLiteral("× 已取消：%1").arg(label));
    return;
  }

  button->setBusy(true, QStringLiteral("等待确认"),
                  QStringLiteral("请求处理中，收到 ack 或超时后可重试"));
  pending_button_ = button;
  // Phase 2 hardening：禁用其他 9 个按钮,而不是允许它们堆积 silent-drop。
  setAllButtonsEnabled(false);
  // busy=true 的按钮自身也要禁用,避免用户重复点击同一按钮造成 race。
  button->setEnabled(false);
  const QString request_id = callback_ ? callback_(command, 0, label) : QString();
  if (request_id.isEmpty()) {
    button->setBusy(false);
    button->setEnabled(true);
    pending_button_ = nullptr;
    setAllButtonsEnabled(true);
    logEvent(QStringLiteral("! 请求未发送：已有请求进行中或接口不可用"));
    return;
  }
  pending_request_id_ = request_id;
  logEvent(QStringLiteral("→ 已发送 cmd=%1 request_id=%2").arg(command).arg(request_id));
}

void FaultInjectPanel::onRequestChanged(const QString& request_id, int state,
                                        const QString& detail) {
  if (state == static_cast<int>(RequestState::Sent)) return;
  if (request_id != pending_request_id_) return;
  const bool ok = state == static_cast<int>(RequestState::Acknowledged);
  logEvent(QStringLiteral("%1 %2 — %3")
               .arg(ok ? QStringLiteral("✓ 已确认") : QStringLiteral("! 失败"),
                    request_id, detail));
  if (pending_button_) {
    pending_button_->setBusy(false);
    pending_button_->setEnabled(true);
  }
  pending_button_ = nullptr;
  pending_request_id_.clear();
  setAllButtonsEnabled(true);
}

void FaultInjectPanel::setAllButtonsEnabled(bool enabled) {
  // pending_button_ 自身保持 disabled(busy 可视化),其余按钮统一启用/禁用。
  for (auto* btn : all_buttons_) {
    if (btn == pending_button_) continue;
    btn->setEnabled(enabled);
  }
}

void FaultInjectPanel::logEvent(const QString& line) {
  const auto timestamp =
      QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));
  log_view_->append(QStringLiteral("[%1] %2").arg(timestamp, line));
}

}  // namespace adas::gui
