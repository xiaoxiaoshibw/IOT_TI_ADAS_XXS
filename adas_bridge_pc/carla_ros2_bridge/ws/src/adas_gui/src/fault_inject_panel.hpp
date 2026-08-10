#ifndef ADAS_GUI__FAULT_INJECT_PANEL_HPP_
#define ADAS_GUI__FAULT_INJECT_PANEL_HPP_

#include <QList>
#include <QString>
#include <QTextEdit>
#include <QWidget>

#include <functional>

#include "widgets.hpp"

namespace adas::gui {

// CAN v3 fault-injection console. Every command is confirmed and only one
// request may be in flight; RosBridge provides request-id/ack/timeout tracking.
//
// Robustness contract (Phase 2 hardening):
//   · 当 pending_request_id_ 非空时,所有按钮(除当前 pending_button_
//     自身)必须被 setEnabled(false),而不是静默丢弃点击。
//   · pending_button_ 自身保持 busy=true,可视化提示用户当前在等待 ack。
//   · ack 到达 / 超时 / ROS bridge 不可用 都会强制 setAllButtonsEnabled(true)。
class FaultInjectPanel : public QWidget {
  Q_OBJECT

 public:
  using InjectCallback =
      std::function<QString(int cmd, int param, const QString& label)>;

  explicit FaultInjectPanel(InjectCallback callback, QWidget* parent = nullptr);
  ~FaultInjectPanel() override = default;

 public slots:
  void onRequestChanged(const QString& request_id, int state,
                        const QString& detail);

 private:
  BusyButton* makeButton(const QString& text, const QString& tooltip,
                         int command, ConfirmSeverity severity);
  void requestInjection(BusyButton* button, int command, const QString& label,
                        ConfirmSeverity severity);
  void logEvent(const QString& line);
  // 单 in-flight 飞行期间禁用其他按钮;pending_button_ 自身保持 busy。
  void setAllButtonsEnabled(bool enabled);

  InjectCallback callback_;
  BusyButton* pending_button_{nullptr};
  QString pending_request_id_;
  QList<BusyButton*> all_buttons_;
  QTextEdit* log_view_{nullptr};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__FAULT_INJECT_PANEL_HPP_
