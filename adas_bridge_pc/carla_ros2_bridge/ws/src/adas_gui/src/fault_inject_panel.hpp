#ifndef ADAS_GUI__FAULT_INJECT_PANEL_HPP_
#define ADAS_GUI__FAULT_INJECT_PANEL_HPP_

#include <QString>
#include <QTextEdit>
#include <QWidget>

#include <functional>

#include "widgets.hpp"

namespace adas::gui {

// CAN v3 fault-injection console. Every command is confirmed and only one
// request may be in flight; RosBridge provides request-id/ack/timeout tracking.
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

  InjectCallback callback_;
  BusyButton* pending_button_{nullptr};
  QString pending_request_id_;
  QTextEdit* log_view_{nullptr};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__FAULT_INJECT_PANEL_HPP_
