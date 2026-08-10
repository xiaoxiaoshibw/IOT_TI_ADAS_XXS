#ifndef ADAS_GUI__WIDGETS_HPP_
#define ADAS_GUI__WIDGETS_HPP_

// MD3 风格自定义控件集。继承既有基类（QLabel/QFrame），不改外部 API：
//   · StatusBanner : QLabel   — 顶部状态横幅，左侧 FA 图标 + 渐变填充
//   · LedIndicator : QLabel   — 状态指示灯，自绘 radial glow + 位图渐变
//   · IconLabel    : QLabel   — FA 图标 + 文本水平排列（可作 QGroupBox 标题替身）
//   · StatusChip   : QFrame   — 状态 chip：uint icon + 标题 + 值，含分级色
//   · SegmentedBar : QWidget  — 油门/制动分段进度条（自绘 round chunk + track）
//
// 设计目标：复用 `main_window.hpp` / `launch_panel.hpp` 中已声明的 QLabel*
// 成员，子类化 QLabel 即可直接替换实例而不改头文件 connect 链。
// 这些控件不持有任何业务态：纯渲染/输入回显；状态染色仍由 MainWindow 用
// theme::value_style() / theme::chip_style() 触发。

#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QStringView>
#include <QTimer>

namespace adas::gui {

// 状态分级枚举（与 theme::kOk/kWarn/kDanger/kStale 语义对应）。
enum class LedState { Ok, Warn, Danger, Stale };

enum class ConfirmSeverity { Normal, Warning, Danger, Critical };

// Safety-sensitive operations use one dialog contract: cancel is always the
// default and pressing Escape never confirms the operation.
bool confirm_action(QWidget* parent, const QString& title,
                    const QString& action, const QString& impact,
                    ConfirmSeverity severity = ConfirmSeverity::Warning);

// Push button with an immediate, recoverable busy state. Business code still
// owns the backend guard; this class only makes the lock visible and prevents
// a second UI click from entering the slot.
class BusyButton : public QPushButton {
  Q_OBJECT
 public:
  explicit BusyButton(QWidget* parent = nullptr);
  explicit BusyButton(const QString& text, QWidget* parent = nullptr);

  bool isBusy() const { return busy_; }
  void setBusy(bool busy, const QString& text = QString(),
               const QString& reason = QString());

 private:
  void initialise();
  void updateBusyText();

  QTimer animation_timer_;
  QString idle_text_;
  QString busy_text_;
  QString idle_tooltip_;
  int animation_frame_{0};
  bool busy_{false};
};

class StatusBanner : public QLabel {
  Q_OBJECT
  Q_PROPERTY(qreal pulseStrength READ pulseStrength WRITE setPulseStrength)
 public:
  explicit StatusBanner(QWidget* parent = nullptr);
  // 直接 setText 仍兼容旧调用；额外提供 setIcon / setBannerColor 显式美化。
  void setIconName(const QString& name);
  void setBannerColor(const QColor& color);
  void pulse(const QColor& color);
  qreal pulseStrength() const { return pulse_strength_; }
  void setPulseStrength(qreal strength);

 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 private:
  QString icon_name_{QStringLiteral("flag")};
  QColor color_{Qt::gray};
  QColor pulse_color_{Qt::white};
  qreal pulse_strength_{0.0};
};

class LedIndicator : public QLabel {
  Q_OBJECT
 public:
  explicit LedIndicator(QWidget* parent = nullptr);
  // 兼容旧 setText 调用方式：原代码 make_dot 文本格式是 "● MCU"
  // 新实现里 setText 仍然会被绘制（在右侧）；以及 setState 只改 LED 颜色。
  void setState(LedState state);
  LedState state() const { return state_; }

 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;

 private:
  LedState state_{LedState::Stale};
};

class IconLabel : public QLabel {
  Q_OBJECT
 public:
  explicit IconLabel(QWidget* parent = nullptr);
  IconLabel(const QString& icon_name, const QString& text, QWidget* parent = nullptr);
  // 设置 FA 图标名（fa::icon 字典里查）与可选文本。
  void setIconName(const QString& name);

 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;

 private:
  QString icon_name_;
  int icon_size_{14};
};

class StatusChip : public QFrame {
  Q_OBJECT
 public:
  enum class State { Ok, Warn, Danger, Stale, Primary, Neutral };
  explicit StatusChip(QWidget* parent = nullptr);
  // 设置图标 / 标题 / 值 / 分级状态。
  void setIconName(const QString& name);
  void setLabel(const QString& label);
  void setValue(const QString& value);
  void setState(State state);

 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 private:
  QString icon_name_;
  QString label_;
  QString value_;
  State state_{State::Neutral};
};

// 分段进度条：左侧 FA 图标 + accent 色 chunk，右侧 percentage 文本。
// 兼容 setRange/setValue，以替代 QProgressBar，但 chunk 颜色由 setColor 控制。
class SegmentedBar : public QWidget {
  Q_OBJECT
 public:
  explicit SegmentedBar(QWidget* parent = nullptr);
  void setRange(int min, int max);
  void setValue(int value);
  void setColor(const QColor& color);
  void setIconName(const QString& name);
  void setTextVisible(bool on);

  // 兼容外侧使用的 QProgressBar 句柄（无成员多余依赖；用 setTextVisible 即可）
 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;

 private:
  int min_{0}, max_{100}, value_{0};
  QColor color_{QColor("#66bb6a")};
  QString icon_name_;
  bool text_visible_{true};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__WIDGETS_HPP_
