#include "widgets.hpp"

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPropertyAnimation>
#include <QScreen>
#include <QSizePolicy>
#include <QWindow>

#include <algorithm>
#include <cmath>

#include "icons.hpp"
#include "theme.hpp"

namespace adas::gui {

namespace {

QFont default_font(int point_size, bool bold = false) {
  QFont f = QApplication::font();
  f.setPointSize(point_size);
  if (bold) f.setBold(true);
  return f;
}

// 与 IconLabel 字体一起计算 SVG 图标实际像素目标尺寸，按设备像素比渲染。
inline qreal device_pixel_ratio_for(const QWidget* w) {
  if (w && w->windowHandle()) return w->windowHandle()->devicePixelRatio();
  return qApp ? qApp->devicePixelRatio() : 1.0;
}

QColor led_color(LedState state) {
  switch (state) {
    case LedState::Ok:     return QColor(theme::kOk);
    case LedState::Warn:   return QColor(theme::kWarn);
    case LedState::Danger: return QColor(theme::kDanger);
    case LedState::Stale:  return QColor(theme::kStale);
  }
  return QColor(theme::kStale);
}

}  // namespace

// ============================ StatusBanner ====================================
StatusBanner::StatusBanner(QWidget* parent) : QLabel(parent) {
  setFixedHeight(54);
  setAlignment(Qt::AlignCenter);
  setTextInteractionFlags(Qt::TextSelectableByMouse);
}

void StatusBanner::setIconName(const QString& name) { icon_name_ = name; update(); }
void StatusBanner::setBannerColor(const QColor& color) { color_ = color; update(); }

void StatusBanner::setPulseStrength(qreal strength) {
  pulse_strength_ = std::clamp(strength, 0.0, 1.0);
  update();
}

void StatusBanner::pulse(const QColor& color) {
  pulse_color_ = color;
  auto* animation = new QPropertyAnimation(this, "pulseStrength", this);
  animation->setDuration(900);
  animation->setStartValue(0.0);
  animation->setKeyValueAt(0.18, 1.0);
  animation->setKeyValueAt(0.42, 0.18);
  animation->setKeyValueAt(0.68, 0.75);
  animation->setEndValue(0.0);
  animation->start(QAbstractAnimation::DeleteWhenStopped);
}

void StatusBanner::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const QRect r = rect().adjusted(1, 1, -2, -2);
  QColor accent = color_;
  if (accent == QColor(theme::kStale)) accent = QColor(theme::kMdOutline);
  QColor border = accent;
  border.setAlpha(150);
  p.setPen(QPen(border, 1.0));
  p.setBrush(QColor(theme::kMdSurfaceContainerHigh));
  p.drawRoundedRect(r, 8, 8);
  p.setPen(Qt::NoPen);
  p.setBrush(accent);
  p.drawRoundedRect(QRect(r.left(), r.top() + 7, 3, r.height() - 14), 1, 1);

  if (pulse_strength_ > 0.0) {
    QColor pulse = pulse_color_;
    pulse.setAlphaF(0.18 + pulse_strength_ * 0.55);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(pulse, 1.0 + pulse_strength_ * 2.0));
    p.drawRoundedRect(r.adjusted(2, 2, -2, -2), 6, 6);
  }

  const int icon_box = 28;
  const int icon_px = 18;
  QRect icon_rect(r.left() + 12, r.center().y() - icon_box / 2, icon_box, icon_box);
  const qreal dpr = device_pixel_ratio_for(this);
  const QPixmap ico = icons::pixmap(icon_name_, icon_px, dpr);
  if (!ico.isNull()) {
    p.drawPixmap(icon_rect, ico, ico.rect());
  }

  QFont text_font = default_font(13, true);
  p.setFont(text_font);
  p.setPen(QColor(theme::kTextPrimary));
  QRect text_rect = r.adjusted(icon_box + 20, 0, -10, 0);
  p.drawText(text_rect, Qt::AlignVCenter | Qt::AlignLeft, text());
}

QSize StatusBanner::sizeHint() const { return QSize(0, 54); }
QSize StatusBanner::minimumSizeHint() const { return QSize(0, 54); }

// ============================ LedIndicator ====================================
LedIndicator::LedIndicator(QWidget* parent) : QLabel(parent) {
  setMinimumHeight(20);
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
}

void LedIndicator::setState(LedState state) {
  if (state_ == state) return;
  state_ = state;
  update();
}

void LedIndicator::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  constexpr qreal kDotRadius = 3.5;
  const QPointF center(7.0, height() / 2.0);
  const QColor base = led_color(state_);

  QRadialGradient halo(center, 7.0);
  halo.setColorAt(0.0, QColor(base.red(), base.green(), base.blue(), 150));
  halo.setColorAt(0.6, QColor(base.red(), base.green(), base.blue(), 36));
  halo.setColorAt(1.0, QColor(base.red(), base.green(), base.blue(), 0));
  p.setPen(Qt::NoPen);
  p.setBrush(halo);
  p.drawEllipse(center, 7.0, 7.0);

  p.setPen(QPen(base.lighter(125), 1.0));
  p.setBrush(base);
  p.drawEllipse(center, kDotRadius, kDotRadius);

  // 右侧文本（如果有）
  if (!text().isEmpty()) {
    QRect r = rect().adjusted(17, 0, 0, 0);
    QFont f = default_font(std::max(8, QApplication::font().pointSize() - 1), true);
    p.setFont(f);
    QColor text_color = (state_ == LedState::Stale)
                            ? QColor(theme::kTextSecondary)
                            : QColor(theme::kTextPrimary);
    p.setPen(text_color);
    p.drawText(r, Qt::AlignVCenter | Qt::AlignLeft, text());
  }
}

QSize LedIndicator::sizeHint() const {
  const QFont f = default_font(std::max(8, QApplication::font().pointSize() - 1), true);
  const int text_width = text().isEmpty() ? 0 : QFontMetrics(f).horizontalAdvance(text()) + 5;
  return QSize(17 + text_width, 20);
}

// ============================ IconLabel ====================================
IconLabel::IconLabel(QWidget* parent) : QLabel(parent), icon_size_(14) {
  setMinimumHeight(18);
}
IconLabel::IconLabel(const QString& icon, const QString& text, QWidget* parent)
    : QLabel(text, parent), icon_name_(icon), icon_size_(14) {
  setMinimumHeight(18);
}

void IconLabel::setIconName(const QString& name) { icon_name_ = name; update(); }

void IconLabel::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  // 绘 SVG 图标（左）
  QRect icon_rect(2, 0, icon_size_ + 6, height());
  if (!icon_name_.isEmpty()) {
    const qreal dpr = device_pixel_ratio_for(this);
    const QPixmap ico = icons::pixmap(icon_name_, icon_size_, dpr);
    if (!ico.isNull()) {
      const QPoint pos(icon_rect.center().x() - int(ico.width() / dpr) / 2,
                       icon_rect.center().y() - int(ico.height() / dpr) / 2);
      p.drawPixmap(pos, ico);
    }
  }
  // 绘文本
  QRect text_rect = rect().adjusted(icon_size_ + 10, 0, -6, 0);
  QFont f = font();
  p.setFont(f);
  QColor c = palette().color(QPalette::WindowText);
  if (c.value() < 80) c = QColor("#c3c6cf");
  p.setPen(c);
  p.drawText(text_rect, Qt::AlignVCenter | Qt::AlignLeft, text());
}

QSize IconLabel::sizeHint() const {
  const QFontMetrics metrics(font());
  const int text_width = metrics.horizontalAdvance(text());
  return QSize(icon_size_ + 10 + text_width + 8,
               std::max(metrics.height() + 4, 18));
}

// ============================ StatusChip ====================================
StatusChip::StatusChip(QWidget* parent) : QFrame(parent) {
  setObjectName(QStringLiteral("StatusChip"));
  setMinimumHeight(34);
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(theme::chip_style("neutral"));
}

void StatusChip::setIconName(const QString& name) { icon_name_ = name; update(); }
void StatusChip::setLabel(const QString& label) { label_ = label; update(); }
void StatusChip::setValue(const QString& value) { value_ = value; update(); }
void StatusChip::setState(State state) {
  static const char* names[] = {"ok","warn","danger","stale","primary","neutral"};
  state_ = state;
  setStyleSheet(theme::chip_style(names[static_cast<int>(state)]));
  update();
}

void StatusChip::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const QRect r = rect().adjusted(2, 2, -2, -2);
  // 背景：MD3 chip 容器色（分级色低饱和填充 + 边）
  QColor base;
  QString state_str;
  switch (state_) {
    case State::Ok:     base = QColor("#2e7d32"); state_str = "ok"; break;
    case State::Warn:   base = QColor("#f9a825"); state_str = "warn"; break;
    case State::Danger: base = QColor("#c62828"); state_str = "danger"; break;
    case State::Stale:  base = QColor("#616161"); state_str = "stale"; break;
    case State::Primary: base = QColor("#00497c"); state_str = "primary"; break;
    case State::Neutral: base = QColor("#3a3f47"); state_str = "neutral"; break;
  }
  (void)base;

  // 容器使用 stylesheet 设置；这里只画图标 + 文本

  // 左侧 SVG 图标
  int icon_size = 14;
  QRect icon_rect(r.left() + 6, r.top() + (r.height() - icon_size) / 2,
                  icon_size + 4, icon_size);
  if (!icon_name_.isEmpty()) {
    const qreal dpr = device_pixel_ratio_for(this);
    const QPixmap ico = icons::pixmap(icon_name_, icon_size, dpr);
    if (!ico.isNull()) {
      const QPoint pos(icon_rect.center().x() - int(ico.width() / dpr) / 2,
                       icon_rect.center().y() - int(ico.height() / dpr) / 2);
      p.drawPixmap(pos, ico);
    }
  }
  // 标题 + 值
  QRect text_rect = r.adjusted(icon_size + 14, 0, -6, 0);
  QFont label_font = default_font(QApplication::font().pointSize(), true);
  QFont val_font = default_font(QApplication::font().pointSize(), false);
  // 上行 label（如 "位置/航向"），下行 value
  if (label_.isEmpty() || value_.isEmpty()) {
    // 单行模式
    p.setFont(label_font);
    QColor fg = (state_ == State::Danger) ? QColor("#ef9a9a") :
                (state_ == State::Ok)     ? QColor("#a5d6a7") :
                (state_ == State::Warn)   ? QColor("#ffe082") :
                                            QColor("#e4e6ea");
    p.setPen(fg);
    p.drawText(text_rect, Qt::AlignVCenter | Qt::AlignLeft, label_.isEmpty() ? value_ : label_);
  } else {
    // 双行：label 在左上小字，value 在右下大字
    QRectF label_rect = QRectF(text_rect).adjusted(0, 4, 0, -text_rect.height()/2);
    QRectF value_rect = QRectF(text_rect).adjusted(0, text_rect.height()/2 - 2, 0, -4);
    p.setFont(label_font);
    p.setPen(QColor("#9ea3ac"));
    p.drawText(label_rect, Qt::AlignVCenter | Qt::AlignLeft, label_);
    p.setFont(val_font);
    QColor fv = (state_ == State::Danger) ? QColor("#ef9a9a") :
                (state_ == State::Ok)     ? QColor("#a5d6a7") :
                (state_ == State::Warn)   ? QColor("#ffe082") :
                (state_ == State::Primary)? QColor("#a6c8ff") :
                                            QColor("#e4e6ea");
    p.setPen(fv);
    p.drawText(value_rect, Qt::AlignVCenter | Qt::AlignLeft, value_);
  }
}

QSize StatusChip::sizeHint() const { return QSize(160, 44); }
QSize StatusChip::minimumSizeHint() const { return QSize(120, 36); }

// ============================ SegmentedBar ====================================
SegmentedBar::SegmentedBar(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(22);
}

void SegmentedBar::setRange(int min, int max) { min_ = min; max_ = max; update(); }
void SegmentedBar::setValue(int value) { value_ = value; update(); }
void SegmentedBar::setColor(const QColor& color) { color_ = color; update(); }
void SegmentedBar::setIconName(const QString& name) { icon_name_ = name; update(); }
void SegmentedBar::setTextVisible(bool on) { text_visible_ = on; update(); }

void SegmentedBar::paintEvent(QPaintEvent*) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  const QRect r = rect().adjusted(0, 0, -1, -1);
  // 标签图标（可选）
  int icon_w = 0;
  if (!icon_name_.isEmpty()) {
    QRect icon_rect(r.left(), r.top(), 22, r.height());
    const qreal dpr = device_pixel_ratio_for(this);
    const QPixmap ico = icons::pixmap(icon_name_, 14, dpr);
    if (!ico.isNull()) {
      const QPoint pos(icon_rect.center().x() - int(ico.width() / dpr) / 2,
                       icon_rect.center().y() - int(ico.height() / dpr) / 2);
      p.drawPixmap(pos, ico);
    }
    icon_w = 22;
  }
  QRect bar_rect = r.adjusted(icon_w, 0, 0, 0);
  // track
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(theme::kMdSurfaceContainerLow));
  p.drawRoundedRect(bar_rect, 5, 5);
  // chunk
  const int span = max_ - min_;
  if (span > 0 && value_ > min_) {
    const double ratio = static_cast<double>(value_ - min_) / span;
    int chunk_w = static_cast<int>(bar_rect.width() * ratio);
    if (chunk_w < 6) chunk_w = 6;
    p.setBrush(color_);
    p.drawRoundedRect(QRect(bar_rect.left(), bar_rect.top(), chunk_w, bar_rect.height()), 5, 5);
  }
  // 中央 percentage 文本
  if (text_visible_) {
    QFont f = default_font(QApplication::font().pointSize(), true);
    p.setFont(f);
    p.setPen(QColor("#e4e6ea"));
    p.drawText(bar_rect, Qt::AlignCenter, QString::number(value_) + "%");
  }
}

QSize SegmentedBar::sizeHint() const { return QSize(120, 22); }

}  // namespace adas::gui
