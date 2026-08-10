#include "realtime_plot.hpp"

#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>
#include <utility>

#include "theme.hpp"

namespace adas::gui {

RealtimePlot::RealtimePlot(QString title, QString unit, QColor color, QWidget* parent)
    : QWidget(parent), title_(std::move(title)), unit_(std::move(unit)), color_(color) {
  setMinimumHeight(82);
  setToolTip(QStringLiteral("仅显示真实订阅数据；灰色遮罩表示输入断流"));
  clock_.start();
  freshness_timer_.setInterval(500);
  connect(&freshness_timer_, &QTimer::timeout, this, [this]() {
    if (!samples_.isEmpty()) update();
  });
  freshness_timer_.start();
}

void RealtimePlot::setRange(double minimum, double maximum) {
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum >= maximum) return;
  minimum_ = minimum;
  maximum_ = maximum;
  update();
}

void RealtimePlot::setWindowSeconds(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 1.0) return;
  window_s_ = seconds;
  prune(clock_.elapsed() / 1000.0);
  update();
}

void RealtimePlot::setStaleAfterSeconds(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 0.0) return;
  stale_after_s_ = seconds;
  update();
}

bool RealtimePlot::dataStale() const {
  if (samples_.isEmpty()) return true;
  return clock_.elapsed() / 1000.0 - samples_.constLast().x() > stale_after_s_;
}

void RealtimePlot::appendValue(double value) {
  if (!std::isfinite(value)) return;
  const double now_s = clock_.elapsed() / 1000.0;
  samples_.append(QPointF(now_s, value));
  prune(now_s);
  update();
}

void RealtimePlot::clear() {
  samples_.clear();
  clock_.restart();
  update();
}

void RealtimePlot::prune(double now_s) {
  const double cutoff = now_s - window_s_;
  int first = 0;
  while (first < samples_.size() && samples_[first].x() < cutoff) ++first;
  if (first > 0) samples_.remove(0, first);
  constexpr int kMaxSamples = 1200;
  if (samples_.size() > kMaxSamples) samples_.remove(0, samples_.size() - kMaxSamples);
}

void RealtimePlot::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(theme::kMdSurfaceContainerLow));

  const QRectF plot = QRectF(rect()).adjusted(34.0, 20.0, -8.0, -18.0);
  painter.setPen(QPen(QColor(theme::kCardBorder), 1.0));
  painter.drawRoundedRect(plot, 6.0, 6.0);
  painter.setPen(QPen(QColor(theme::kCardBorder), 1.0, Qt::DashLine));
  painter.drawLine(QPointF(plot.left(), plot.center().y()),
                   QPointF(plot.right(), plot.center().y()));

  painter.setPen(QColor(theme::kTextSecondary));
  QFont small = font();
  small.setPointSize(std::max(7, small.pointSize() - 2));
  painter.setFont(small);
  const QRectF title_rect(4, 2, width() * 0.62, 16);
  painter.drawText(title_rect, Qt::AlignLeft | Qt::AlignVCenter,
                   title_ + (samples_.isEmpty() ? QStringLiteral(" · 等待数据") : QString()));
  if (!samples_.isEmpty()) {
    QFont value_font = small;
    value_font.setBold(true);
    painter.setFont(value_font);
    painter.setPen(color_);
    const double current = samples_.constLast().y();
    const int precision = std::abs(current) < 10.0 ? 2 : 1;
    const QString current_text = QStringLiteral("%1 %2")
        .arg(current, 0, 'f', precision)
        .arg(unit_);
    painter.drawText(QRectF(width() * 0.55, 2, width() * 0.45 - 8, 16),
                     Qt::AlignRight | Qt::AlignVCenter, current_text);
    painter.setFont(small);
    painter.setPen(QColor(theme::kTextSecondary));
  }
  painter.drawText(QRectF(2, plot.top() - 3, 30, 14), Qt::AlignRight | Qt::AlignTop,
                   QString::number(maximum_, 'f', 1));
  painter.drawText(QRectF(2, plot.bottom() - 11, 30, 14), Qt::AlignRight | Qt::AlignTop,
                   QString::number(minimum_, 'f', 1));
  painter.drawText(QRectF(plot.left(), plot.bottom() + 1, plot.width(), 15),
                   Qt::AlignRight | Qt::AlignVCenter,
                   QStringLiteral("最近 %1 s  [%2]").arg(window_s_, 0, 'f', 0).arg(unit_));
  if (samples_.isEmpty()) return;

  const double now_s = clock_.elapsed() / 1000.0;
  const double start_s = now_s - window_s_;
  const double span = maximum_ - minimum_;
  QPainterPath path;
  bool started = false;
  for (const auto& sample : samples_) {
    const double x = plot.left() + (sample.x() - start_s) / window_s_ * plot.width();
    const double clamped = std::clamp(sample.y(), minimum_, maximum_);
    const double y = plot.bottom() - (clamped - minimum_) / span * plot.height();
    if (!started) {
      path.moveTo(x, y);
      started = true;
    } else {
      path.lineTo(x, y);
    }
  }
  painter.save();
  painter.setClipRect(plot);
  painter.setPen(QPen(color_, 2.0));
  painter.drawPath(path);
  const QPointF last = path.currentPosition();
  painter.setBrush(color_);
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(last, 3.0, 3.0);
  painter.restore();

  if (dataStale()) {
    painter.fillRect(plot, QColor(20, 24, 28, 150));
    painter.setPen(QColor(theme::kWarn));
    QFont stale_font = small;
    stale_font.setBold(true);
    painter.setFont(stale_font);
    painter.drawText(plot, Qt::AlignCenter,
                     QStringLiteral("数据断流 · %1 s")
                         .arg(clock_.elapsed() / 1000.0 - samples_.constLast().x(),
                              0, 'f', 1));
  }
}

QSize RealtimePlot::sizeHint() const { return QSize(260, 96); }

}  // namespace adas::gui
