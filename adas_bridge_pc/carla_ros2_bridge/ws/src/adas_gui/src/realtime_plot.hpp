#ifndef ADAS_GUI__REALTIME_PLOT_HPP_
#define ADAS_GUI__REALTIME_PLOT_HPP_

#include <QColor>
#include <QElapsedTimer>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

namespace adas::gui {

// 无 Qt Charts 依赖的实时曲线。只接受真实订阅回调数据，保留最近 window_s 秒，
// 断流时停止追加而不是补合成点。
class RealtimePlot : public QWidget {
  Q_OBJECT
 public:
  explicit RealtimePlot(QString title, QString unit, QColor color,
                        QWidget* parent = nullptr);

  void setRange(double minimum, double maximum);
  void setWindowSeconds(double seconds);
  void appendValue(double value);
  void clear();
  int sampleCount() const { return samples_.size(); }

 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;

 private:
  void prune(double now_s);

  QString title_;
  QString unit_;
  QColor color_;
  QVector<QPointF> samples_;
  QElapsedTimer clock_;
  double minimum_{-1.0};
  double maximum_{1.0};
  double window_s_{30.0};
};

}  // namespace adas::gui

#endif  // ADAS_GUI__REALTIME_PLOT_HPP_
