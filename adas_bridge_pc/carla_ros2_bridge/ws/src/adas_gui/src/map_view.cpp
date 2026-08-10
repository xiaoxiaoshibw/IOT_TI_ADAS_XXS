#include "map_view.hpp"

#include <QApplication>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>

#include "lane_hit.hpp"
#include "theme.hpp"

namespace adas::gui {
namespace {

constexpr double kClickTolerancePx = 5.0;
constexpr int kTrailMaxPoints = 600;   // 10 Hz × 60 s

QPointF to_screen(const MapCamera& camera, const QPointF& world) {
  const auto point = camera.world_to_screen(world.x(), world.y());
  return {point.x, point.y};
}

QPolygonF polyline_to_screen(const MapCamera& camera, const QPolygonF& world) {
  QPolygonF result;
  result.reserve(world.size());
  for (const auto& point : world) result.append(to_screen(camera, point));
  return result;
}

}  // namespace

MapView::MapView(QWidget* parent) : QWidget(parent) {
  // Q_DECLARE_METATYPE / qRegisterMetaType for GuiLane 集中在 ros_bridge.cpp
  // 统一负责；map_view.cpp 不重复注册。
  setMinimumSize(360, 360);
  setMouseTracking(true);   // 悬停吸附预览需要
  setCursor(Qt::CrossCursor);
  click_timer_.setSingleShot(true);
  click_timer_.setInterval(QApplication::doubleClickInterval());
  connect(&click_timer_, &QTimer::timeout, this, [this]() {
    emit goalRequested(pending_goal_x_, pending_goal_y_);
  });
}

void MapView::setMapMetadata(const QString& map_id, const QString& map_hash) {
  const bool changed = (!map_id_.isEmpty() && map_id_ != map_id) ||
                       (!map_hash_.isEmpty() && map_hash_ != map_hash);
  if (changed) {
    click_timer_.stop();
    lanes_.clear();
    route_.clear();
    trail_.clear();
    objects_.clear();
    goal_valid_ = false;
    vehicle_valid_ = false;
    hover_valid_ = false;
    fitted_ = false;
  }
  map_id_ = map_id;
  map_hash_ = map_hash;
  update();
}

void MapView::setLanes(const QVector<GuiLane>& lanes, const QString& map_id) {
  if (!map_id_.isEmpty() && map_id_ != map_id) {
    click_timer_.stop();
    route_.clear();
    trail_.clear();
    objects_.clear();
    goal_valid_ = false;
    hover_valid_ = false;
    fitted_ = false;
  }
  lanes_ = lanes;
  map_id_ = map_id;
  if (!fitted_) fitToLanes();
  update();
}

void MapView::setRoute(const QPolygonF& route) {
  route_ = route;
  update();
}

void MapView::setVehicle(double x, double y, double yaw_rad, bool valid) {
  vehicle_x_ = x;
  vehicle_y_ = y;
  vehicle_yaw_ = yaw_rad;
  vehicle_valid_ = valid;
  if (valid) {
    trail_.append(QPointF(x, y));
    if (trail_.size() > kTrailMaxPoints) trail_.remove(0, trail_.size() - kTrailMaxPoints);
    if (follow_) camera_.center_on(x, y);
  }
  update();
}

void MapView::setObjects(const QVector<GuiMapObject>& objects) {
  objects_ = objects.mid(0, 64);
  update();
}

void MapView::setGoal(double x, double y, bool valid) {
  goal_x_ = x;
  goal_y_ = y;
  goal_valid_ = valid;
  update();
}

void MapView::setGoalSelectionEnabled(bool enabled) {
  goal_selection_enabled_ = enabled;
  if (!enabled) click_timer_.stop();
  setCursor(enabled ? Qt::CrossCursor : Qt::ForbiddenCursor);
  setToolTip(enabled ? QString() : QStringLiteral("导航请求处理中，暂不能设置新目标"));
}

void MapView::setFollowEnabled(bool enabled) {
  follow_ = enabled;
  if (follow_ && vehicle_valid_) camera_.center_on(vehicle_x_, vehicle_y_);
  update();
}

void MapView::setLayers(quint32 flags) {
  if (layer_flags_ == flags) return;
  layer_flags_ = flags;
  update();
}

void MapView::clearTrail() {
  trail_.clear();
  update();
}

void MapView::fitToLanes() {
  if (lanes_.isEmpty()) return;
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto& lane : lanes_) {
    for (const auto& point : lane.centerline) {
      min_x = std::min(min_x, point.x());
      min_y = std::min(min_y, point.y());
      max_x = std::max(max_x, point.x());
      max_y = std::max(max_y, point.y());
    }
  }
  camera_.set_viewport(width(), height());
  camera_.fit(min_x, min_y, max_x, max_y);
  fitted_ = true;
  update();
}

void MapView::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  const QRectF surface = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath surface_path;
  surface_path.addRoundedRect(surface, 7.0, 7.0);
  painter.fillPath(surface_path, QColor(theme::kMapBg));
  painter.setPen(QPen(QColor(theme::kCardBorder), 1.0));
  painter.drawPath(surface_path);
  painter.setClipPath(surface_path);

  const double scale = camera_.scale();
  const double grid_m = (scale > 12.0) ? 5.0 : (scale > 4.0 ? 10.0 : 50.0);
  const double grid_px = grid_m * scale;
  if ((layer_flags_ & kLayerGrid) && grid_px >= 12.0) {
    const QPointF origin = to_screen(camera_, {0.0, 0.0});
    painter.setPen(QPen(QColor(255, 255, 255, 14), 1.0));
    for (double gx = std::fmod(origin.x(), grid_px); gx < width(); gx += grid_px) {
      painter.drawLine(QPointF(gx, 0), QPointF(gx, height()));
    }
    for (double gy = std::fmod(origin.y(), grid_px); gy < height(); gy += grid_px) {
      painter.drawLine(QPointF(0, gy), QPointF(width(), gy));
    }
  }

  camera_.set_viewport(width(), height());

  if (lanes_.isEmpty()) {
    const QRect center = rect().adjusted(24, 0, -24, 0);
    QFont title = painter.font();
    title.setPointSize(12);
    title.setBold(true);
    painter.setFont(title);
    painter.setPen(QColor(theme::kTextPrimary));
    painter.drawText(center.adjusted(0, -20, 0, 0), Qt::AlignCenter,
                     QStringLiteral("等待地图数据"));
    QFont detail = painter.font();
    detail.setPointSize(9);
    detail.setBold(false);
    painter.setFont(detail);
    painter.setPen(QColor(theme::kTextSecondary));
    painter.drawText(center.adjusted(0, 22, 0, 0), Qt::AlignCenter,
                     QStringLiteral("/adas/map/lane_graph"));
    return;
  }

  // 车道中心线：普通灰实线、路口暗黄虚线
  for (const auto& lane : lanes_) {
    if (lane.junction) {
      painter.setPen(QPen(QColor("#a8801f"), 1.4, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin));
    } else {
      painter.setPen(QPen(QColor(theme::kLaneNormal), 1.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    }
    painter.drawPolyline(polyline_to_screen(camera_, lane.centerline));

    // LaneGraph centerline order is the legal driving direction.  A compact
    // arrow makes adjacent opposite-direction lanes unambiguous for goal
    // selection without turning the GUI into a duplicate CARLA scene.
    if (lane.centerline.size() >= 2) {
      const int segment = std::max<int>(0, static_cast<int>(lane.centerline.size() / 2) - 1);
      const QPointF a = to_screen(camera_, lane.centerline[segment]);
      const QPointF b = to_screen(camera_, lane.centerline[segment + 1]);
      const double length = std::hypot(b.x() - a.x(), b.y() - a.y());
      if (length > 5.0) {
        const QPointF direction((b.x() - a.x()) / length,
                                (b.y() - a.y()) / length);
        const QPointF normal(-direction.y(), direction.x());
        const QPointF tip = (a + b) * 0.5;
        const QPointF arrow[3] = {
            tip,
            tip - direction * 7.0 + normal * 3.5,
            tip - direction * 7.0 - normal * 3.5};
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(lane.junction ? "#d6ad43" : "#7892a8"));
        painter.drawPolygon(arrow, 3);
      }
    }
  }

  // 行驶尾迹：渐变 alpha 暗绿（越早越淡）；下方铺在路线之下
  if ((layer_flags_ & kLayerTrail) && trail_.size() >= 2) {
    const QPolygonF trail = polyline_to_screen(camera_, trail_);
    const int n = trail.size();
    for (int i = 1; i < n; ++i) {
      const double t = static_cast<double>(i) / n;   // 0=最早 → 1=最新
      const int alpha = static_cast<int>(60 + 160 * t);
      QColor c(theme::kEgoTrail);
      c.setAlpha(alpha);
      painter.setPen(QPen(c, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      painter.drawLine(trail[i - 1], trail[i]);
    }
  }

  // 规划路线：MD3 primary 蓝粗线 + 外侧淡蓝光晕
  if (route_.size() >= 2) {
    const QPolygonF route = polyline_to_screen(camera_, route_);
    painter.setPen(QPen(QColor(166, 200, 255, 70), 7.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPolyline(route);
    painter.setPen(QPen(QColor(theme::kAccent), 3.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPolyline(route);
  }

  // 悬停吸附预览：dashed primary 圆 + 8m 半径淡 ring（调试图层，默认关）
  if ((layer_flags_ & kLayerSnapRadius) && hover_valid_ && !dragging_) {
    const QPointF snap = to_screen(camera_, {hover_x_, hover_y_});
    // 8m 半径 ring
    const double snap_px = kSnapMaxDistanceM * camera_.scale();
    painter.setPen(QPen(QColor(theme::kSnapPreview), 1.0, Qt::DotLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(snap, snap_px, snap_px);
    // 中心点
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(theme::kSnapPreview));
    painter.drawEllipse(snap, 4.0, 4.0);
  }

  // 目标点：MD3 goal pin（自绘 path，不依赖字体）
  if (goal_valid_) {
    const QPointF goal = to_screen(camera_, {goal_x_, goal_y_});
    QPainterPath pin;
    pin.moveTo(goal.x(), goal.y() + 14);
    pin.cubicTo(goal.x() - 8, goal.y() + 6,
                goal.x() - 8, goal.y() - 4,
                goal.x(), goal.y() - 4);
    pin.cubicTo(goal.x() + 8, goal.y() - 4,
                goal.x() + 8, goal.y() + 6,
                goal.x(), goal.y() + 14);
    painter.setPen(QPen(QColor("#7a1116"), 1.5));
    painter.setBrush(QColor(theme::kGoal));
    painter.drawPath(pin);
    painter.setBrush(Qt::white);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(goal, 2.5, 2.5);
  }

  // 完整目标物：按消息中的稳定 ID 绘制，图层默认关闭。车辆/卡车画有朝向的
  // 矩形，行人画圆，避免 20~64 个目标时创建 QWidget 子项阻塞 UI。
  if (layer_flags_ & kLayerObjects) {
    for (const auto& object : objects_) {
      const QPointF center = to_screen(camera_, {object.x, object.y});
      painter.save();
      painter.translate(center);
      painter.rotate(-object.yaw_rad * 180.0 / M_PI);
      QColor color = object.classification == 3 ? QColor("#ffb74d")
                     : object.classification == 2 ? QColor("#ce93d8")
                                                  : QColor("#64b5f6");
      painter.setPen(QPen(color.darker(160), 1.2));
      painter.setBrush(color);
      if (object.classification == 3) {
        painter.drawEllipse(QPointF(0, 0), 4.0, 4.0);
      } else {
        const double length_px = std::clamp(object.length_m * scale, 7.0, 22.0);
        const double width_px = std::clamp(object.width_m * scale, 4.0, 12.0);
        painter.drawRoundedRect(QRectF(-length_px / 2.0, -width_px / 2.0,
                                      length_px, width_px), 2.0, 2.0);
        painter.drawLine(QPointF(length_px / 2.0, 0),
                         QPointF(length_px / 2.0 + 3.0, 0));
      }
      painter.restore();
    }
  }

  // 自车三角 + dropshadow + 朝向描边（屏幕 y 翻转）
  if (vehicle_valid_) {
    const QPointF center = to_screen(camera_, {vehicle_x_, vehicle_y_});
    // 光晕（默认关，演示时不抢眼）
    if (layer_flags_ & kLayerEgoHalo) {
      QRadialGradient halo(center, 22);
      halo.setColorAt(0.0, QColor(102, 187, 106, 110));
      halo.setColorAt(1.0, QColor(102, 187, 106, 0));
      painter.setPen(Qt::NoPen);
      painter.setBrush(halo);
      painter.drawEllipse(center, 22, 22);
    }
    // 三角本体
    painter.save();
    painter.translate(center);
    painter.rotate(-vehicle_yaw_ * 180.0 / M_PI);
    painter.setPen(QPen(QColor("#1b3d22"), 1.5));
    painter.setBrush(QColor(theme::kEgo));
    const QPointF triangle[3] = {{11.0, 0.0}, {-7.0, 6.0}, {-7.0, -6.0}};
    painter.drawPolygon(triangle, 3);
    // 中心点
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor("#1b3d22"));
    painter.drawEllipse(QPointF(0, 0), 1.6, 1.6);
    painter.restore();
  }

  // 比例尺、地图 ID 与跟车标记（MD3 chip 风）
  QFont f = painter.font();
  f.setPointSize(9);
  painter.setFont(f);
  painter.setPen(QPen(QColor(255, 255, 255, 70), 1.2));
  const double bar_m = 50.0;
  const double bar_px = bar_m * camera_.scale();
  const QPointF bar_a(14, height() - 18);
  const QPointF bar_b(14 + bar_px, height() - 18);
  painter.drawLine(bar_a, bar_b);
  painter.drawLine(bar_a + QPointF(0, -3), bar_a + QPointF(0, 3));
  painter.drawLine(bar_b + QPointF(0, -3), bar_b + QPointF(0, 3));
  painter.setPen(QColor(theme::kTextSecondary));
  painter.drawText(QPointF(14, height() - 22), QStringLiteral("50 m"));
  // 地图 ID chip
  const QString hash_text = map_hash_.isEmpty()
                                ? QString()
                                : QStringLiteral(" · %1").arg(map_hash_.left(8));
  const QString map_text = QStringLiteral("%1%2 · %3 lanes")
                               .arg(map_id_, hash_text)
                               .arg(lanes_.size());
  const QFontMetrics metrics(painter.font());
  QRectF id_rect(12, 12,
                 std::max(140, metrics.horizontalAdvance(map_text) + 18), 22);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(theme::kMdSurfaceContainerHigh));
  painter.drawRoundedRect(id_rect, 5, 5);
  painter.setPen(QColor(theme::kTextPrimary));
  painter.drawText(id_rect.adjusted(8, 0, -8, 0), Qt::AlignVCenter | Qt::AlignLeft,
                   map_text);
  if (follow_) {
    QRectF f_r(12, 40, 110, 22);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(theme::kMdPrimary));
    painter.drawRoundedRect(f_r, 5, 5);
    painter.setPen(QColor(theme::kMdOnPrimary));
    painter.drawText(f_r.adjusted(8, 0, -8, 0), Qt::AlignVCenter | Qt::AlignLeft,
                     QStringLiteral("跟车视角"));
  }
}

void MapView::resizeEvent(QResizeEvent*) {
  camera_.set_viewport(width(), height());
}

void MapView::wheelEvent(QWheelEvent* event) {
  const double factor = event->angleDelta().y() > 0 ? 1.2 : 1.0 / 1.2;
  camera_.zoom_at(event->position().x(), event->position().y(), factor);
  update();
}

void MapView::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) return;
  dragging_ = true;
  drag_moved_ = false;
  press_pos_ = event->position();
  last_drag_pos_ = press_pos_;
}

void MapView::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_) {
    const QPointF delta = event->position() - last_drag_pos_;
    last_drag_pos_ = event->position();
    const QPointF moved = event->position() - press_pos_;
    if (std::hypot(moved.x(), moved.y()) > kClickTolerancePx) drag_moved_ = true;
    if (drag_moved_ && follow_) {
      follow_ = false;   // 手动拖拽退出跟车，避免视角被立刻拉回
      emit followBroken();
    }
    camera_.pan_pixels(delta.x(), delta.y());
    update();
    return;
  }
  // 悬停吸附预览
  if (lanes_.isEmpty()) return;
  const auto world = camera_.screen_to_world(event->position().x(),
                                             event->position().y());
  const LaneHit hit = nearest_lane_point(lanes_, world.x, world.y,
                                         kSnapMaxDistanceM);
  hover_valid_ = hit.valid;
  hover_x_ = hit.x;
  hover_y_ = hit.y;
  update();
}

void MapView::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton || !dragging_) return;
  dragging_ = false;
  if (drag_moved_ || lanes_.isEmpty() || !goal_selection_enabled_) return;
  const auto world = camera_.screen_to_world(event->position().x(),
                                             event->position().y());
  const LaneHit hit = nearest_lane_point(lanes_, world.x, world.y,
                                         kSnapMaxDistanceM);
  if (hit.valid) {
    // Submit the same safe lane projection shown by the hover preview. Sending
    // the raw click made the marker disagree with the planner's canonical goal.
    pending_goal_x_ = hit.x;
    pending_goal_y_ = hit.y;
    click_timer_.start();   // 等双击窗口过后再提交，双击=适配全图不发目标
  } else {
    // 全局规划器 snap 会失败的点击就地拒绝，给出明确反馈
    const LaneHit nearest = nearest_lane_point(
        lanes_, world.x, world.y, std::numeric_limits<double>::infinity());
    emit goalRejected(nearest.distance);
  }
}

void MapView::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) return;
  click_timer_.stop();   // 双击取消挂起的单击选点
  if (follow_) {
    follow_ = false;
    emit followBroken();
  }
  fitToLanes();
}

void MapView::leaveEvent(QEvent*) {
  hover_valid_ = false;
  update();
}

}  // namespace adas::gui
