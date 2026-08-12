#ifndef ADAS_GUI__MAP_VIEW_HPP_
#define ADAS_GUI__MAP_VIEW_HPP_

#include <QPolygonF>
#include <QTimer>
#include <QVector>
#include <QWidget>

#include "map_camera.hpp"

namespace adas::gui {

struct GuiLaneConnection {
  qint64 to_lane_id{0};
  quint8 maneuver{0};
};

struct GuiLane {
  qint64 id{0};
  QPolygonF centerline;   // 世界坐标(m)
  double speed_limit_mps{0.0};
  bool junction{false};
  QVector<GuiLaneConnection> outgoing;
};

struct GuiMapObject {
  quint32 id{0};
  quint8 classification{0};
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
  double length_m{4.5};
  double width_m{1.8};
};

// CARLA 车道图视图。交互：
//   · 滚轮缩放（光标锚点）· 左键拖拽平移 · 双击适配全图
//   · 左键点击（无拖动）选择导航目标；悬停时预览就近车道吸附点，
//     超出吸附半径的点击就地拒绝（发 goalRejected，不发布目标）
//   · 跟车模式：视角锁定自车，手动拖拽自动退出（发 followBroken）
// 点击只发信号，发布行为由 MainWindow 决定。
class MapView : public QWidget {
  Q_OBJECT

 public:
  explicit MapView(QWidget* parent = nullptr);

// 与全局规划器 snap_max_distance_m 一致；超出则点击无效
  static constexpr double kSnapMaxDistanceM = 8.0;

  // 调试图层位掩码：默认仅显示道路、自车、路线、目标、悬停吸附预览。
  // 比赛演示模式下应关闭 Trail/Halo/Snap Radius 等"抢眼"装饰。
  enum LayerFlag : quint32 {
    kLayerTrail      = 1u << 0,    // 自车行驶尾迹
    kLayerEgoHalo    = 1u << 1,    // 自车发光晕（演示时压眼，默认关）
    kLayerSnapRadius = 1u << 2,    // 8m 吸附半径 ring
    kLayerGrid       = 1u << 3,    // 淡点阵网格
    // P1.F: 场景 actor 渲染路径已彻底移除。CARLA 才是唯一场景展示端；
    // GUI 仅画 Town 路网、自车、路线、目标、限速、路口。
  };
  static constexpr quint32 kDefaultLayers = kLayerGrid;  // 演示风：默认无尾迹、无 halo、无 snap ring

  public slots:
   void setLanes(const QVector<adas::gui::GuiLane>& lanes, const QString& map_id);
   void setMapMetadata(const QString& map_id, const QString& map_hash);
   // P0.3: 原子 LaneGraph 切换入口——同时写入 (lanes, map_id, map_hash);
   // 内部用 map_identity_changed() 决定是否清空旧地图的可视状态,避免
   // 依赖外部信号顺序,也不依赖 setLanes/setMapMetadata 先后顺序。
   void setLanesAtomic(const QVector<adas::gui::GuiLane>& lanes,
                       const QString& map_id, const QString& map_hash);
   // P1.F: 地图切换时清空路线、目标、相机，确保不会残留旧会话视觉。
   void clearForMapChange();

  // P1.F: 单元测试访问器，不参与 GUI 渲染。
  int routeSizeForTest() const { return route_.size(); }
   void setRoute(const QPolygonF& route);
   void setVehicle(double x, double y, double yaw_rad, bool valid);
   void setObjects(const QVector<adas::gui::GuiMapObject>& objects);
   void setGoal(double x, double y, bool valid);
   void setGoalSelectionEnabled(bool enabled);
   void setFollowEnabled(bool enabled);
   void fitToLanes();
   void clearTrail();
   // 调试图层显隐（位掩码 LayerFlag 组合）；0 表示全部装饰图层关闭。
   void setLayers(quint32 flags);
   quint32 layerFlags() const { return layer_flags_; }
   int objectCount() const { return objects_.size(); }

 signals:
  void goalRequested(double world_x, double world_y);
  void goalRejected(double distance_m);   // 点击离车道太远
  void followBroken();                    // 手动拖拽退出跟车

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void leaveEvent(QEvent* event) override;

 private:
  MapCamera camera_;
  QVector<GuiLane> lanes_;
  QVector<GuiMapObject> objects_;
  QString map_id_;
  QString map_hash_;
  QPolygonF route_;
  QPolygonF trail_;               // 自车轨迹尾迹（10 Hz 抽样，上限截断）
  double vehicle_x_{0.0}, vehicle_y_{0.0}, vehicle_yaw_{0.0};
  bool vehicle_valid_{false};
  double goal_x_{0.0}, goal_y_{0.0};
  bool goal_valid_{false};
  bool goal_selection_enabled_{true};
  bool fitted_{false};
  bool follow_{false};
  bool dragging_{false};
  bool drag_moved_{false};
  bool hover_valid_{false};
  double hover_x_{0.0}, hover_y_{0.0};
  QPointF press_pos_;
  QPointF last_drag_pos_;
  // 单击选点延迟提交：等确认不是双击（双击=适配全图）再发 goalRequested
  QTimer click_timer_;
  double pending_goal_x_{0.0}, pending_goal_y_{0.0};
  quint32 layer_flags_{kDefaultLayers};
};

}  // namespace adas::gui

Q_DECLARE_METATYPE(adas::gui::GuiLane)
Q_DECLARE_METATYPE(adas::gui::GuiMapObject)

#endif  // ADAS_GUI__MAP_VIEW_HPP_
