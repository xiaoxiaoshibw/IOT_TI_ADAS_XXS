#ifndef ADAS_GUI__ROS_BRIDGE_HPP_
#define ADAS_GUI__ROS_BRIDGE_HPP_

#include <QObject>
#include <QString>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <QPolygonF>
#include <QTimer>

#include "adas_msgs/msg/actuation_command.hpp"
#include "adas_msgs/msg/aeb_status.hpp"
#include "adas_msgs/msg/behavior_state.hpp"
#include "adas_msgs/msg/gate_status.hpp"
#include "adas_msgs/msg/global_route.hpp"
#include "adas_msgs/msg/lane_graph.hpp"
#include "adas_msgs/msg/lane_state.hpp"
#include "adas_msgs/msg/mcu_status.hpp"
#include "adas_msgs/msg/navigation_status.hpp"
#include "adas_msgs/msg/safety_status.hpp"
#include "adas_msgs/msg/tracked_object_array.hpp"
#include "adas_msgs/srv/cancel_navigation.hpp"
#include "adas_msgs/srv/set_navigation_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "map_view.hpp"
#include "health_model.hpp"
#include "request_tracker.hpp"

namespace adas::gui {

// UI 线程可安全消费的 MCU 状态镜像（跨线程用 queued signal 传递）。
struct GuiMcuStatus {
  quint8 system_state{0};
  quint8 active_source{0};
  quint8 fault_level{0};
  quint16 fault_code{0};
  bool primary_fresh{false};
  bool aeb_floor_active{false};
  bool degraded{false};
  bool manual_override{false};
  bool estop{false};
  quint8 protocol_version{0};
  bool protocol_version_ok{false};
  bool test_build{false};
  float heartbeat_age_s{-1.0f};
  float feedback_age_s{-1.0f};
  float command_age_s{-1.0f};
  QString degrade_reason;
  bool ever_received{false};
};

struct GuiActuation {
  float steer{0.0f};
  float throttle{0.0f};
  float brake{0.0f};
  bool ever_received{false};
};

struct GuiNavStatus {
  quint8 state{0};
  QString detail;
  QString map_id;
  QString goal_id;
  QString route_id;
  float remaining_distance_m{0.0f};
  bool ever_received{false};
};

struct GuiLeadObject {
  float gap_m{0.0f};
  float speed_mps{0.0f};
  bool valid{false};
  bool ever_received{false};
};

struct GuiHealthStatus {
  QString id;
  QString display_name;
  HealthState state{HealthState::Unknown};
  QString detail;
};

// ROS2 executor 在独立线程 spin；GUI 主线程只经 Qt queued 信号收数据，
// DDS 回调不得触碰任何 QWidget（DEF-ARCH-002：GUI 不与控制环共享线程）。
class RosBridge : public QObject {
  Q_OBJECT

 public:
  explicit RosBridge(QObject* parent = nullptr);
  ~RosBridge() override;
  // MainWindow 完成所有 Qt 信号连接后再开始 spin，避免 transient-local 的
  // 一次性地图在订阅回调与 UI 连接之间到达而永久丢失。
  void start();

  // GUI 线程调用；rclcpp publisher 线程安全。导航目标/取消是 GUI 允许发布的
  // 唯二话题——它们只是导航请求，最终控制仍由 SoC 规划链与 MCU 仲裁。
  QString requestGoal(double world_x, double world_y);
  QString requestCancel(const QString& goal_id);
  bool hasCarlaBridgeNode() const;
  // SIL 适配仍沿用原 GUI 的 ROS 观察面；仅在 ADAS_GUI_MODE=sil 时把
  // vehicle_interface 的闭环输出映射为原来的 MCU/执行器显示模型。
  bool isSilMode() const { return sil_mode_; }
  bool isMilMode() const { return mil_mode_; }
  bool isMcuLessMode() const { return mcu_less_mode_; }
  // 故障注入命令（审计整改 TOP10-2）：向 `/adas/_debug/fault_inject_cmd`
  // 发布 JSON `{cmd,param,label,ts_ms,source}`。桥节点订阅后通过 PC CANalyst-II
  // 发送 0x301 帧到 MCU。仅当 MCU 烧录 ADAS_TEST_BUILD=1 时生效。
  QString requestFaultInjectCommand(int cmd, int param, const QString& label);

 signals:
  void mcuStatusChanged(const adas::gui::GuiMcuStatus& status);
  void actuationChanged(const adas::gui::GuiActuation& actuation);
  void dtcHistoryChanged(const QString& json);
  void laneGraphChanged(const QVector<adas::gui::GuiLane>& lanes,
                        const QString& map_id);
  void mapMetadataChanged(const QString& map_id, const QString& map_hash);
  // P0.3: 同一条 LaneGraph 消息中的 (lanes, map_id, map_hash) 一次性提交,
  // 避免历史 setLanes/setMapMetadata 双发信号之间的顺序依赖。GUI 消费者
  // 应当连接 laneGraphReady 而不是分开的两个信号。
  void laneGraphReady(const QVector<adas::gui::GuiLane>& lanes,
                      const QString& map_id, const QString& map_hash);
  void routeChanged(const QPolygonF& route);
  void navStatusChanged(const adas::gui::GuiNavStatus& status);
  void leadObjectChanged(const adas::gui::GuiLeadObject& lead);
  void objectsChanged(const QVector<adas::gui::GuiMapObject>& objects);
  void objectSummaryChanged(int visible_count, int primary_lead_id);
  // 自车运动状态：位置、航向、车速、横摆角速度（里程计限流 10 Hz）
  void egoChanged(double x, double y, double yaw_rad,
                  double speed_mps, double yaw_rate_rps);
  void behaviorChanged(int state, double target_speed_mps, int target_lane);
  void gateChanged(int source, bool limited, const QString& reason);
  void aebChanged(int state, double ttc_s, double required_decel_mps2);
  void safetyChanged(int overall, const QString& failed_components);
  void laneStateChanged(double lateral_offset_m, bool valid);
  void healthSnapshotChanged(const QVector<adas::gui::GuiHealthStatus>& statuses);
  void navigationRequestChanged(const QString& request_id,
                                const QString& operation, int state,
                                const QString& detail);
  void navigationGoalAccepted(const QString& request_id,
                              const QString& goal_id);
  void faultRequestChanged(const QString& request_id, int state,
                           const QString& detail);

 private:
  void spin();
  void updateHealthSnapshot();
  void handleObjects(const adas_msgs::msg::TrackedObjectArray& message);
  void emitSilMcuStatus();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<adas_msgs::msg::McuStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<adas_msgs::msg::ActuationCommand>::SharedPtr sub_actuation_;
  rclcpp::Subscription<adas_msgs::msg::ActuationCommand>::SharedPtr sub_sil_actuation_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_dtc_;
  rclcpp::Subscription<adas_msgs::msg::LaneGraph>::SharedPtr sub_map_;
  rclcpp::Subscription<adas_msgs::msg::GlobalRoute>::SharedPtr sub_global_route_;
  rclcpp::Subscription<adas_msgs::msg::NavigationStatus>::SharedPtr sub_nav_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<adas_msgs::msg::TrackedObjectArray>::SharedPtr sub_objects_;
  rclcpp::Subscription<adas_msgs::msg::LaneState>::SharedPtr sub_lane_state_;
  rclcpp::Subscription<adas_msgs::msg::BehaviorState>::SharedPtr sub_behavior_;
  rclcpp::Subscription<adas_msgs::msg::GateStatus>::SharedPtr sub_gate_;
  rclcpp::Subscription<adas_msgs::msg::AebStatus>::SharedPtr sub_aeb_;
  rclcpp::Subscription<adas_msgs::msg::SafetyStatus>::SharedPtr sub_safety_;
  rclcpp::Client<adas_msgs::srv::SetNavigationGoal>::SharedPtr client_goal_;
  rclcpp::Client<adas_msgs::srv::CancelNavigation>::SharedPtr client_cancel_;
  // 审计整改 TOP10-2：故障注入命令 publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_fault_inject_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_carla_visualization_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fault_ack_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  QTimer request_timer_;
  RequestTracker requests_;
  using Clock = std::chrono::steady_clock;
  Clock::time_point last_odom_{};
  Clock::time_point last_lane_{};
  Clock::time_point last_mcu_{};
  Clock::time_point last_actuation_{};
  Clock::time_point last_behavior_{};
  Clock::time_point last_gate_{};
  Clock::time_point last_aeb_{};
  Clock::time_point last_safety_{};
  Clock::time_point last_objects_{};
  bool ever_nav_{false};
  bool ever_mcu_{false};
  GuiMcuStatus latest_mcu_;
  int latest_safety_level_{-1};
  int latest_nav_state_{-1};
  bool have_latest_ego_{false};
  double latest_ego_x_{0.0};
  double latest_ego_y_{0.0};
  double latest_ego_yaw_{0.0};
  std::chrono::steady_clock::time_point last_pose_emit_{};
  std::chrono::steady_clock::time_point last_sil_status_emit_{};
  bool sil_mode_{false};
  bool mil_mode_{false};
  bool mcu_less_mode_{false};
  std::atomic<bool> running_{false};
  std::thread spin_thread_;
};

}  // namespace adas::gui

Q_DECLARE_METATYPE(adas::gui::GuiMcuStatus)
Q_DECLARE_METATYPE(adas::gui::GuiActuation)
Q_DECLARE_METATYPE(adas::gui::GuiNavStatus)
Q_DECLARE_METATYPE(adas::gui::GuiLeadObject)
Q_DECLARE_METATYPE(adas::gui::GuiHealthStatus)

#endif  // ADAS_GUI__ROS_BRIDGE_HPP_
