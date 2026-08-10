#include "ros_bridge.hpp"

#include <QDateTime>
#include <QProcess>
#include <QMetaObject>
#include <QStringList>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace adas::gui {
namespace {

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool has_node(const std::vector<std::string>& names, const std::string& wanted) {
  return std::any_of(names.begin(), names.end(), [&wanted](const std::string& name) {
    return name == wanted || name == "/" + wanted ||
           (name.size() > wanted.size() &&
            name.compare(name.size() - wanted.size(), wanted.size(), wanted) == 0 &&
            name[name.size() - wanted.size() - 1] == '/');
  });
}

QString age_detail(bool ever, double age_s, const QString& evidence) {
  if (!ever) return QStringLiteral("等待 %1").arg(evidence);
  return QStringLiteral("%1，最近更新 %2 s").arg(evidence).arg(age_s, 0, 'f', 1);
}

}  // namespace

RosBridge::RosBridge(QObject* parent) : QObject(parent) {
  sil_mode_ = QString::fromLocal8Bit(qgetenv("ADAS_GUI_MODE"))
                  .compare(QStringLiteral("sil"), Qt::CaseInsensitive) == 0;
  qRegisterMetaType<GuiMcuStatus>("adas::gui::GuiMcuStatus");
  qRegisterMetaType<GuiActuation>("adas::gui::GuiActuation");
  qRegisterMetaType<GuiNavStatus>("adas::gui::GuiNavStatus");
  qRegisterMetaType<GuiLeadObject>("adas::gui::GuiLeadObject");
  qRegisterMetaType<GuiHealthStatus>("adas::gui::GuiHealthStatus");
  qRegisterMetaType<QVector<GuiHealthStatus>>("QVector<adas::gui::GuiHealthStatus>");
  qRegisterMetaType<QVector<GuiLane>>("QVector<adas::gui::GuiLane>");
  qRegisterMetaType<QPolygonF>("QPolygonF");

  node_ = std::make_shared<rclcpp::Node>("adas_gui");
  sub_status_ = node_->create_subscription<adas_msgs::msg::McuStatus>(
      "/adas/mcu/status", rclcpp::QoS(1).reliable(),
      [this](adas_msgs::msg::McuStatus::ConstSharedPtr message) {
        GuiMcuStatus status;
        status.system_state = message->system_state;
        status.active_source = message->active_source;
        status.fault_level = message->fault_level;
        status.fault_code = message->fault_code;
        status.primary_fresh = message->primary_fresh;
        status.aeb_floor_active = message->aeb_floor_active;
        status.degraded = message->degraded;
        status.manual_override = message->manual_override;
        status.estop = message->estop;
        status.protocol_version = message->protocol_version;
        status.protocol_version_ok = message->protocol_version_ok;
        status.test_build = message->test_build;
        status.heartbeat_age_s = message->heartbeat_age_s;
        status.feedback_age_s = message->feedback_age_s;
        status.command_age_s = message->command_age_s;
        status.degrade_reason = QString::fromStdString(message->degrade_reason);
        status.ever_received = true;
        last_mcu_ = Clock::now();
        ever_mcu_ = true;
        latest_mcu_ = status;
        emit mcuStatusChanged(status);
      });
  sub_actuation_ = node_->create_subscription<adas_msgs::msg::ActuationCommand>(
      "/adas/mcu/actuation_feedback", rclcpp::QoS(1).reliable(),
      [this](adas_msgs::msg::ActuationCommand::ConstSharedPtr message) {
        GuiActuation actuation;
        actuation.steer = message->steer;
        actuation.throttle = message->throttle;
        actuation.brake = message->brake;
        actuation.ever_received = true;
        last_actuation_ = Clock::now();
        emit actuationChanged(actuation);
      });
  if (sil_mode_) {
    // SIL 没有 MCU/CAN feedback 话题；vehicle_interface 的闭环命令就是原 GUI
    // 执行器卡片需要的同构数据。保留 HIL 订阅，上线时只由模式开关决定数据源。
    sub_sil_actuation_ = node_->create_subscription<adas_msgs::msg::ActuationCommand>(
        "/adas/vehicle/actuation_cmd", rclcpp::QoS(10).reliable(),
        [this](adas_msgs::msg::ActuationCommand::ConstSharedPtr message) {
          GuiActuation actuation;
          actuation.steer = message->steer;
          actuation.throttle = message->throttle;
          actuation.brake = message->brake;
          actuation.ever_received = true;
          last_actuation_ = Clock::now();
          emit actuationChanged(actuation);
          emitSilMcuStatus();
        });
  }
  {
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    sub_dtc_ = node_->create_subscription<std_msgs::msg::String>(
        "/adas/diagnostics/dtc_history", qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          emit dtcHistoryChanged(QString::fromStdString(message->data));
        });
  }
  {
    rclcpp::QoS map_qos(1);
    map_qos.reliable().transient_local();
    sub_map_ = node_->create_subscription<adas_msgs::msg::LaneGraph>(
        "/adas/map/lane_graph", map_qos,
        [this](adas_msgs::msg::LaneGraph::ConstSharedPtr message) {
          QVector<GuiLane> lanes;
          lanes.reserve(static_cast<int>(message->lanes.size()));
          for (const auto& lane : message->lanes) {
            GuiLane converted;
            converted.junction = lane.junction;
            converted.centerline.reserve(static_cast<int>(lane.centerline.size()));
            for (const auto& pose : lane.centerline) {
              converted.centerline.append(
                  QPointF(pose.position.x, pose.position.y));
            }
            lanes.append(converted);
          }
          emit laneGraphChanged(lanes, QString::fromStdString(message->map_id));
        });
    sub_route_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/adas/planning/global_route", map_qos,
        [this](nav_msgs::msg::Path::ConstSharedPtr message) {
          QPolygonF route;
          route.reserve(static_cast<int>(message->poses.size()));
          for (const auto& pose : message->poses) {
            route.append(QPointF(pose.pose.position.x, pose.pose.position.y));
          }
          emit routeChanged(route);
        });
    sub_nav_ = node_->create_subscription<adas_msgs::msg::NavigationStatus>(
        "/adas/navigation/status", map_qos,
        [this](adas_msgs::msg::NavigationStatus::ConstSharedPtr message) {
          GuiNavStatus status;
          status.state = message->state;
          status.detail = QString::fromStdString(message->detail);
          status.goal_id = QString::fromStdString(message->goal_id);
          status.remaining_distance_m = message->remaining_distance_m;
          status.ever_received = true;
          ever_nav_ = true;
          latest_nav_state_ = static_cast<int>(message->state);
          emit navStatusChanged(status);
        });
  }
  // 只订阅 SoC 端 tracker 输出的 `/adas/perception/objects`，与行为规划/AEB
  // 实际消费的源一致；raw CARLA 真值（`/adas/perception/objects_raw`，primary
  // lead 恒为 -1）只是 tracker 的输入，不再用于前车距离显示，避免双源交错
  // 导致 HUD 抖动。
  sub_objects_ = node_->create_subscription<adas_msgs::msg::TrackedObjectArray>(
      "/adas/perception/objects", rclcpp::SensorDataQoS(),
      [this](adas_msgs::msg::TrackedObjectArray::ConstSharedPtr message) {
        handleObjects(*message);
      });
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/adas/localization/kinematic_state", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        last_odom_ = Clock::now();
        latest_ego_x_ = message->pose.pose.position.x;
        latest_ego_y_ = message->pose.pose.position.y;
        latest_ego_yaw_ = yaw_from_quaternion(message->pose.pose.orientation);
        have_latest_ego_ = true;
        // 50 Hz 里程计限流到 10 Hz，避免 queued 信号淹没 UI 线程
        const auto now = std::chrono::steady_clock::now();
        if (now - last_pose_emit_ < std::chrono::milliseconds(100)) return;
        last_pose_emit_ = now;
        emit egoChanged(latest_ego_x_,
                        latest_ego_y_,
                        latest_ego_yaw_,
                        message->twist.twist.linear.x,
                        message->twist.twist.angular.z);
      });
  sub_lane_state_ = node_->create_subscription<adas_msgs::msg::LaneState>(
      "/adas/perception/lane_state", rclcpp::SensorDataQoS(),
      [this](adas_msgs::msg::LaneState::ConstSharedPtr message) {
        last_lane_ = Clock::now();
        emit laneStateChanged(message->lateral_offset, message->valid);
      });
  sub_behavior_ = node_->create_subscription<adas_msgs::msg::BehaviorState>(
      "/adas/planning/behavior", rclcpp::QoS(10),
      [this](adas_msgs::msg::BehaviorState::ConstSharedPtr message) {
        last_behavior_ = Clock::now();
        emit behaviorChanged(message->state, message->target_speed_mps,
                             message->target_lane);
      });
  sub_gate_ = node_->create_subscription<adas_msgs::msg::GateStatus>(
      "/adas/control/gate/status", rclcpp::QoS(10),
      [this](adas_msgs::msg::GateStatus::ConstSharedPtr message) {
        last_gate_ = Clock::now();
        emit gateChanged(message->selected_source, message->limited,
                         QString::fromStdString(message->reason));
      });
  {
    rclcpp::QoS live(1);
    live.reliable();
    sub_aeb_ = node_->create_subscription<adas_msgs::msg::AebStatus>(
        "/adas/control/aeb/status", live,
        [this](adas_msgs::msg::AebStatus::ConstSharedPtr message) {
          last_aeb_ = Clock::now();
          emit aebChanged(message->state, message->ttc_s,
                          message->required_decel_mps2);
        });
    sub_safety_ = node_->create_subscription<adas_msgs::msg::SafetyStatus>(
        "/adas/system/safety_status", live,
        [this](adas_msgs::msg::SafetyStatus::ConstSharedPtr message) {
          last_safety_ = Clock::now();
          latest_safety_level_ = static_cast<int>(message->overall);
          QStringList failed;
          for (const auto& component : message->failed_components) {
            failed << QString::fromStdString(component);
          }
          emit safetyChanged(message->overall, failed.join("+"));
        });
  }
  client_goal_ = node_->create_client<adas_msgs::srv::SetNavigationGoal>(
      "/adas/navigation/set_goal");
  client_cancel_ = node_->create_client<adas_msgs::srv::CancelNavigation>(
      "/adas/navigation/cancel_goal");
  // 审计整改 TOP10-2：故障注入命令 topic；桥节点订阅后通过 0x301 帧发给 MCU
  pub_fault_inject_ = node_->create_publisher<std_msgs::msg::String>(
      "/adas/_debug/fault_inject_cmd", rclcpp::QoS(10).reliable());

  health_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), [this]() { updateHealthSnapshot(); });

  request_timer_.setInterval(200);
  connect(&request_timer_, &QTimer::timeout, this, [this]() {
    const auto expired = requests_.expire(QDateTime::currentMSecsSinceEpoch());
    for (const auto& record : expired) {
      emit navigationRequestChanged(record.request_id, record.operation,
                                    static_cast<int>(record.state),
                                    QStringLiteral("请求超时，可重试"));
    }
  });
  request_timer_.start();

  spin_thread_ = std::thread([this]() { spin(); });
}

QString RosBridge::requestGoal(double world_x, double world_y) {
  const QString id = QUuid::createUuid().toString(QUuid::WithoutBraces);
  const QString operation = QStringLiteral("navigation.goal");
  if (!requests_.begin(operation, id, QDateTime::currentMSecsSinceEpoch(), 5000)) {
    return {};
  }
  emit navigationRequestChanged(id, operation,
                                static_cast<int>(RequestState::Sent),
                                QStringLiteral("目标请求已发送"));
  if (!client_goal_->service_is_ready()) {
    requests_.finish(operation, id, RequestState::Failed,
                     QStringLiteral("导航服务不可用"));
    emit navigationRequestChanged(id, operation,
                                  static_cast<int>(RequestState::Failed),
                                  QStringLiteral("导航服务不可用"));
    return {};
  }
  auto request = std::make_shared<adas_msgs::srv::SetNavigationGoal::Request>();
  request->request_id = id.toStdString();
  request->goal.header.stamp = node_->now();
  request->goal.header.frame_id = "map";
  request->goal.pose.position.x = world_x;
  request->goal.pose.position.y = world_y;
  request->goal.pose.orientation.w = 1.0;
  client_goal_->async_send_request(
      request, [this, id, operation](
                   rclcpp::Client<adas_msgs::srv::SetNavigationGoal>::SharedFuture future) {
        const auto response = future.get();
        QMetaObject::invokeMethod(this, [this, id, operation, response]() {
          const bool accepted = response->accepted &&
              QString::fromStdString(response->request_id) == id;
          const auto state = accepted ? RequestState::Acknowledged
                                      : RequestState::Failed;
          const QString detail = QString::fromStdString(response->message);
          if (!requests_.finish(operation, id, state, detail)) return;
          emit navigationRequestChanged(id, operation,
                                        static_cast<int>(state), detail);
        }, Qt::QueuedConnection);
      });
  return id;
}

QString RosBridge::requestCancel(const QString& goal_id) {
  const QString id = QUuid::createUuid().toString(QUuid::WithoutBraces);
  const QString operation = QStringLiteral("navigation.cancel");
  if (!requests_.begin(operation, id, QDateTime::currentMSecsSinceEpoch(), 5000)) {
    return {};
  }
  emit navigationRequestChanged(id, operation,
                                static_cast<int>(RequestState::Sent),
                                QStringLiteral("取消请求已发送"));
  if (!client_cancel_->service_is_ready()) {
    requests_.finish(operation, id, RequestState::Failed,
                     QStringLiteral("导航取消服务不可用"));
    emit navigationRequestChanged(id, operation,
                                  static_cast<int>(RequestState::Failed),
                                  QStringLiteral("导航取消服务不可用"));
    return {};
  }
  auto request = std::make_shared<adas_msgs::srv::CancelNavigation::Request>();
  request->request_id = id.toStdString();
  request->goal_id = goal_id.toStdString();
  client_cancel_->async_send_request(
      request, [this, id, operation](
                   rclcpp::Client<adas_msgs::srv::CancelNavigation>::SharedFuture future) {
        const auto response = future.get();
        QMetaObject::invokeMethod(this, [this, id, operation, response]() {
          const bool accepted = response->accepted &&
              QString::fromStdString(response->request_id) == id;
          const auto state = accepted ? RequestState::Acknowledged
                                      : RequestState::Failed;
          const QString detail = QString::fromStdString(response->message);
          if (!requests_.finish(operation, id, state, detail)) return;
          emit navigationRequestChanged(id, operation,
                                        static_cast<int>(state), detail);
        }, Qt::QueuedConnection);
      });
  return id;
}

void RosBridge::publishFaultInjectCommand(int cmd, int param, const QString& label) {
  std_msgs::msg::String msg;
  // 极简 JSON 序列化（避免引 nlohmann 依赖）：手工构造 {"cmd":N,"param":N,...}
  QString escaped = label;
  escaped.replace(QChar('"'), QLatin1String("\\\""));
  QString json = QStringLiteral("{\"cmd\":%1,\"param\":%2,\"label\":\"%3\","
                                "\"ts_ms\":%4,\"source\":\"adas_gui\"}")
                     .arg(cmd)
                     .arg(param)
                     .arg(escaped)
                     .arg(QDateTime::currentMSecsSinceEpoch());
  msg.data = json.toStdString();
  pub_fault_inject_->publish(msg);
}

void RosBridge::emitSilMcuStatus() {
  if (!sil_mode_) return;
  const auto now = Clock::now();
  if (last_sil_status_emit_ != Clock::time_point{} &&
      now - last_sil_status_emit_ < std::chrono::milliseconds(100)) {
    return;
  }
  last_sil_status_emit_ = now;

  GuiMcuStatus status;
  // 给原 SafetyPanel 提供与 HIL McuStatus 相同的语义：SIL 的控制栈正常时
  // 等价于 ACTIVE/PRIMARY，安全聚合进入 MRM 时映射为对应的停车态。
  status.system_state = latest_safety_level_ >= 3 ? 5U
                       : latest_safety_level_ >= 2 ? 4U
                       : latest_safety_level_ == 1 ? 3U : 2U;
  status.active_source = 1U;
  status.fault_level = latest_safety_level_ >= 2 ? 2U
                      : latest_safety_level_ == 1 ? 1U : 0U;
  status.primary_fresh = true;
  status.aeb_floor_active = false;
  status.degraded = latest_safety_level_ >= 1;
  status.manual_override = false;
  status.estop = latest_safety_level_ >= 3;
  status.protocol_version = 3U;
  status.protocol_version_ok = true;
  status.test_build = false;
  status.heartbeat_age_s = 0.0F;
  status.feedback_age_s = 0.0F;
  status.command_age_s = 0.0F;
  status.degrade_reason = QStringLiteral("SIL vehicle_interface（无 MCU/CAN）");
  status.ever_received = true;
  last_mcu_ = now;
  ever_mcu_ = true;
  latest_mcu_ = status;
  emit mcuStatusChanged(status);
}

bool RosBridge::hasCarlaBridgeNode() const {
  auto names = node_->get_node_names();
  if (has_node(names, "carla_bridge")) {
    // DDS zombie: 进程已死但 discovery 缓存未过期（~10s），节点名仍在。
    // 用 pgrep 确认进程真实存活，避免误判孤儿/僵尸为"外部实例"。
    QProcess check;
    check.start("pgrep", {"-f", "adas_carla_bridge.*bridge_node|carla_bridge"});
    check.waitForFinished(1000);
    return check.exitCode() == 0;
  }
  if (sil_mode_) {
    // SIL 没有 carla_bridge 节点；用控制输出和安全态势两个发布者作为
    // 原 GUI 的 bridge probe，允许一键启动时复用已经运行的 SIL。
    return node_->count_publishers("/adas/vehicle/actuation_cmd") > 0U &&
           node_->count_publishers("/adas/system/safety_status") > 0U;
  }
  return false;
}

void RosBridge::handleObjects(const adas_msgs::msg::TrackedObjectArray& message) {
  GuiLeadObject lead;
  lead.ever_received = true;

  if (message.primary_lead_id >= 0 && std::isfinite(message.primary_lead_gap_m)) {
    lead.valid = true;
    lead.gap_m = message.primary_lead_gap_m;
    lead.speed_mps = std::isfinite(message.primary_lead_speed_mps)
                         ? message.primary_lead_speed_mps : 0.0f;
    emit leadObjectChanged(lead);
    return;
  }

  float best_gap = std::numeric_limits<float>::infinity();
  float best_speed = 0.0f;
  bool found = false;
  const double cos_yaw = std::cos(latest_ego_yaw_);
  const double sin_yaw = std::sin(latest_ego_yaw_);
  for (const auto& object : message.objects) {
    double longitudinal = object.path_longitudinal_m;
    double lateral = object.path_lateral_m;
    if ((!std::isfinite(longitudinal) || longitudinal <= 0.01) && have_latest_ego_) {
      const double dx = object.pose.pose.position.x - latest_ego_x_;
      const double dy = object.pose.pose.position.y - latest_ego_y_;
      longitudinal = cos_yaw * dx + sin_yaw * dy;
      lateral = -sin_yaw * dx + cos_yaw * dy;
    }
    if (!std::isfinite(longitudinal) || !std::isfinite(lateral)) continue;
    if (longitudinal <= 0.0 || std::abs(lateral) > 2.5) continue;
    if (longitudinal < best_gap) {
      best_gap = static_cast<float>(longitudinal);
      best_speed = object.twist.twist.linear.x;
      found = true;
    }
  }

  lead.valid = found;
  if (found) {
    lead.gap_m = best_gap;
    lead.speed_mps = best_speed;
  }
  emit leadObjectChanged(lead);
}

void RosBridge::updateHealthSnapshot() {
  const auto now = Clock::now();
  const auto age = [now](Clock::time_point stamp) {
    if (stamp == Clock::time_point{}) return -1.0;
    return std::chrono::duration<double>(now - stamp).count();
  };
  const auto fresh = [&age](Clock::time_point stamp, double limit_s) {
    const double seconds = age(stamp);
    return seconds >= 0.0 && seconds <= limit_s;
  };
  const auto names = node_->get_node_names();
  const bool sil_runtime = sil_mode_ &&
                           node_->count_publishers("/adas/vehicle/actuation_cmd") > 0U;
  const bool bridge_node = has_node(names, "carla_bridge") || sil_runtime;
  const bool odom_fresh = fresh(last_odom_, 0.5);
  const bool lane_fresh = fresh(last_lane_, 0.5);
  const bool mcu_fresh = fresh(last_mcu_, 0.5);
  const bool actuation_fresh = fresh(last_actuation_, 0.5);
  const bool safety_fresh = fresh(last_safety_, 0.6);
  const bool stack_evidence = fresh(last_behavior_, 0.6) && fresh(last_gate_, 0.6) &&
                              fresh(last_aeb_, 0.6) && safety_fresh;

  QVector<GuiHealthStatus> out;
  out.reserve(9);
  const auto add = [&out](const char* id, const char* name, HealthState state,
                          const QString& detail) {
    out.append({QString::fromLatin1(id), QString::fromUtf8(name), state, detail});
  };

  HealthState carla_state = HealthState::Offline;
  if (bridge_node && odom_fresh) carla_state = HealthState::Healthy;
  else if (bridge_node && last_odom_ == Clock::time_point{}) carla_state = HealthState::Starting;
  else if (bridge_node) carla_state = HealthState::Degraded;
  add("carla", sil_mode_ ? "SIL 仿真" : "CARLA", carla_state,
      sil_mode_ ? QStringLiteral("本地 SIL 里程计=%1，控制输出=%2")
                      .arg(odom_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"),
                           actuation_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"))
                : age_detail(last_odom_ != Clock::time_point{}, age(last_odom_),
                             QStringLiteral("里程计流")));

  HealthState bridge_state = HealthState::Offline;
  if (bridge_node && odom_fresh && lane_fresh) bridge_state = HealthState::Healthy;
  else if (bridge_node && last_odom_ == Clock::time_point{}) bridge_state = HealthState::Starting;
  else if (bridge_node) bridge_state = HealthState::Degraded;
  add("bridge", sil_mode_ ? "SIL 控制栈" : "CARLA ROS2 Bridge", bridge_state,
      QStringLiteral("节点/运行时=%1，里程计=%2，车道=%3")
          .arg(bridge_node ? QStringLiteral("已发现") : QStringLiteral("未发现"),
               odom_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"),
               lane_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流")));

  static const std::vector<std::string> kStackNodes = {
      "vehicle_interface", "command_gate", "safety_monitor", "aeb",
      "trajectory_follower", "trajectory_planner", "behavior_planner", "object_tracker"};
  // Orin 跑 Humble、上位机跑 Jazzy：两者 ros_discovery_info 的 type-hash 不兼容，
  // get_node_names() 结构性地枚举不到 Orin 节点名（数据面照常收发，与图元数据无关）。
  // 因此 Orin 侧三块（控制栈 / CAN Gateway / Safety Monitor）一律以话题新鲜度作
  // 存活判据（与下方 F280025C MCU 块一致），节点名仅并入明细供诊断，不参与判定。
  int stack_nodes = 0;
  for (const auto& expected : kStackNodes) stack_nodes += has_node(names, expected) ? 1 : 0;
  const bool stack_ever = last_behavior_ != Clock::time_point{} ||
                          last_gate_ != Clock::time_point{} ||
                          last_aeb_ != Clock::time_point{} ||
                          last_safety_ != Clock::time_point{};
  HealthState stack_state = HealthState::Offline;
  if (stack_evidence) stack_state = HealthState::Healthy;
  else if (!stack_ever) stack_state = HealthState::Starting;
  else stack_state = HealthState::Degraded;
  add("orin_stack", "Orin控制栈", stack_state,
      QStringLiteral("控制话题(behavior/gate/aeb/safety)=%1，节点名可见 %2/%3（跨发行版仅供参考）")
          .arg(stack_evidence ? QStringLiteral("新鲜") : QStringLiteral("不完整"))
          .arg(stack_nodes).arg(kStackNodes.size()));

  const bool gateway_node = has_node(names, "can_gateway");
  HealthState gateway_state = HealthState::Offline;
  if (sil_mode_ && actuation_fresh) gateway_state = HealthState::Healthy;
  else if (mcu_fresh && actuation_fresh) gateway_state = HealthState::Healthy;
  else if (!ever_mcu_) gateway_state = HealthState::Starting;
  else gateway_state = HealthState::Degraded;
  add("can_gateway", sil_mode_ ? "SIL 执行输出" : "CAN Gateway", gateway_state,
      QStringLiteral("MCU遥测=%1，执行反馈=%2，节点名=%3（跨发行版仅供参考）")
          .arg(mcu_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"),
               actuation_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"),
               gateway_node ? QStringLiteral("已发现") : QStringLiteral("未发现")));

  const bool safety_node = has_node(names, "safety_monitor");
  HealthState safety_state = HealthState::Offline;
  if (safety_fresh) {
    safety_state = latest_safety_level_ >= 2 ? HealthState::Fault
                 : latest_safety_level_ == 1 ? HealthState::Degraded
                                             : HealthState::Healthy;
  } else if (last_safety_ == Clock::time_point{}) {
    safety_state = HealthState::Starting;
  } else {
    safety_state = HealthState::Degraded;
  }
  add("safety_monitor", "Safety Monitor", safety_state,
      QStringLiteral("状态流=%1，等级=%2，节点名=%3（跨发行版仅供参考）")
          .arg(safety_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"))
          .arg(latest_safety_level_)
          .arg(safety_node ? QStringLiteral("已发现") : QStringLiteral("未发现")));

  HealthState mcu_state = HealthState::Unknown;
  if (ever_mcu_ && !mcu_fresh) mcu_state = HealthState::Offline;
  else if (mcu_fresh) {
    mcu_state = (latest_mcu_.system_state >= 5U || latest_mcu_.fault_level >= 2U)
                    ? HealthState::Fault
              : (latest_mcu_.degraded || latest_mcu_.system_state == 3U ||
                 latest_mcu_.system_state == 4U)
                    ? HealthState::Degraded
                    : HealthState::Healthy;
  }
  add("mcu", sil_mode_ ? "SIL MCU 模型" : "F280025C MCU", mcu_state,
      ever_mcu_ ? QStringLiteral("状态=%1，故障码=0x%2，心跳年龄=%3 s")
                      .arg(latest_mcu_.system_state)
                      .arg(latest_mcu_.fault_code, 4, 16, QChar('0'))
                      .arg(latest_mcu_.heartbeat_age_s, 0, 'f', 2)
                : QStringLiteral("等待 /adas/mcu/status"));

  HealthState can_state = HealthState::Unknown;
  if (sil_mode_) {
    can_state = actuation_fresh ? HealthState::Healthy :
                last_actuation_ == Clock::time_point{} ? HealthState::Starting
                                                        : HealthState::Degraded;
  }
  if (!sil_mode_ && ever_mcu_ && !mcu_fresh) can_state = HealthState::Offline;
  else if (!sil_mode_ && mcu_fresh) {
    const bool bus_off = (latest_mcu_.fault_code & (1U << 11U)) != 0U;
    if (bus_off || !latest_mcu_.protocol_version_ok) can_state = HealthState::Fault;
    else if (latest_mcu_.feedback_age_s < 0.0f || latest_mcu_.feedback_age_s >= 0.5f)
      can_state = HealthState::Degraded;
    else can_state = HealthState::Healthy;
  }
  add("can_link", sil_mode_ ? "SIL 数据链路" : "CAN链路", can_state,
      sil_mode_ ? QStringLiteral("SIL 不经 CAN，执行输出=%1")
                      .arg(actuation_fresh ? QStringLiteral("新鲜") : QStringLiteral("断流"))
                : ever_mcu_ ? QStringLiteral("反馈年龄=%1 s，协议=%2")
                      .arg(latest_mcu_.feedback_age_s, 0, 'f', 2)
                      .arg(latest_mcu_.protocol_version_ok ? QStringLiteral("匹配")
                                                           : QStringLiteral("不匹配"))
                : QStringLiteral("等待 CAN 网关证据"));

  HealthState odom_state = HealthState::Offline;
  const bool odom_publisher = node_->count_publishers("/adas/localization/kinematic_state") > 0U;
  if (odom_fresh) odom_state = HealthState::Healthy;
  else if (odom_publisher && last_odom_ == Clock::time_point{}) odom_state = HealthState::Starting;
  else if (odom_publisher) odom_state = HealthState::Degraded;
  add("odometry", "里程计", odom_state,
      age_detail(last_odom_ != Clock::time_point{}, age(last_odom_),
                 QStringLiteral("/adas/localization/kinematic_state")));

  const bool global_planner = has_node(names, "global_planner");
  const bool trajectory_planner = has_node(names, "trajectory_planner");
  HealthState nav_state = HealthState::Offline;
  if (global_planner && trajectory_planner && ever_nav_) {
    nav_state = latest_nav_state_ == 5 ? HealthState::Degraded : HealthState::Healthy;
  } else if (global_planner || trajectory_planner) {
    nav_state = (global_planner && trajectory_planner) ? HealthState::Starting
                                                       : HealthState::Degraded;
  }
  add("navigation", "导航模块", nav_state,
      QStringLiteral("全局规划=%1，轨迹规划=%2，导航状态=%3")
          .arg(global_planner ? QStringLiteral("在线") : QStringLiteral("离线"),
               trajectory_planner ? QStringLiteral("在线") : QStringLiteral("离线"))
          .arg(latest_nav_state_));

  emit healthSnapshotChanged(out);
}

RosBridge::~RosBridge() {
  running_ = false;
  if (spin_thread_.joinable()) spin_thread_.join();
}

void RosBridge::spin() {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  while (running_ && rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(50));
  }
}

}  // namespace adas::gui
