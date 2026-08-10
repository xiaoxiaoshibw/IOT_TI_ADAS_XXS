#ifndef ADAS_GUI__SCENARIO_WORKFLOW_HPP_
#define ADAS_GUI__SCENARIO_WORKFLOW_HPP_

#include <QList>
#include <QString>
#include <QStringList>

namespace adas::gui {

// GUI 侧场景验收口径。它只消费 ROS 遥测证据，不参与控制决策；同一份
// profile 同时驱动场景卡片、自动导航距离和运行中检查清单，避免三处硬编码漂移。
enum class ScenarioEvidenceKey {
  MapReady,
  EgoMoving,
  LaneValid,
  NavigationDriving,
  LeadDetected,
  FollowLead,
  OvertakeDecision,
  AebWarning,
  AebEmergency,
  PedestrianDetected,
  DenseObjectSet,
  SafetyHealthy,
};

struct ScenarioRequirement {
  ScenarioEvidenceKey key;
  QString label;
};

struct ScenarioWorkflowProfile {
  QString id;
  QString family;
  QString objective;
  QString operator_hint;
  bool requires_navigation{true};
  double recommended_goal_distance_m{500.0};
  QList<ScenarioRequirement> requirements;
};

inline QList<ScenarioWorkflowProfile> scenario_workflow_profiles() {
  using Key = ScenarioEvidenceKey;
  const QList<ScenarioRequirement> common = {
      {Key::MapReady, QStringLiteral("Town 地图已载入")},
      {Key::LaneValid, QStringLiteral("车道感知有效")},
      {Key::EgoMoving, QStringLiteral("自车进入闭环行驶")},
      {Key::SafetyHealthy, QStringLiteral("安全链无 MRM 故障")},
  };
  const QList<ScenarioRequirement> navigation_common =
      common + QList<ScenarioRequirement>{
                   {Key::NavigationDriving, QStringLiteral("导航路线执行中")}};
  const QList<ScenarioRequirement> acc_requirements =
      navigation_common + QList<ScenarioRequirement>{
                              {Key::LeadDetected, QStringLiteral("主前车已稳定检出")},
                              {Key::FollowLead, QStringLiteral("行为层进入跟车")}};
  const QList<ScenarioRequirement> vehicle_aeb_requirements =
      navigation_common + QList<ScenarioRequirement>{
                              {Key::LeadDetected, QStringLiteral("风险目标已检出")},
                              {Key::AebWarning, QStringLiteral("AEB 预警已触发")},
                              {Key::AebEmergency, QStringLiteral("AEB 紧急制动已触发")}};

  return {
      {QStringLiteral("lka"), QStringLiteral("LKA"),
       QStringLiteral("高速车道保持"),
       QStringLiteral("观察弯道横向误差与方向盘输出是否连续"), true, 800.0,
       navigation_common},
      {QStringLiteral("acc"), QStringLiteral("ACC"),
       QStringLiteral("自适应巡航跟随"),
       QStringLiteral("观察主前车选择、目标速度与安全时距"), true, 650.0,
       acc_requirements},
      {QStringLiteral("acc_stop_and_go"), QStringLiteral("ACC"),
       QStringLiteral("停车再走跟随"),
       QStringLiteral("观察主前车选择、目标速度与安全时距"), true, 650.0,
       acc_requirements},
      {QStringLiteral("acc_slow_truck"), QStringLiteral("ACC"),
       QStringLiteral("慢速卡车长时跟随"),
       QStringLiteral("观察主前车选择、目标速度与安全时距"), true, 650.0,
       acc_requirements},
      {QStringLiteral("overtake"), QStringLiteral("OVERTAKE"),
       QStringLiteral("慢车超越决策"),
       QStringLiteral("观察等待、执行、返回原车道的行为状态"), true, 900.0,
       navigation_common + QList<ScenarioRequirement>{
                               {Key::LeadDetected, QStringLiteral("慢速前车已检出")},
                               {Key::OvertakeDecision, QStringLiteral("超越状态已触发")}}},
      {QStringLiteral("aeb"), QStringLiteral("AEB"),
       QStringLiteral("前车急停紧急制动"),
       QStringLiteral("关注 TTC、预警到制动的状态跃迁"), true, 350.0,
       vehicle_aeb_requirements},
      {QStringLiteral("aeb_stationary"), QStringLiteral("AEB"),
       QStringLiteral("静止障碍物紧急制动"),
       QStringLiteral("关注 TTC、预警到制动的状态跃迁"), true, 350.0,
       vehicle_aeb_requirements},
      {QStringLiteral("aeb_pedestrian"), QStringLiteral("AEB"),
       QStringLiteral("横穿行人紧急制动"),
       QStringLiteral("关注行人分类、TTC 预警与紧急制动"), true, 350.0,
       navigation_common + QList<ScenarioRequirement>{
                               {Key::PedestrianDetected, QStringLiteral("行人目标已识别")},
                               {Key::AebWarning, QStringLiteral("AEB 预警已触发")},
                               {Key::AebEmergency, QStringLiteral("AEB 紧急制动已触发")}}},
      {QStringLiteral("free"), QStringLiteral("FREE"),
       QStringLiteral("自由巡航与长时间调试"),
       QStringLiteral("可在 Town 地图任意选择目标，随时重新规划"), false, 800.0,
       {{Key::MapReady, QStringLiteral("Town 地图已载入")},
        {Key::LaneValid, QStringLiteral("车道感知有效")},
        {Key::SafetyHealthy, QStringLiteral("安全链无 MRM 故障")}}},
      {QStringLiteral("dense_overtake_v1"), QStringLiteral("DENSE"),
       QStringLiteral("20 车确定性压力场景"),
       QStringLiteral("检查稳定 ID、目标排序与密集车流中的超越决策"), true, 900.0,
       navigation_common + QList<ScenarioRequirement>{
                               {Key::DenseObjectSet, QStringLiteral("20 个目标完整检出")},
                               {Key::OvertakeDecision, QStringLiteral("密集交通行为决策已触发")}}},
  };
}

inline QStringList scenario_workflow_ids() {
  QStringList ids;
  for (const auto& profile : scenario_workflow_profiles()) ids << profile.id;
  return ids;
}

inline ScenarioWorkflowProfile scenario_workflow_profile(const QString& id) {
  for (const auto& profile : scenario_workflow_profiles()) {
    if (profile.id == id) return profile;
  }
  using Key = ScenarioEvidenceKey;
  // 未知 ID 仅保留可观测的安全回退；覆盖测试会防止 catalog 场景落入此分支。
  return {id.isEmpty() ? QStringLiteral("free") : id, QStringLiteral("FREE"),
          QStringLiteral("自由巡航与长时间调试"),
          QStringLiteral("可在 Town 地图任意选择目标，随时重新规划"), false, 800.0,
          {{Key::MapReady, QStringLiteral("Town 地图已载入")},
           {Key::LaneValid, QStringLiteral("车道感知有效")},
           {Key::SafetyHealthy, QStringLiteral("安全链无 MRM 故障")}}};
}

inline bool scenario_requires_auto_navigation(const QString& id) {
  return scenario_workflow_profile(id).requires_navigation;
}

}  // namespace adas::gui

#endif  // ADAS_GUI__SCENARIO_WORKFLOW_HPP_
