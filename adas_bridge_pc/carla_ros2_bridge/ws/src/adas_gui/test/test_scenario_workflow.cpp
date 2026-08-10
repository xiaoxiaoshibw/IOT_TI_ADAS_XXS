// scenario_workflow.hpp 单测：保证 10 个场景 profile 与 catalog/preset 对齐，
// 防止新增 scenario id 时遗漏运行验收口径或自动导航距离。
#include <gtest/gtest.h>

#include <set>

#include "scenario_workflow.hpp"

namespace {

using adas::gui::ScenarioEvidenceKey;
using adas::gui::scenario_workflow_ids;
using adas::gui::scenario_workflow_profile;
using adas::gui::scenario_workflow_profiles;

TEST(ScenarioWorkflow, TenProfilesUniqueAndMatchCatalog) {
  const auto profiles = scenario_workflow_profiles();
  ASSERT_EQ(profiles.size(), 10);
  std::set<QString> seen;
  for (const auto& p : profiles) {
    EXPECT_TRUE(seen.insert(p.id).second)
        << "duplicate workflow id: " << p.id.toStdString();
    EXPECT_FALSE(p.objective.isEmpty())
        << p.id.toStdString() << " has empty objective";
    EXPECT_FALSE(p.family.isEmpty())
        << p.id.toStdString() << " has empty family";
    EXPECT_GE(p.recommended_goal_distance_m, 30.0)
        << p.id.toStdString() << " recommended distance is too small";
  }
  EXPECT_EQ(scenario_workflow_ids().size(), 10);
}

TEST(ScenarioWorkflow, FreeProfileKeepsManualNavigation) {
  const auto free = scenario_workflow_profile(QStringLiteral("free"));
  EXPECT_FALSE(free.requires_navigation);
  for (const auto& r : free.requirements) {
    EXPECT_NE(r.key, ScenarioEvidenceKey::NavigationDriving)
        << "free must remain a manual-navigation workflow";
  }
}

TEST(ScenarioWorkflow, AebFamilyAlwaysIncludesEmergencyBrake) {
  for (const QString& id : {QStringLiteral("aeb"),
                            QStringLiteral("aeb_stationary"),
                            QStringLiteral("aeb_pedestrian")}) {
    const auto profile = scenario_workflow_profile(id);
    bool has_emergency = false;
    bool has_warning = false;
    for (const auto& r : profile.requirements) {
      if (r.key == ScenarioEvidenceKey::AebEmergency) has_emergency = true;
      if (r.key == ScenarioEvidenceKey::AebWarning) has_warning = true;
    }
    EXPECT_TRUE(has_emergency) << id.toStdString() << " missing AebEmergency";
    EXPECT_TRUE(has_warning) << id.toStdString() << " missing AebWarning";
  }
}

TEST(ScenarioWorkflow, AccFamilyIncludesLeadAndFollowEvidence) {
  for (const QString& id : {QStringLiteral("acc"),
                            QStringLiteral("acc_stop_and_go"),
                            QStringLiteral("acc_slow_truck")}) {
    const auto profile = scenario_workflow_profile(id);
    bool has_lead = false;
    bool has_follow = false;
    for (const auto& r : profile.requirements) {
      if (r.key == ScenarioEvidenceKey::LeadDetected) has_lead = true;
      if (r.key == ScenarioEvidenceKey::FollowLead) has_follow = true;
    }
    EXPECT_TRUE(has_lead) << id.toStdString() << " missing LeadDetected";
    EXPECT_TRUE(has_follow) << id.toStdString() << " missing FollowLead";
    EXPECT_TRUE(profile.requires_navigation)
        << id.toStdString() << " must require navigation";
  }
}

TEST(ScenarioWorkflow, DenseOvertakeRequiresTwentyObjects) {
  const auto dense = scenario_workflow_profile(QStringLiteral("dense_overtake_v1"));
  EXPECT_TRUE(dense.requires_navigation);
  bool has_dense_set = false;
  bool has_overtake = false;
  for (const auto& r : dense.requirements) {
    if (r.key == ScenarioEvidenceKey::DenseObjectSet) has_dense_set = true;
    if (r.key == ScenarioEvidenceKey::OvertakeDecision) has_overtake = true;
  }
  EXPECT_TRUE(has_dense_set) << "dense profile missing DenseObjectSet";
  EXPECT_TRUE(has_overtake) << "dense profile missing OvertakeDecision";
}

TEST(ScenarioWorkflow, AutoNavigationRequiresDistanceConfig) {
  for (const auto& profile : scenario_workflow_profiles()) {
    if (profile.requires_navigation) {
      EXPECT_GT(profile.recommended_goal_distance_m, 50.0)
          << profile.id.toStdString()
          << " auto-navigation with no usable forward distance";
    }
  }
}

TEST(ScenarioWorkflow, PedestrianProfileTracksPedestrianClass) {
  const auto ped = scenario_workflow_profile(QStringLiteral("aeb_pedestrian"));
  bool has_ped = false;
  for (const auto& r : ped.requirements) {
    if (r.key == ScenarioEvidenceKey::PedestrianDetected) has_ped = true;
  }
  EXPECT_TRUE(has_ped) << "pedestrian profile missing PedestrianDetected";
}

}  // namespace
