// 行为状态机实现
#include "adas_behavior_planner/behavior_core.hpp"

#include <cmath>

namespace adas::planning {

BehaviorCore::BehaviorCore(const BehaviorParams& params) : params_(params) {}

// 邻道（目标车道）清空校验：目标车道带内、纵向窗口 [-rear, front] 内无目标
bool BehaviorCore::adjacent_lane_clear(const BehaviorInput& in) const {
  const double band_center = -static_cast<double>(params_.target_lane) * params_.lane_width_m;
  const double half_band = params_.lane_width_m * 0.6;  // 稍宽于半车道，含跨线目标
  for (const auto& o : in.objects) {
    if (std::fabs(o.lat_m - band_center) < half_band && o.lon_m > -params_.clear_rear_m &&
        o.lon_m < params_.clear_front_m) {
      return false;
    }
  }
  return true;
}

BehaviorOutput BehaviorCore::update(const BehaviorInput& in) {
  // MRM 停车最高优先级
  if (in.mrm_stop) {
    state_ = BehaviorKind::kStopping;
    exit_count_ = slow_count_ = clear_count_ = lost_count_ = 0;
  } else if (state_ == BehaviorKind::kStopping) {
    state_ = BehaviorKind::kLaneFollow;
  }

  const bool lead_slow = in.primary_lead_id >= 0 &&
                         in.lead_speed_mps <
                             params_.overtake_slow_ratio * params_.cruise_speed_mps &&
                         in.lead_gap_m < params_.overtake_trigger_gap_m;

  switch (state_) {
    case BehaviorKind::kLaneFollow:
      if (in.primary_lead_id >= 0 && in.lead_gap_m < params_.follow_enter_range_m) {
        state_ = BehaviorKind::kFollowLead;
        exit_count_ = 0;
        slow_count_ = 0;
      }
      break;

    case BehaviorKind::kFollowLead:
      // 退出跟车（前车远离/消失）
      if (in.primary_lead_id < 0 || in.lead_gap_m > params_.follow_exit_range_m) {
        if (++exit_count_ >= params_.exit_hysteresis_frames) {
          state_ = BehaviorKind::kLaneFollow;
          exit_count_ = 0;
        }
        break;
      }
      exit_count_ = 0;
      // 慢车持续 → 准备超车
      if (params_.overtake_enabled && lead_slow) {
        if (++slow_count_ >= params_.overtake_wait_frames) {
          state_ = BehaviorKind::kOvertakeWait;
          clear_count_ = 0;
        }
      } else {
        slow_count_ = 0;
      }
      break;

    case BehaviorKind::kOvertakeWait:
      // 前车恢复速度/消失 → 放弃
      if (in.primary_lead_id < 0 || !lead_slow) {
        state_ = BehaviorKind::kFollowLead;
        slow_count_ = 0;
        break;
      }
      // 邻道连续清空 → 变道
      if (adjacent_lane_clear(in)) {
        if (++clear_count_ >= params_.clear_confirm_frames) {
          state_ = BehaviorKind::kOvertakeActive;
          overtaking_id_ = static_cast<uint32_t>(in.primary_lead_id);
          lost_count_ = 0;
        }
      } else {
        clear_count_ = 0;
      }
      break;

    case BehaviorKind::kOvertakeActive: {
      // 中止条件：邻道出现危险目标且自车仍基本在本车道内 → 退回跟车
      if (!adjacent_lane_clear(in) &&
          std::fabs(in.ego_lateral_m) < params_.abort_lat_limit_m) {
        state_ = BehaviorKind::kFollowLead;
        slow_count_ = 0;
        break;
      }
      // 超越判定：被超车辆已落后 pass_margin
      const ObjectLite* target = nullptr;
      for (const auto& o : in.objects) {
        if (o.id == overtaking_id_) {
          target = &o;
          break;
        }
      }
      // 回线前提：已横移到位目标车道（避免相对速度大时"半道超越"侧向余量不足）
      const double target_center =
          -static_cast<double>(params_.target_lane) * params_.lane_width_m;
      const bool lane_attained =
          std::fabs(in.ego_lateral_m - target_center) < params_.lane_attain_tol_m;
      if (target == nullptr) {
        // 目标丢失（驶出赛道/消失）连续数帧 → 直接回线
        if (++lost_count_ >= 10 && lane_attained) {
          state_ = BehaviorKind::kOvertakeReturn;
        }
      } else {
        lost_count_ = 0;
        if (lane_attained && target->lon_m < -params_.pass_margin_m) {
          state_ = BehaviorKind::kOvertakeReturn;
        }
      }
      break;
    }

    case BehaviorKind::kOvertakeReturn:
      if (std::fabs(in.ego_lateral_m) < params_.return_done_lat_m) {
        state_ = BehaviorKind::kLaneFollow;
      }
      break;

    case BehaviorKind::kStopping:
      break;
  }

  BehaviorOutput out;
  out.state = state_;
  out.target_speed_mps =
      state_ == BehaviorKind::kStopping ? 0.0 : params_.cruise_speed_mps;
  out.target_lane =
      state_ == BehaviorKind::kOvertakeActive ? params_.target_lane : 0;
  return out;
}

}  // namespace adas::planning
