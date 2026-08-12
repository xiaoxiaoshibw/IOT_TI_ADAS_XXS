// 行为状态机实现
#include "adas_behavior_planner/behavior_core.hpp"

#include <cmath>

#include "adas_common/parameter_validation.hpp"

namespace adas::planning {

BehaviorCore::BehaviorCore(const BehaviorParams& params) : params_(params) {
  adas::common::require_positive("stop_sign_approach_m", params_.stop_sign_approach_m);
  adas::common::require_positive("stop_sign_stop_duration_s",
                                 params_.stop_sign_stop_duration_s);
  adas::common::require_positive("traffic_light_approach_m",
                                 params_.traffic_light_approach_m);
  adas::common::require_positive("junction_approach_m", params_.junction_approach_m);
}

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

  const auto sign_near = [&in](MapSignType type, double approach_m, bool red_only) {
    for (const auto& sign : in.map_signs) {
      if (sign.type != type || sign.distance_m < 0.0 || sign.distance_m > approach_m ||
          (red_only && !sign.traffic_light_red)) {
        continue;
      }
      return true;
    }
    return false;
  };
  const bool stop_sign_near =
      sign_near(MapSignType::kStopSign, params_.stop_sign_approach_m, false);
  const bool red_light_near =
      sign_near(MapSignType::kTrafficLight, params_.traffic_light_approach_m, true);
  const bool junction_near =
      sign_near(MapSignType::kJunction, params_.junction_approach_m, false);

  // Road signs have priority over the normal follow/overtake FSM.  A stop
  // line is considered reached at 1m; the explicit timer keeps the stop
  // duration deterministic in both ROS time and unit tests.
  if (state_ == BehaviorKind::kStoppingAtStop) {
    if (!stop_sign_near || in.now_s - stop_start_s_ >= params_.stop_sign_stop_duration_s) {
      state_ = BehaviorKind::kLaneFollow;
    }
  } else if (state_ == BehaviorKind::kApproachingStop) {
    if (!stop_sign_near) {
      state_ = BehaviorKind::kLaneFollow;
    } else {
      for (const auto& sign : in.map_signs) {
        if (sign.type == MapSignType::kStopSign && sign.distance_m <= 1.0) {
          state_ = BehaviorKind::kStoppingAtStop;
          stop_start_s_ = in.now_s;
          break;
        }
      }
    }
  } else if (state_ == BehaviorKind::kWaitingAtLight) {
    if (!red_light_near) state_ = BehaviorKind::kLaneFollow;
  } else if (state_ == BehaviorKind::kEnteringJunction) {
    state_ = BehaviorKind::kLaneFollow;
  } else if (stop_sign_near) {
    state_ = BehaviorKind::kApproachingStop;
  } else if (red_light_near) {
    state_ = BehaviorKind::kWaitingAtLight;
  } else if (junction_near) {
    state_ = BehaviorKind::kEnteringJunction;
  }

  const bool sign_state = state_ == BehaviorKind::kApproachingStop ||
                          state_ == BehaviorKind::kStoppingAtStop ||
                          state_ == BehaviorKind::kWaitingAtLight ||
                          state_ == BehaviorKind::kEnteringJunction;
  if (sign_state) {
    BehaviorOutput out;
    out.state = state_;
    out.target_speed_mps =
        (state_ == BehaviorKind::kStoppingAtStop || state_ == BehaviorKind::kWaitingAtLight)
            ? 0.0
            : params_.cruise_speed_mps;
    out.target_lane = 0;
    return out;
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
      if (params_.overtake_enabled && in.target_lane_available && lead_slow) {
        if (++slow_count_ >= params_.overtake_wait_frames) {
          state_ = BehaviorKind::kOvertakeWait;
          clear_count_ = 0;
        }
      } else {
        slow_count_ = 0;
      }
      break;

    case BehaviorKind::kOvertakeWait:
      // 地图/车道线明确表明目标邻道不存在或不允许变道：
      // 立即放弃超车，不能只凭"邻道无目标"就把路肩当车道。
      if (!in.target_lane_available) {
        state_ = BehaviorKind::kFollowLead;
        slow_count_ = clear_count_ = 0;
        break;
      }
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
      if (!in.target_lane_available &&
          std::fabs(in.ego_lateral_m) < params_.abort_lat_limit_m) {
        state_ = BehaviorKind::kFollowLead;
        slow_count_ = 0;
        break;
      }
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

    case BehaviorKind::kApproachingStop:
    case BehaviorKind::kStoppingAtStop:
    case BehaviorKind::kWaitingAtLight:
    case BehaviorKind::kEnteringJunction:
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
