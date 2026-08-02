// adas_common/geometry.hpp — 平面几何与轨迹查询工具（纯函数，无状态）
#ifndef ADAS_COMMON__GEOMETRY_HPP_
#define ADAS_COMMON__GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "adas_common/types.hpp"

namespace adas::common {

// M_PI 非标准 C++，为可移植性自带常量
inline constexpr double kPi = 3.14159265358979323846;

// 角度归一化到 [-pi, pi]
inline double normalize_angle(double angle) {
  const double two_pi = 2.0 * kPi;
  double a = std::fmod(angle + kPi, two_pi);
  if (a < 0.0) {
    a += two_pi;
  }
  return a - kPi;
}

inline double distance2d(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

// 轨迹上离 (x,y) 最近的点索引；空轨迹返回 0（调用方须先判空）
inline std::size_t find_nearest_index(const Trajectory& traj, double x, double y) {
  std::size_t best = 0;
  double best_d2 = 1e300;
  for (std::size_t i = 0; i < traj.size(); ++i) {
    const double dx = traj[i].x - x;
    const double dy = traj[i].y - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

// 从 start_idx 沿轨迹前向累计弧长 arclength，返回线性插值点。
// 超出轨迹末端时钳制到最后一点（速度/曲率同样取末点）。
inline TrajPoint point_at_arclength(const Trajectory& traj, std::size_t start_idx,
                                    double arclength) {
  if (traj.empty()) {
    return TrajPoint{};
  }
  if (start_idx >= traj.size() - 1 || arclength <= 0.0) {
    if (start_idx >= traj.size()) {
      return traj.back();
    }
    if (arclength <= 0.0) {
      return traj[start_idx];
    }
  }
  double remaining = arclength;
  for (std::size_t i = start_idx; i + 1 < traj.size(); ++i) {
    const TrajPoint& a = traj[i];
    const TrajPoint& b = traj[i + 1];
    const double seg = distance2d(a.x, a.y, b.x, b.y);
    if (seg < 1e-9) {
      continue;
    }
    if (remaining <= seg) {
      const double t = remaining / seg;
      TrajPoint out;
      out.x = a.x + t * (b.x - a.x);
      out.y = a.y + t * (b.y - a.y);
      out.yaw = normalize_angle(a.yaw + t * normalize_angle(b.yaw - a.yaw));
      out.velocity_mps = a.velocity_mps + t * (b.velocity_mps - a.velocity_mps);
      out.acceleration_mps2 = a.acceleration_mps2 + t * (b.acceleration_mps2 - a.acceleration_mps2);
      out.curvature = a.curvature + t * (b.curvature - a.curvature);
      out.time_from_start_s = a.time_from_start_s + t * (b.time_from_start_s - a.time_from_start_s);
      return out;
    }
    remaining -= seg;
  }
  return traj.back();
}

// (x,y) 相对轨迹最近段的有符号横向偏移：轨迹前进方向左侧为正
inline double signed_lateral_offset(const Trajectory& traj, double x, double y) {
  if (traj.size() < 2) {
    return 0.0;
  }
  std::size_t i = find_nearest_index(traj, x, y);
  // 取最近点所在的前向段（末点退一段）
  if (i >= traj.size() - 1) {
    i = traj.size() - 2;
  }
  const TrajPoint& a = traj[i];
  const TrajPoint& b = traj[i + 1];
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = x - a.x;
  const double wy = y - a.y;
  const double seg_len = std::hypot(vx, vy);
  if (seg_len < 1e-9) {
    return 0.0;
  }
  // 二维叉积 / 段长 = 有符号距离（左正）
  return (vx * wy - vy * wx) / seg_len;
}

}  // namespace adas::common

#endif  // ADAS_COMMON__GEOMETRY_HPP_
