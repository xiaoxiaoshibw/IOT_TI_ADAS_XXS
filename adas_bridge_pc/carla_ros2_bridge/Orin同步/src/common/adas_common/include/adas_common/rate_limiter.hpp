// adas_common/rate_limiter.hpp — 变化率限幅器（支持不对称升/降速率）
#ifndef ADAS_COMMON__RATE_LIMITER_HPP_
#define ADAS_COMMON__RATE_LIMITER_HPP_

#include <algorithm>

namespace adas::common {

class RateLimiter {
 public:
  // rise_rate/fall_rate 均为正值（单位/秒）；fall_rate 缺省与 rise_rate 相同
  explicit RateLimiter(double rise_rate, double fall_rate = -1.0, double initial = 0.0)
      : rise_rate_(rise_rate),
        fall_rate_(fall_rate > 0.0 ? fall_rate : rise_rate),
        value_(initial) {}

  double update(double target, double dt) {
    if (dt <= 0.0) {
      return value_;
    }
    const double max_up = rise_rate_ * dt;
    const double max_down = fall_rate_ * dt;
    value_ += std::clamp(target - value_, -max_down, max_up);
    return value_;
  }

  void reset(double value) { value_ = value; }
  double value() const { return value_; }

  // 接管/切源瞬态时收紧速率（继承旧栈接管守护窗思想）
  void set_rates(double rise_rate, double fall_rate) {
    rise_rate_ = rise_rate;
    fall_rate_ = fall_rate;
  }

 private:
  double rise_rate_;
  double fall_rate_;
  double value_;
};

}  // namespace adas::common

#endif  // ADAS_COMMON__RATE_LIMITER_HPP_
