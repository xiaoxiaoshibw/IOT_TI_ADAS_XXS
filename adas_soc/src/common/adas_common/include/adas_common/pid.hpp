// adas_common/pid.hpp — 带积分限幅与输出限幅的 PID
#ifndef ADAS_COMMON__PID_HPP_
#define ADAS_COMMON__PID_HPP_

#include <algorithm>

namespace adas::common {

class Pid {
 public:
  struct Gains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
  };

  Pid(const Gains& gains, double out_min, double out_max, double integral_min,
      double integral_max)
      : gains_(gains),
        out_min_(out_min),
        out_max_(out_max),
        integral_min_(integral_min),
        integral_max_(integral_max) {}

  // error = 目标 - 实际；dt 秒。首拍不产生微分项。
  double update(double error, double dt) {
    if (dt <= 0.0) {
      return last_output_;
    }
    integral_ = std::clamp(integral_ + error * dt, integral_min_, integral_max_);
    double derivative = 0.0;
    if (has_prev_) {
      derivative = (error - prev_error_) / dt;
    }
    prev_error_ = error;
    has_prev_ = true;
    const double raw =
        gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    last_output_ = std::clamp(raw, out_min_, out_max_);
    // 抗积分饱和：输出饱和且误差同向时回退本拍积分
    if (raw != last_output_ && ((raw > out_max_ && error > 0.0) || (raw < out_min_ && error < 0.0))) {
      integral_ = std::clamp(integral_ - error * dt, integral_min_, integral_max_);
    }
    return last_output_;
  }

  void reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    has_prev_ = false;
    last_output_ = 0.0;
  }

  double integral() const { return integral_; }

 private:
  Gains gains_;
  double out_min_;
  double out_max_;
  double integral_min_;
  double integral_max_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool has_prev_{false};
  double last_output_{0.0};
};

}  // namespace adas::common

#endif  // ADAS_COMMON__PID_HPP_
