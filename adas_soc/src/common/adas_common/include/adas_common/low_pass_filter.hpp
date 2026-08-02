// adas_common/low_pass_filter.hpp — 一阶低通
#ifndef ADAS_COMMON__LOW_PASS_FILTER_HPP_
#define ADAS_COMMON__LOW_PASS_FILTER_HPP_

namespace adas::common {

class LowPassFilter {
 public:
  // tau: 时间常数 [s]。tau<=0 时直通。
  explicit LowPassFilter(double tau) : tau_(tau) {}

  double update(double input, double dt) {
    if (!initialized_) {
      value_ = input;
      initialized_ = true;
      return value_;
    }
    if (tau_ <= 0.0 || dt <= 0.0) {
      value_ = input;
      return value_;
    }
    const double alpha = dt / (tau_ + dt);
    value_ += alpha * (input - value_);
    return value_;
  }

  void reset() { initialized_ = false; }
  void reset(double value) {
    value_ = value;
    initialized_ = true;
  }
  double value() const { return value_; }

 private:
  double tau_;
  double value_{0.0};
  bool initialized_{false};
};

}  // namespace adas::common

#endif  // ADAS_COMMON__LOW_PASS_FILTER_HPP_
