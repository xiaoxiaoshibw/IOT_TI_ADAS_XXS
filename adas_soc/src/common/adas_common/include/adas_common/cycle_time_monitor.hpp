#ifndef ADAS_COMMON__CYCLE_TIME_MONITOR_HPP_
#define ADAS_COMMON__CYCLE_TIME_MONITOR_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace adas::common {

// Measures real steady-clock cycle time while bounding the value consumed by
// controllers and safety filters. A scheduler stall must not become an
// unbounded steering, jerk, or integrator step.
class CycleTimeMonitor {
 public:
  struct Snapshot {
    double nominal_s{0.0};
    double last_raw_s{0.0};
    double last_used_s{0.0};
    double average_raw_s{0.0};
    double min_raw_s{0.0};
    double max_raw_s{0.0};
    double max_abs_jitter_s{0.0};
    std::size_t samples{0U};
    std::size_t clamped_samples{0U};
  };

  explicit CycleTimeMonitor(double rate_hz = 50.0, double min_ratio = 0.5,
                            double max_ratio = 2.0) {
    configure(rate_hz, min_ratio, max_ratio);
  }

  void configure(double rate_hz, double min_ratio = 0.5, double max_ratio = 2.0) {
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      throw std::invalid_argument("rate_hz must be finite and positive");
    }
    if (!std::isfinite(min_ratio) || !std::isfinite(max_ratio) || min_ratio <= 0.0 ||
        max_ratio < min_ratio) {
      throw std::invalid_argument("cycle clamp ratios are invalid");
    }
    nominal_s_ = 1.0 / rate_hz;
    min_s_ = nominal_s_ * min_ratio;
    max_s_ = nominal_s_ * max_ratio;
    reset();
  }

  void reset() {
    initialized_ = false;
    last_now_s_ = 0.0;
    last_raw_s_ = nominal_s_;
    last_used_s_ = nominal_s_;
    total_raw_s_ = 0.0;
    min_raw_s_ = std::numeric_limits<double>::infinity();
    max_raw_s_ = 0.0;
    max_abs_jitter_s_ = 0.0;
    samples_ = 0U;
    clamped_samples_ = 0U;
  }

  double tick() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return tick_seconds(std::chrono::duration<double>(now).count());
  }

  double tick_seconds(double now_s) {
    if (!std::isfinite(now_s)) {
      ++clamped_samples_;
      last_used_s_ = nominal_s_;
      return last_used_s_;
    }
    if (!initialized_) {
      initialized_ = true;
      last_now_s_ = now_s;
      last_raw_s_ = nominal_s_;
      last_used_s_ = nominal_s_;
      return last_used_s_;
    }
    const double raw = now_s - last_now_s_;
    last_now_s_ = now_s;
    if (!std::isfinite(raw) || raw <= 0.0) {
      ++clamped_samples_;
      last_raw_s_ = raw;
      last_used_s_ = nominal_s_;
      return last_used_s_;
    }
    last_raw_s_ = raw;
    last_used_s_ = std::clamp(raw, min_s_, max_s_);
    if (last_used_s_ != raw) ++clamped_samples_;
    total_raw_s_ += raw;
    min_raw_s_ = std::min(min_raw_s_, raw);
    max_raw_s_ = std::max(max_raw_s_, raw);
    max_abs_jitter_s_ = std::max(max_abs_jitter_s_, std::fabs(raw - nominal_s_));
    ++samples_;
    return last_used_s_;
  }

  Snapshot snapshot() const {
    const double average = samples_ == 0U ? 0.0 : total_raw_s_ / samples_;
    const double minimum = samples_ == 0U ? 0.0 : min_raw_s_;
    return Snapshot{nominal_s_, last_raw_s_, last_used_s_, average, minimum, max_raw_s_,
                    max_abs_jitter_s_, samples_, clamped_samples_};
  }

 private:
  double nominal_s_{0.02};
  double min_s_{0.01};
  double max_s_{0.04};
  bool initialized_{false};
  double last_now_s_{0.0};
  double last_raw_s_{0.02};
  double last_used_s_{0.02};
  double total_raw_s_{0.0};
  double min_raw_s_{std::numeric_limits<double>::infinity()};
  double max_raw_s_{0.0};
  double max_abs_jitter_s_{0.0};
  std::size_t samples_{0U};
  std::size_t clamped_samples_{0U};
};

}  // namespace adas::common

#endif  // ADAS_COMMON__CYCLE_TIME_MONITOR_HPP_

