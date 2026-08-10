#ifndef ADAS_COMMON__TIMING_MONITOR_HPP_
#define ADAS_COMMON__TIMING_MONITOR_HPP_

#include <algorithm>
#include <chrono>
#include <cstddef>

namespace adas::common {

class TimingMonitor {
 public:
  struct Snapshot {
    double last_ms{0.0};
    double average_ms{0.0};
    double max_ms{0.0};
    double budget_ms{0.0};
    std::size_t samples{0U};
    bool warning{false};
    bool error{false};
  };

  explicit TimingMonitor(double budget_ms = 0.0) : budget_ms_(budget_ms) {}

  void set_budget_ms(double budget_ms) { budget_ms_ = budget_ms; }

  void reset() {
    last_ms_ = 0.0;
    total_ms_ = 0.0;
    max_ms_ = 0.0;
    samples_ = 0U;
  }

  void record_ms(double elapsed_ms) {
    last_ms_ = std::max(0.0, elapsed_ms);
    total_ms_ += last_ms_;
    max_ms_ = std::max(max_ms_, last_ms_);
    ++samples_;
  }

  template <typename Rep, typename Period>
  void record(std::chrono::duration<Rep, Period> elapsed) {
    record_ms(std::chrono::duration<double, std::milli>(elapsed).count());
  }

  Snapshot snapshot() const {
    const double average = samples_ == 0U ? 0.0 : total_ms_ / static_cast<double>(samples_);
    const bool has_budget = budget_ms_ > 0.0;
    return Snapshot{last_ms_, average, max_ms_, budget_ms_, samples_,
                    has_budget && last_ms_ >= budget_ms_ * 0.8,
                    has_budget && last_ms_ >= budget_ms_};
  }

 private:
  double budget_ms_{0.0};
  double last_ms_{0.0};
  double total_ms_{0.0};
  double max_ms_{0.0};
  std::size_t samples_{0U};
};

class ScopedTimingSample {
 public:
  explicit ScopedTimingSample(TimingMonitor & monitor)
      : monitor_(monitor), started_(Clock::now()) {}

  ~ScopedTimingSample() { monitor_.record(Clock::now() - started_); }

  ScopedTimingSample(const ScopedTimingSample &) = delete;
  ScopedTimingSample & operator=(const ScopedTimingSample &) = delete;

 private:
  using Clock = std::chrono::steady_clock;
  TimingMonitor & monitor_;
  Clock::time_point started_;
};

}  // namespace adas::common

#endif  // ADAS_COMMON__TIMING_MONITOR_HPP_
