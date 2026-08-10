#ifndef ADAS_COMMON__PARAMETER_VALIDATION_HPP_
#define ADAS_COMMON__PARAMETER_VALIDATION_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace adas::common {

inline void require_finite(const std::string & name, double value) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
}

inline void require_positive(const std::string & name, double value) {
  require_finite(name, value);
  if (value <= 0.0) {
    throw std::invalid_argument(name + " must be > 0");
  }
}

inline void require_nonnegative(const std::string & name, double value) {
  require_finite(name, value);
  if (value < 0.0) {
    throw std::invalid_argument(name + " must be >= 0");
  }
}

inline void require_range(const std::string & name, double value, double minimum,
                          double maximum) {
  require_finite(name, value);
  if (value < minimum || value > maximum) {
    throw std::invalid_argument(name + " must be in [" + std::to_string(minimum) + ", " +
                                std::to_string(maximum) + "]");
  }
}

inline void require_timeout_exceeds_period(const std::string & timeout_name, double timeout_s,
                                           const std::string & rate_name, double rate_hz) {
  require_positive(timeout_name, timeout_s);
  require_positive(rate_name, rate_hz);
  const double period_s = 1.0 / rate_hz;
  if (timeout_s <= period_s) {
    throw std::invalid_argument(timeout_name + " must exceed the period implied by " + rate_name);
  }
}

}  // namespace adas::common

#endif  // ADAS_COMMON__PARAMETER_VALIDATION_HPP_
