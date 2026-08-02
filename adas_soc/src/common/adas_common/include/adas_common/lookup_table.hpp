// adas_common/lookup_table.hpp — 一维线性插值查表（端点外钳制）
// 用途：gate 的速度相关限幅曲线、车辆接口的加速度→油门/制动标定表
#ifndef ADAS_COMMON__LOOKUP_TABLE_HPP_
#define ADAS_COMMON__LOOKUP_TABLE_HPP_

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace adas::common {

class LookupTable1D {
 public:
  // xs 必须严格递增且与 ys 等长（至少 1 点）；违反抛 invalid_argument
  //（安全关键参数的启动期审计：非法表直接拒绝构造）
  LookupTable1D(std::vector<double> xs, std::vector<double> ys)
      : xs_(std::move(xs)), ys_(std::move(ys)) {
    if (xs_.empty() || xs_.size() != ys_.size()) {
      throw std::invalid_argument("LookupTable1D: xs/ys 为空或长度不等");
    }
    for (std::size_t i = 1; i < xs_.size(); ++i) {
      if (xs_[i] <= xs_[i - 1]) {
        throw std::invalid_argument("LookupTable1D: xs 必须严格递增");
      }
    }
  }

  double operator()(double x) const {
    if (x <= xs_.front()) {
      return ys_.front();
    }
    if (x >= xs_.back()) {
      return ys_.back();
    }
    // 线性扫描：表规模都很小（<10 点），无需二分
    std::size_t i = 1;
    while (xs_[i] < x) {
      ++i;
    }
    const double t = (x - xs_[i - 1]) / (xs_[i] - xs_[i - 1]);
    return ys_[i - 1] + t * (ys_[i] - ys_[i - 1]);
  }

 private:
  std::vector<double> xs_;
  std::vector<double> ys_;
};

}  // namespace adas::common

#endif  // ADAS_COMMON__LOOKUP_TABLE_HPP_
