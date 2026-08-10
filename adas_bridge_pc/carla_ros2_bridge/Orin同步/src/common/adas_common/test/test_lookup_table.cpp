// adas_common 查表单测
#include <gtest/gtest.h>

#include "adas_common/lookup_table.hpp"

namespace ac = adas::common;

TEST(LookupTable1D, InterpolatesLinearly) {
  ac::LookupTable1D t({0.0, 10.0, 20.0}, {1.0, 2.0, 4.0});
  EXPECT_NEAR(t(5.0), 1.5, 1e-9);
  EXPECT_NEAR(t(15.0), 3.0, 1e-9);
}

TEST(LookupTable1D, ClampsOutside) {
  ac::LookupTable1D t({0.0, 10.0}, {1.0, 2.0});
  EXPECT_NEAR(t(-5.0), 1.0, 1e-9);
  EXPECT_NEAR(t(50.0), 2.0, 1e-9);
}

TEST(LookupTable1D, RejectsIllegalTable) {
  EXPECT_THROW(ac::LookupTable1D({}, {}), std::invalid_argument);
  EXPECT_THROW(ac::LookupTable1D({0.0, 0.0}, {1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW(ac::LookupTable1D({0.0, 1.0}, {1.0}), std::invalid_argument);
}
