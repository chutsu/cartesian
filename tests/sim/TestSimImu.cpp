#include <gtest/gtest.h>

#include "sim/SimCircle.hpp"
#include "sim/SimImu.hpp"

namespace cartesian {

TEST(SimImu, construct) {
  SimCircle circle;
  SimImu sim(circle);
  sim.save("/tmp/sim.csv");
}

} // namespace cartesian
