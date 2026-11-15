#include <gtest/gtest.h>

#include "sim/SimImu.hpp"

namespace cartesian {

TEST(SimImu, construct) {
  SimImu sim;
  sim.save("/tmp/sim.csv");
}

} // namespace cartesian
