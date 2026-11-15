#include <gtest/gtest.h>

#include "sim/SimImu.hpp"

namespace cartesian {

TEST(Sim, construct) {
  SimImu sim;
  sim.save("/tmp/sim.csv");
}

} // namespace cartesian
