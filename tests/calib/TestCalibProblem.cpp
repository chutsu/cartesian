#include <gtest/gtest.h>

#include "calib/CalibProblem.hpp"

#define TEST_CONFIG TEST_DATA "/calib_camera.yaml"

namespace cartesian {

TEST(CalibProblem, load) {
  CalibProblem calib{TEST_CONFIG};
  ASSERT_EQ(calib.get_num_cameras(), 2);
}

} // namespace cartesian
