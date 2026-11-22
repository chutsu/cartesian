#include <gtest/gtest.h>

#include "sim/SimPoseSampling.hpp"
#include "sim/SimCamera.hpp"

namespace cartesian {

static SimPoseSampling make_sampling() {
  SimPoseSampling s;
  s.seed = 1;
  s.num_poses = 20;
  s.min_dist = 0.5;
  s.max_dist = 1.5;
  s.max_tilt = 30.0;
  return s;
}

TEST(SimPoseSampling, sample_poses) {
  const auto s = make_sampling();
  const auto poses = s.sample_poses(0.6, 0.4);
  ASSERT_EQ(static_cast<int>(poses.size()), s.num_poses);

  const Vec3 center{0.3, 0.2, 0.0};
  for (const auto &pose : poses) {
    const Vec3 r_TC = tf_trans(pose.T_TC);
    ASSERT_NEAR((r_TC - center).norm(), pose.dist, 1e-9);
    ASSERT_GE(pose.dist, s.min_dist);
    ASSERT_LE(pose.dist, s.max_dist);
    ASSERT_LE(pose.tilt, s.max_tilt + 1e-9);
    ASSERT_GT(r_TC.z(), 0.0); // In front of the board
  }

  // Same seed gives same poses
  const auto poses2 = s.sample_poses(0.6, 0.4);
  ASSERT_EQ((poses[0].T_TC - poses2[0].T_TC).norm(), 0.0);
}

TEST(SimPoseSampling, sample_poses_spread) {
  const auto s = make_sampling();
  const auto camera = SimCamera{}.make_brown_conrady4();
  const double max_theta = 40.0;
  const auto poses = s.sample_poses_spread(0.6, 0.4, camera, max_theta);
  ASSERT_EQ(static_cast<int>(poses.size()), s.num_poses);

  // Board center must land inside the allowed field of view
  const Vec3 center{0.3, 0.2, 0.0};
  for (const auto &pose : poses) {
    const Vec3 p_C = tf_point(pose.T_TC.inverse(), center);
    ASSERT_LE(rad2deg(acos(p_C.normalized().z())), max_theta + 1e-6);
  }
}

} // namespace cartesian
