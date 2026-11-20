#include <gtest/gtest.h>

#include "ceres/ImuError.hpp"
#include "imu/ImuPreintegrate.hpp"

namespace cartesian {

static ImuParams test_params() {
  ImuParams imu_params;
  imu_params.noise_acc = 0.08;
  imu_params.noise_gyr = 0.004;
  imu_params.noise_ba = 0.00004;
  imu_params.noise_bg = 2.0e-6;
  return imu_params;
}

static ImuBuffer test_buffer(const double rate,
                             const double duration,
                             const Vec3 &w) {
  ImuBuffer imu_buffer;
  const int n = static_cast<int>(rate * duration) + 1;
  for (int k = 0; k < n; ++k) {
    imu_buffer.add(sec2ts(k / rate), Vec3{0.1, 0.2, 9.81}, w);
  }
  return imu_buffer;
}

TEST(ImuPreintegrate, relative_rotation_stays_normalized) {
  // A constant rotation of 5.7 rad/s for 1 s (200 Hz) is a rotation of 5.7 rad
  const Vec3 w{5.0, 2.5, 1.0};
  const ImuPreintegrate pint{test_params(), test_buffer(200.0, 1.0, w)};
  ASSERT_NEAR(pint.dq.norm(), 1.0, 1e-12);

  const Quat expected = quat_delta(w * 1.0);
  ASSERT_LT((pint.dq.toRotationMatrix() - expected.toRotationMatrix()).norm(),
            1e-9);
}

TEST(ImuPreintegrate, covariance_does_not_depend_on_imu_rate) {
  const Vec3 w{0.1, 0.05, 0.02};
  const ImuPreintegrate pint_200{test_params(), test_buffer(200.0, 0.2, w)};
  const ImuPreintegrate pint_1000{test_params(), test_buffer(1000.0, 0.2, w)};
  for (int i = 0; i < 15; ++i) {
    const double ratio = pint_1000.state_P(i, i) / pint_200.state_P(i, i);
    ASSERT_GT(ratio, 0.7) << i;
    ASSERT_LT(ratio, 1.3) << i;
  }
}

TEST(ImuPreintegrate, agrees_with_imu_error) {
  // Both preintegrate the same way: same relative motion and covariance
  const auto imu_params = test_params();
  const auto imu_buffer = test_buffer(200.0, 0.5, Vec3{1.0, 0.5, 0.2});
  const ImuPreintegrate pint{imu_params, imu_buffer};

  VecX pose = VecX::Zero(7);
  pose(6) = 1.0;
  Eigen::Vector<double, 9> sb = Eigen::Vector<double, 9>::Zero();
  const auto block = ImuError::create(
      imu_params, imu_buffer, pose.data(), sb.data(), pose.data(), sb.data());

  ASSERT_LT((pint.dq.coeffs() - block->get_relative_rotation().coeffs()).norm(),
            1e-12);
  ASSERT_LT((pint.dr - block->get_relative_position()).norm(), 1e-12);
  ASSERT_LT((pint.dv - block->get_relative_velocity()).norm(), 1e-12);
  ASSERT_LT((pint.state_P - block->get_matrix_p()).norm() /
                block->get_matrix_p().norm(),
            1e-9);
}

} // namespace cartesian
