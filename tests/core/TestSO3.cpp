#include <gtest/gtest.h>

#include "core/SO3.hpp"

namespace cartesian {

TEST(SO3, exp_log_roundtrip) {
  const std::vector<Vec3> axes = {Vec3{1.0, 0.0, 0.0},
                                  Vec3{0.0, 1.0, 0.0},
                                  Vec3{0.0, 0.0, 1.0},
                                  Vec3{1.0, 2.0, 3.0}.normalized(),
                                  Vec3{-0.3, 0.8, -0.5}.normalized()};

  // Includes very small angles, where acos based log loses precision, and
  // angles close to pi
  const std::vector<double> angles = {1e-12, 1e-10, 1e-8, 1e-6, 1e-5, 1e-4,
                                      1e-3,  1e-2,  0.1,  0.5,  1.0,  2.0,
                                      3.0,   M_PI - 1e-2, M_PI - 1e-3,
                                      M_PI - 1e-4, M_PI - 1e-6};
  for (const auto &axis : axes) {
    for (const double angle : angles) {
      const Vec3 w = angle * axis;
      const Vec3 w_back = SO3::log(SO3::exp(w));

      // Relative error for the small angles, absolute error otherwise
      const double tol = std::max(1e-9 * angle, 1e-12);
      const double near_pi_tol = (M_PI - angle < 1e-3) ? 1e-6 : 0.0;
      ASSERT_NEAR((w - w_back).norm(), 0.0, std::max(tol, near_pi_tol))
          << "angle: " << angle << ", axis: " << axis.transpose();
    }
  }
}

TEST(SO3, log_identity) {
  const Vec3 w = SO3::log(Mat3::Identity());
  ASSERT_NEAR(w.norm(), 0.0, 1e-15);
}

TEST(SO3, log_pi) {
  // Rotation of exactly pi, the axis is only defined up to its sign
  const Vec3 axis = Vec3{1.0, -2.0, 0.5}.normalized();
  const Mat3 R = SO3::exp(M_PI * axis);
  const Vec3 w = SO3::log(R);
  ASSERT_NEAR(w.norm(), M_PI, 1e-6);
  ASSERT_NEAR((SO3::exp(w) - R).norm(), 0.0, 1e-6);
}

TEST(SO3, log_small_angle_precision) {
  // A tiny rotation must come back with a small relative error
  const Vec3 w{3e-7, -4e-7, 1e-7};
  const Vec3 w_back = SO3::log(SO3::exp(w));
  ASSERT_NEAR((w - w_back).norm() / w.norm(), 0.0, 1e-6);
}

TEST(SO3, right_jacobian_small_angle) {
  // exp(w + dw) = exp(w) exp(Jr(w) dw), compared with central differences. For
  // small w the closed form of Jr cancels badly, so check it is accurate there
  const double h = 1e-6;
  const Vec3 dir = Vec3{0.3, -0.5, 0.8}.normalized();
  for (const double angle : {0.0, 1e-9, 1e-6, 1e-4, 9e-3, 1.1e-2, 0.1, 1.0}) {
    const Vec3 w = angle * dir;
    const Mat3 R = SO3::exp(w);
    Mat3 J_num;
    for (int i = 0; i < 3; i++) {
      Vec3 dw = Vec3::Zero();
      dw[i] = h;
      const Vec3 fp = SO3::log(R.transpose() * SO3::exp(w + dw));
      const Vec3 fm = SO3::log(R.transpose() * SO3::exp(w - dw));
      J_num.col(i) = (fp - fm) / (2.0 * h);
    }
    ASSERT_NEAR((SO3::right_jacobian(w) - J_num).norm(), 0.0, 1e-8)
        << "angle: " << angle;
  }

  // Continuous across the switch between the series and the closed form. The
  // jacobians change by 0.5 * |skew(dw)| = 0.5 * sqrt(2) * |dw| over the step
  const Vec3 lo = 0.0099999 * dir;
  const Vec3 hi = 0.0100001 * dir;
  const double expected = 0.5 * std::sqrt(2.0) * (hi - lo).norm();
  const Mat3 dJ = SO3::right_jacobian(hi) - SO3::right_jacobian(lo);
  const Mat3 dJ_inv = SO3::right_jacobian_inv(hi) - SO3::right_jacobian_inv(lo);
  ASSERT_NEAR(dJ.norm(), expected, 1e-9);
  ASSERT_NEAR(dJ_inv.norm(), expected, 1e-9);
}

TEST(SO3, right_jacobian_inverse) {
  const std::vector<Vec3> ws = {Vec3{1e-9, 0.0, 0.0},
                                Vec3{1e-4, -2e-4, 3e-4},
                                Vec3{5e-3, 5e-3, 5e-3},
                                Vec3{0.1, -0.2, 0.3},
                                Vec3{1.0, 0.5, -1.5}};
  for (const auto &w : ws) {
    const Mat3 I = SO3::right_jacobian(w) * SO3::right_jacobian_inv(w);
    ASSERT_NEAR((I - Mat3::Identity()).norm(), 0.0, 1e-8);
  }
}

} // namespace cartesian
