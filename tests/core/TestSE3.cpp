#include <gtest/gtest.h>

#include "core/SE3.hpp"
#include "core/SO3.hpp"

namespace cartesian {

static std::vector<Vec6> test_tangents() {
  std::vector<Vec6> xis;
  Vec6 xi;
  xi << 0.1, -0.2, 0.3, 0.0, 0.0, 0.0;
  xis.push_back(xi);
  xi << 1.0, 2.0, -3.0, 1e-9, -2e-9, 1e-9; // Near zero rotation
  xis.push_back(xi);
  xi << 0.5, -1.0, 2.0, 0.3, -0.2, 0.1;
  xis.push_back(xi);
  xi << -1.0, 0.4, 0.7, 1.0, -1.5, 0.8;
  xis.push_back(xi);
  xi << 2.0, -3.0, 1.0, 2.5, 0.5, -1.0; // Large rotation
  xis.push_back(xi);
  return xis;
}

TEST(SE3, exp_log) {
  for (const auto &xi : test_tangents()) {
    const Mat4 T = SE3::exp(xi);

    // Valid transform
    const Mat3 R = T.block<3, 3>(0, 0);
    ASSERT_NEAR((R.transpose() * R - Mat3::Identity()).norm(), 0.0, 1e-9);
    ASSERT_NEAR(R.determinant(), 1.0, 1e-9);
    ASSERT_NEAR(T(3, 3), 1.0, 1e-12);

    // Round trip
    const Vec6 xi_back = SE3::log(T);
    ASSERT_NEAR((xi - xi_back).norm(), 0.0, 1e-8);
  }
}

TEST(SE3, exp_identity_and_translation) {
  // Zero tangent is the identity
  const Mat4 I = SE3::exp(Vec6::Zero());
  ASSERT_NEAR((I - Mat4::Identity()).norm(), 0.0, 1e-12);

  // Pure translation
  Vec6 xi = Vec6::Zero();
  xi << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0;
  const Mat4 T = SE3::exp(xi);
  const Vec3 t = T.block<3, 1>(0, 3);
  const Mat3 R = T.block<3, 3>(0, 0);
  ASSERT_NEAR((t - Vec3{1.0, 2.0, 3.0}).norm(), 0.0, 1e-12);
  ASSERT_NEAR((R - Mat3::Identity()).norm(), 0.0, 1e-12);

  // Pure rotation, matches SO3
  const Vec3 w{0.3, -0.2, 0.5};
  xi << 0.0, 0.0, 0.0, w.x(), w.y(), w.z();
  const Mat4 T_rot = SE3::exp(xi);
  const Mat3 R_rot = T_rot.block<3, 3>(0, 0);
  ASSERT_NEAR((R_rot - SO3::exp(w)).norm(), 0.0, 1e-12);
  const Vec3 t_rot = T_rot.block<3, 1>(0, 3);
  ASSERT_NEAR(t_rot.norm(), 0.0, 1e-12);
}

TEST(SE3, adjoint) {
  const Mat4 T = SE3::exp(test_tangents()[3]);
  for (const auto &xi : test_tangents()) {
    // T exp(xi) T^-1 = exp(Ad(T) xi)
    const Mat4 lhs = T * SE3::exp(xi) * T.inverse();
    const Mat4 rhs = SE3::exp(SE3::adjoint(T) * xi);
    ASSERT_NEAR((lhs - rhs).norm(), 0.0, 1e-8);
  }
}

TEST(SE3, right_jacobian) {
  // exp(xi + dxi) = exp(xi) exp(Jr(xi) dxi), to first order. Compare against
  // central differences of log(exp(xi)^-1 exp(xi + dxi))
  const double h = 1e-6;
  for (const auto &xi : test_tangents()) {
    const Mat4 T = SE3::exp(xi);

    Mat<6, 6> J_num;
    for (int i = 0; i < 6; i++) {
      Vec6 dxi = Vec6::Zero();
      dxi[i] = h;
      const Vec6 fp = SE3::log(T.inverse() * SE3::exp(xi + dxi));
      const Vec6 fm = SE3::log(T.inverse() * SE3::exp(xi - dxi));
      J_num.col(i) = (fp - fm) / (2.0 * h);
    }

    const Mat<6, 6> J = SE3::right_jacobian(xi);
    ASSERT_NEAR((J - J_num).norm(), 0.0, 1e-5);
  }
}

TEST(SE3, jacobian_inverse) {
  for (const auto &xi : test_tangents()) {
    const Mat<6, 6> I = Mat<6, 6>::Identity();
    ASSERT_NEAR((SE3::right_jacobian(xi) * SE3::right_jacobian_inv(xi) - I)
                    .norm(),
                0.0,
                1e-8);
    ASSERT_NEAR((SE3::left_jacobian(xi) * SE3::left_jacobian_inv(xi) - I)
                    .norm(),
                0.0,
                1e-8);
  }
}

TEST(SE3, left_right_jacobian) {
  // Jl(xi) = Ad(exp(xi)) Jr(xi)
  for (const auto &xi : test_tangents()) {
    const Mat<6, 6> lhs = SE3::left_jacobian(xi);
    const Mat<6, 6> rhs = SE3::adjoint(SE3::exp(xi)) * SE3::right_jacobian(xi);
    ASSERT_NEAR((lhs - rhs).norm(), 0.0, 1e-8);
  }
}

} // namespace cartesian
