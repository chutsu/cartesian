#include <gtest/gtest.h>

#include "ceres/TiltManifold.hpp"
#include "core/Core.hpp"

namespace cartesian {

static Vec7 setup_pose() {
  const Mat3 C = euler321(Vec3{0.3, -0.2, 0.7});
  const Vec3 r{1.0, 2.0, 3.0};
  return tf_vec(tf(C, r));
}

TEST(TiltManifold, sizes) {
  TiltManifold m;
  ASSERT_EQ(m.AmbientSize(), 7);
  ASSERT_EQ(m.TangentSize(), 2);
}

TEST(TiltManifold, plus_zero) {
  TiltManifold m;
  const Vec7 x = setup_pose();
  const double delta[2] = {0.0, 0.0};
  Vec7 x_plus;
  ASSERT_TRUE(m.Plus(x.data(), delta, x_plus.data()));
  ASSERT_NEAR((x - x_plus).norm(), 0.0, 1e-12);
}

TEST(TiltManifold, plus_fixes_translation_and_yaw) {
  TiltManifold m;
  const Vec7 x = setup_pose();
  const double delta[2] = {0.05, -0.03};
  Vec7 x_plus;
  ASSERT_TRUE(m.Plus(x.data(), delta, x_plus.data()));

  // Translation unchanged
  ASSERT_NEAR((tf_trans(tf(x)) - tf_trans(tf(x_plus))).norm(), 0.0, 1e-12);

  // Unit quaternion
  ASSERT_NEAR(x_plus.tail<4>().norm(), 1.0, 1e-12);

  // Rotation changed
  const Mat3 C = tf_rot(tf(x));
  const Mat3 C_plus = tf_rot(tf(x_plus));
  ASSERT_GT((C - C_plus).norm(), 1e-3);

  // The applied rotation dC = C_plus * C^T is a pure world x-y rotation, so it
  // has no component about the world z axis (gravity), i.e. yaw is unchanged.
  const Mat3 dC = C_plus * C.transpose();
  const Eigen::AngleAxisd aa{dC};
  ASSERT_NEAR((aa.angle() * aa.axis()).z(), 0.0, 1e-12);
}

TEST(TiltManifold, plus_jacobian) {
  TiltManifold m;
  const Vec7 x = setup_pose();

  // Analytical
  Eigen::Matrix<double, 7, 2, Eigen::RowMajor> J;
  ASSERT_TRUE(m.PlusJacobian(x.data(), J.data()));

  // Numerical: express the tilt update as a local rotation C^T * C_plus
  const double h = 1e-6;
  const Mat3 C = tf_rot(tf(x));
  for (int i = 0; i < 2; ++i) {
    double delta[2] = {0.0, 0.0};
    delta[i] = h;
    Vec7 x_plus;
    ASSERT_TRUE(m.Plus(x.data(), delta, x_plus.data()));

    const Mat3 dC = C.transpose() * tf_rot(tf(x_plus));
    const Eigen::AngleAxisd aa{dC};
    const Vec3 dalpha = aa.angle() * aa.axis() / h;

    ASSERT_NEAR((J.block<3, 1>(0, i)).norm(), 0.0, 1e-12); // translation
    ASSERT_NEAR((J.block<3, 1>(3, i) - dalpha).norm(), 0.0, 1e-5);
    ASSERT_NEAR(J(6, i), 0.0, 1e-12); // qw
  }
}

} // namespace cartesian
