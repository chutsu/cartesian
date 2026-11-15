#include "PoseManifold.hpp"

namespace cartesian {

bool PoseManifold::Plus(const double *x,
                        const double *delta,
                        double *x_plus_delta) const {
  // Form transform
  Mat4 T = tf(x);

  // Update rotation
  Vec3 dalpha{delta[3], delta[4], delta[5]};
  Quat q = tf_quat(T);
  Quat dq = quat_delta(dalpha);
  q = q * dq;
  q.normalize();

  // Update translation
  Vec3 dr{delta[0], delta[1], delta[2]};
  Vec3 r = tf_trans(T);
  r = r + dr;

  // Copy results to `x_plus_delta`
  x_plus_delta[0] = r.x();
  x_plus_delta[1] = r.y();
  x_plus_delta[2] = r.z();

  x_plus_delta[3] = q.x();
  x_plus_delta[4] = q.y();
  x_plus_delta[5] = q.z();
  x_plus_delta[6] = q.w();

  return true;
}

bool PoseManifold::PlusJacobian(const double *x, double *jacobian) const {
  // Jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);
  UNUSED(x);
  J.topRows<6>().setIdentity();
  J.bottomRows<1>().setZero();
  return true;
}

bool PoseManifold::Minus(const double *x,
                         const double *delta,
                         double *x_minus_delta) const {
  UNUSED(x);
  UNUSED(delta);
  UNUSED(x_minus_delta);
  return false;
}

bool PoseManifold::MinusJacobian(const double *x, double *jacobian) const {
  UNUSED(x);
  UNUSED(jacobian);
  return false;
}

int PoseManifold::AmbientSize() const { return 7; }

int PoseManifold::TangentSize() const { return 6; }

bool TiltManifold::Plus(const double *x,
                        const double *delta,
                        double *x_plus_delta) const {
  // Translation is unchanged
  x_plus_delta[0] = x[0];
  x_plus_delta[1] = x[1];
  x_plus_delta[2] = x[2];

  // Rotate about the world x and y axes only (no yaw)
  const Mat4 T = tf(x);
  Quat q = tf_quat(T);
  const Quat dq = quat_delta(Vec3{delta[0], delta[1], 0.0});
  q = dq * q;
  q.normalize();

  x_plus_delta[3] = q.x();
  x_plus_delta[4] = q.y();
  x_plus_delta[5] = q.z();
  x_plus_delta[6] = q.w();

  return true;
}

bool TiltManifold::PlusJacobian(const double *x, double *jacobian) const {
  // World-frame tilt [d0, d1, 0] expressed as a local-frame rotation:
  // dalpha_local = C_WT^T * [d0, d1, 0]
  Eigen::Map<Eigen::Matrix<double, 7, 2, Eigen::RowMajor>> J(jacobian);
  const Mat3 C = tf_rot(tf(x));
  J.setZero();
  J.block<3, 2>(3, 0) = C.transpose().leftCols<2>();
  return true;
}

bool TiltManifold::Minus(const double *x,
                         const double *delta,
                         double *x_minus_delta) const {
  UNUSED(x);
  UNUSED(delta);
  UNUSED(x_minus_delta);
  return false;
}

bool TiltManifold::MinusJacobian(const double *x, double *jacobian) const {
  UNUSED(x);
  UNUSED(jacobian);
  return false;
}

int TiltManifold::AmbientSize() const { return 7; }

int TiltManifold::TangentSize() const { return 2; }

} // namespace cartesian
