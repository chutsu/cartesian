#include "SE3.hpp"
#include "SO3.hpp"

namespace cartesian {
namespace SE3 {

// Left jacobian of SO(3), J_l(w) = J_r(-w)
static Mat3 so3_left_jacobian(const Vec3 &w) {
  return SO3::right_jacobian(-w);
}

// Inverse of the left jacobian of SO(3)
static Mat3 so3_left_jacobian_inv(const Vec3 &w) {
  return SO3::right_jacobian_inv(-w);
}

// The coupling block Q of the left jacobian of SE(3)
//
//   J_l(xi) = [J_l(phi)  Q(rho, phi)]
//             [0         J_l(phi)   ]
static Mat3 left_jacobian_q(const Vec3 &rho, const Vec3 &phi) {
  const double theta = phi.norm();
  const Mat3 px = skew(phi);
  const Mat3 rx = skew(rho);
  const Mat3 pxrx = px * rx;
  const Mat3 rxpx = rx * px;
  const Mat3 pxrxpx = pxrx * px;

  // Coefficients, with their series expansions for small angles
  double c1, c2, c3;
  if (theta < 1e-3) {
    const double t2 = theta * theta;
    c1 = 1.0 / 6.0 - t2 / 120.0;
    c2 = -1.0 / 24.0 + t2 / 720.0;
    c3 = -1.0 / 120.0 + t2 / 2520.0;
  } else {
    const double t2 = theta * theta;
    const double t3 = t2 * theta;
    const double t4 = t2 * t2;
    const double t5 = t4 * theta;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    c1 = (theta - s) / t3;
    c2 = (t2 + 2.0 * c - 2.0) / (2.0 * t4);
    c3 = (2.0 * theta - 3.0 * s + theta * c) / (2.0 * t5);
  }

  return 0.5 * rx                                               //
         + c1 * (pxrx + rxpx + pxrxpx)                          //
         + c2 * (px * pxrx + rxpx * px - 3.0 * pxrxpx)          //
         + c3 * (pxrxpx * px + px * pxrxpx);
}

Vec6 log(const Mat4 &T) {
  const Mat3 R = T.block<3, 3>(0, 0);
  const Vec3 t = T.block<3, 1>(0, 3);

  const Vec3 phi = SO3::log(R);
  const Vec3 rho = so3_left_jacobian_inv(phi) * t;

  Vec6 xi;
  xi << rho, phi;
  return xi;
}

Mat4 exp(const Vec6 &xi) {
  const Vec3 rho = xi.head<3>();
  const Vec3 phi = xi.tail<3>();

  Mat4 T = Mat4::Identity();
  T.block<3, 3>(0, 0) = SO3::exp(phi);
  T.block<3, 1>(0, 3) = so3_left_jacobian(phi) * rho;
  return T;
}

Mat<6, 6> adjoint(const Mat4 &T) {
  const Mat3 R = T.block<3, 3>(0, 0);
  const Vec3 t = T.block<3, 1>(0, 3);

  Mat<6, 6> Ad = Mat<6, 6>::Zero();
  Ad.block<3, 3>(0, 0) = R;
  Ad.block<3, 3>(0, 3) = skew(t) * R;
  Ad.block<3, 3>(3, 3) = R;
  return Ad;
}

Mat<6, 6> left_jacobian(const Vec6 &xi) {
  const Vec3 rho = xi.head<3>();
  const Vec3 phi = xi.tail<3>();
  const Mat3 J = so3_left_jacobian(phi);

  Mat<6, 6> Jl = Mat<6, 6>::Zero();
  Jl.block<3, 3>(0, 0) = J;
  Jl.block<3, 3>(0, 3) = left_jacobian_q(rho, phi);
  Jl.block<3, 3>(3, 3) = J;
  return Jl;
}

Mat<6, 6> left_jacobian_inv(const Vec6 &xi) {
  const Vec3 rho = xi.head<3>();
  const Vec3 phi = xi.tail<3>();
  const Mat3 J_inv = so3_left_jacobian_inv(phi);
  const Mat3 Q = left_jacobian_q(rho, phi);

  Mat<6, 6> Jl_inv = Mat<6, 6>::Zero();
  Jl_inv.block<3, 3>(0, 0) = J_inv;
  Jl_inv.block<3, 3>(0, 3) = -J_inv * Q * J_inv;
  Jl_inv.block<3, 3>(3, 3) = J_inv;
  return Jl_inv;
}

Mat<6, 6> right_jacobian(const Vec6 &xi) { return left_jacobian(-xi); }

Mat<6, 6> right_jacobian_inv(const Vec6 &xi) { return left_jacobian_inv(-xi); }

} // namespace SE3
} // namespace cartesian
