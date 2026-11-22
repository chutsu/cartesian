#include "SO3.hpp"

namespace cartesian {
namespace SO3 {

Vec3 log(const Mat3 &R) {
  // The skew-symmetric part of R is sin(theta) * axis, and the trace gives
  // cos(theta). Using atan2 on both stays accurate for tiny angles, where
  // acos(cos(theta)) loses all precision, and is well conditioned near pi
  const Vec3 v{R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};
  const Vec3 sin_axis = 0.5 * v;
  const double sin_theta = sin_axis.norm();
  const double cos_theta = 0.5 * (R.trace() - 1.0);
  const double theta = std::atan2(sin_theta, cos_theta);

  // Small angle, log(R) = theta / sin(theta) * sin_axis ~ (1 + theta^2 / 6) v
  if (theta < 1e-4) {
    return (1.0 + theta * theta / 6.0) * sin_axis;
  }

  // Away from pi
  if (M_PI - theta > 1e-3) {
    return (theta / sin_theta) * sin_axis;
  }

  // Close to pi, sin(theta) vanishes and the skew part cannot give the axis.
  // Use the symmetric part instead
  //
  //   (R + R') / 2 = cos(theta) I + (1 - cos(theta)) a a'
  const Mat3 B = 0.5 * (R + R.transpose());
  const Mat3 aat = (B - cos_theta * Mat3::Identity()) / (1.0 - cos_theta);
  int i = 0;
  aat.diagonal().maxCoeff(&i);
  Vec3 axis = aat.col(i) / std::sqrt(aat(i, i));
  axis.normalize();

  // The sign of the axis comes from the skew part, which is still correct to
  // first order
  if (axis.dot(sin_axis) < 0.0) {
    axis = -axis;
  }
  return theta * axis;
}

Mat3 exp(const Vec3 &w) {
  const double theta = w.norm();
  if (theta < 1e-8) {
    return Mat3::Identity() + skew(w);
  }
  const Mat3 wx = skew(w);
  return Mat3::Identity() + (std::sin(theta) / theta) * wx
         + ((1.0 - std::cos(theta)) / (theta * theta)) * wx * wx;
}

Mat3 right_jacobian(const Vec3 &w) {
  // Jr = I - a(theta) W + b(theta) W^2, where W = skew(w)
  //
  //   a = (1 - cos(theta)) / theta^2,  b = (theta - sin(theta)) / theta^3
  //
  // Both lose most of their digits to cancellation for small angles, use their
  // series instead
  const double theta = w.norm();
  const double theta2 = theta * theta;
  double a, b;
  if (theta < 1e-2) {
    a = 1.0 / 2.0 - theta2 / 24.0 + theta2 * theta2 / 720.0;
    b = 1.0 / 6.0 - theta2 / 120.0 + theta2 * theta2 / 5040.0;
  } else {
    a = (1.0 - std::cos(theta)) / theta2;
    b = (theta - std::sin(theta)) / (theta2 * theta);
  }

  const Mat3 W = skew(w);
  return Mat3::Identity() - a * W + b * W * W;
}

Mat3 right_jacobian_inv(const Vec3 &w) {
  // Jr^-1 = I + W / 2 + c(theta) W^2, where W = skew(w)
  //
  //   c = 1 / theta^2 - (1 + cos(theta)) / (2 theta sin(theta))
  //
  // c loses most of its digits to cancellation for small angles, use its
  // series instead
  const double theta = w.norm();
  const double theta2 = theta * theta;
  double c;
  if (theta < 1e-2) {
    c = 1.0 / 12.0 + theta2 / 720.0 + theta2 * theta2 / 30240.0;
  } else {
    c = 1.0 / theta2
        - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  }

  const Mat3 W = skew(w);
  return Mat3::Identity() + 0.5 * W + c * W * W;
}

Mat3 solve_handeye(const std::map<timestamp_t, Mat4> &A,
                   const std::map<timestamp_t, Mat4> &B) {
  // Extract timestamps from A and B
  std::vector<timestamp_t> timestamps_a;
  std::vector<timestamp_t> timestamps_b;
  for (const auto &[ts, _] : A) {
    timestamps_a.push_back(ts);
  }
  for (const auto &[ts, _] : B) {
    timestamps_b.push_back(ts);
  }

  // Perform a intersection between timestamps_a and timestamps_b
  std::vector<timestamp_t> timestamps;
  std::set_intersection(timestamps_a.begin(),
                        timestamps_a.end(),
                        timestamps_b.begin(),
                        timestamps_b.end(),
                        std::back_inserter(timestamps));

  // Form linear system - accumulate the rotation axes of all pairs
  //
  //   a_k = R * b_k  =>  H = sum_k a_k * b_k^T
  //
  // A single pair gives a rank 1 H, which cannot determine the rotation about
  // the axis, and is dominated by noise in that one sample
  Mat3 H = Mat3::Zero();
  for (const auto ts : timestamps) {
    const Vec3 a = log(tf_rot(A.at(ts)));
    const Vec3 b = log(tf_rot(B.at(ts)));
    H += a * b.transpose();
  }

  // Solve linear system
  Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 U = svd.matrixU();
  Mat3 V = svd.matrixV();
  Mat3 X = U * V.transpose();

  // Ensure right-handed rotation
  if (X.determinant() < 0) {
    U.col(2) *= -1;
    X = U * V.transpose();
  }

  return X;
}

} // namespace SO3
} // namespace cartesian
