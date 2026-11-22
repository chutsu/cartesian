#include "SfmTrack.hpp"

namespace cartesian {

bool SfmTrack::triangulate(const CameraGeometry &camera_geom,
                           const Mat4 &T_WC_first,
                           const Mat4 &T_WC_last,
                           const double min_parallax_deg,
                           const double max_reproj_error,
                           Vec3 &p_W) const {
  // Use the first and last measurements, which have the largest baseline
  if (keypoints.size() < 2) {
    return false;
  }
  const auto &kp1 = keypoints.begin()->second;
  const auto &kp2 = keypoints.rbegin()->second;

  // Rays in the world frame
  const Vec3 c1 = tf_trans(T_WC_first);
  const Vec3 c2 = tf_trans(T_WC_last);
  const Vec2 z1{kp1.pt.x, kp1.pt.y};
  const Vec2 z2{kp2.pt.x, kp2.pt.y};
  const Vec3 d1 = tf_rot(T_WC_first) * camera_geom.unit_ray(z1);
  const Vec3 d2 = tf_rot(T_WC_last) * camera_geom.unit_ray(z2);

  // Enough parallax?
  const double cos_angle = std::min(1.0, std::max(-1.0, d1.dot(d2)));
  if (rad2deg(std::acos(cos_angle)) < min_parallax_deg) {
    return false;
  }

  // Midpoint of the shortest segment between the two rays
  const Vec3 b = c2 - c1;
  const double a = d1.dot(d2);
  const double denom = 1.0 - a * a;
  if (denom < 1e-12) {
    return false;
  }
  const double s1 = (d1.dot(b) - a * d2.dot(b)) / denom;
  const double s2 = (a * d1.dot(b) - d2.dot(b)) / denom;
  if (s1 <= 0.0 || s2 <= 0.0) {
    return false;
  }
  p_W = 0.5 * ((c1 + s1 * d1) + (c2 + s2 * d2));

  // Check the point projects back to the measurements
  const std::vector<std::pair<Mat4, Vec2>> views = {{T_WC_first, z1},
                                                    {T_WC_last, z2}};
  for (const auto &[T_WC, z] : views) {
    const Vec3 p_C = tf_point(T_WC.inverse(), p_W);
    Vec2 z_hat;
    if (camera_geom.project(p_C, z_hat) != 0) {
      return false;
    }
    if ((z_hat - z).norm() > max_reproj_error) {
      return false;
    }
  }

  return true;
}

} // namespace cartesian
