#include "CameraSpec.hpp"

namespace cartesian {

static bool is_fisheye(const CameraGeometry &camera) {
  return camera.camera_model->type() == "KannalaBrandt4";
}

/** Radial distortion `thd(th)` of the Kannala-Brandt model */
static double equi_thd(const Vec4 &k, const double th) {
  const double th2 = th * th;
  return th * (1 + k(0) * th2 + k(1) * th2 * th2 + k(2) * th2 * th2 * th2 +
               k(3) * th2 * th2 * th2 * th2);
}

/** Radial distortion of the radial-tangential model along the x-axis */
static double radtan_rd(const Vec4 &d, const double r) {
  const double r2 = r * r;
  return r * (1 + d(0) * r2 + d(1) * r2 * r2);
}

CameraGeometry make_camera(const CameraSpec &spec) {
  const double w = spec.resolution.x();
  const double h = spec.resolution.y();
  const double half_fov = deg2rad(spec.fov / 2.0);

  // Normalized radius of the horizontal image edge, including distortion
  double edge = 0.0;
  if (spec.model == "KannalaBrandt4") {
    edge = equi_thd(spec.dist, half_fov);
  } else if (spec.model == "UnifiedCameraModel") {
    edge = radtan_rd(spec.dist, sin(half_fov) / (cos(half_fov) + spec.xi));
  } else {
    edge = radtan_rd(spec.dist, tan(half_fov));
  }

  // The edge of the image (w / 2 px) sees half the field of view
  const double fx = (w / 2.0) / edge;
  const double fy = fx * spec.fy_ratio;
  const double cx = w / 2.0 + spec.cx_offset;
  const double cy = h / 2.0 + spec.cy_offset;

  if (spec.model == "UnifiedCameraModel") {
    VecX intrinsic = zeros(9, 1);
    intrinsic << fx, fy, cx, cy, spec.xi, spec.dist;
    return CameraGeometry{0, spec.model, spec.resolution, intrinsic};
  }
  VecX intrinsic = zeros(8, 1);
  intrinsic << fx, fy, cx, cy, spec.dist;
  return CameraGeometry{0, spec.model, spec.resolution, intrinsic};
}

Vec3 camera_ray(const CameraGeometry &camera, const Vec2 &z) {
  if (is_fisheye(camera) == false) {
    Vec3 ray;
    camera.back_project(z, ray);
    return ray.normalized();
  }

  // Invert th_d = th * (1 + k1 th^2 + ... + k4 th^8) with Newton's method
  const VecX &K = camera.intrinsic;
  const Vec4 k = K.tail(4);
  const Vec2 p{(z.x() - K(2)) / K(0), (z.y() - K(3)) / K(1)};
  const double thd = p.norm();
  if (thd < 1e-12) {
    return Vec3{0.0, 0.0, 1.0};
  }

  double th = thd;
  for (int i = 0; i < 30; ++i) {
    const double th2 = th * th;
    const double f = equi_thd(k, th) - thd;
    const double df = 1 + 3 * k(0) * th2 + 5 * k(1) * th2 * th2 +
                      7 * k(2) * th2 * th2 * th2 +
                      9 * k(3) * th2 * th2 * th2 * th2;
    th -= f / df;
  }
  const Vec2 dir = p / thd;
  return Vec3{sin(th) * dir.x(), sin(th) * dir.y(), cos(th)};
}

bool camera_is_monotonic(const CameraGeometry &camera) {
  // Walk along the image diagonal, away from the principal point, and make
  // sure the angle off the optical axis keeps increasing
  const VecX &K = camera.intrinsic;
  const Vec2 c{K(2), K(3)};
  const Vec2 corner{camera.resolution.x() * 1.0, camera.resolution.y() * 1.0};
  const Vec2 dir = (corner - c).normalized();
  const double max_r = (corner - c).norm();

  double prev_theta = -1.0;
  for (double r = 0.0; r <= max_r; r += 1.0) {
    const Vec3 ray = camera_ray(camera, c + r * dir);
    const double theta = acos(std::min(1.0, std::max(-1.0, ray.z())));
    if (theta < prev_theta - 1e-9) {
      return false;
    }
    prev_theta = theta;
  }
  return true;
}

} // namespace cartesian
