#include "SimCamera.hpp"

namespace cartesian {

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

/** Make a camera from the distorted radius of the horizontal image edge */
static CameraGeometry make(const SimCamera &cam,
                           const std::string &model,
                           const double edge,
                           const VecX &tail) {
  // The edge of the image (w / 2 px) sees half the field of view
  const double w = cam.resolution.x();
  const double h = cam.resolution.y();
  const double fx = (w / 2.0) / edge;
  const double fy = fx * cam.fy_ratio;
  const double cx = w / 2.0 + cam.cx_offset;
  const double cy = h / 2.0 + cam.cy_offset;

  VecX intrinsic = zeros(4 + tail.size(), 1);
  intrinsic << fx, fy, cx, cy, tail;
  return CameraGeometry{0, model, cam.resolution, intrinsic};
}

double pinhole_edge_radius(const double fov) { return tan(deg2rad(fov / 2.0)); }

double ucm_edge_radius(const double fov, const double xi) {
  const double half_fov = deg2rad(fov / 2.0);
  return sin(half_fov) / (cos(half_fov) + xi);
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
    const Vec3 ray = camera.unit_ray(c + r * dir);
    const double theta = acos(std::min(1.0, std::max(-1.0, ray.z())));
    if (theta < prev_theta - 1e-9) {
      return false;
    }
    prev_theta = theta;
  }
  return true;
}

CameraGeometry SimCamera::make_brown_conrady4(const Vec4 &dist) const {
  const double edge = radtan_rd(dist, pinhole_edge_radius(fov));
  return make(*this, "BrownConrady4", edge, dist);
}

CameraGeometry SimCamera::make_kannala_brandt4(const Vec4 &dist) const {
  const double edge = equi_thd(dist, deg2rad(fov / 2.0));
  return make(*this, "KannalaBrandt4", edge, dist);
}

CameraGeometry SimCamera::make_unified_camera(const double xi,
                                              const Vec4 &dist) const {
  const double edge = radtan_rd(dist, ucm_edge_radius(fov, xi));
  VecX tail = zeros(5, 1);
  tail << xi, dist;
  return make(*this, "UnifiedCameraModel", edge, tail);
}

} // namespace cartesian
