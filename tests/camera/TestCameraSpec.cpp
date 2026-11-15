#include <gtest/gtest.h>

#include "camera/CameraSpec.hpp"

namespace cartesian {

static void check_fov(const CameraSpec &spec) {
  const auto camera = make_camera(spec);
  const double w = spec.resolution.x();
  const double cy = camera.intrinsic(3);

  // The horizontal image edge, through the principal point, sees `fov`
  const Vec3 ray = camera_ray(camera, Vec2{w, cy});
  const double theta = rad2deg(acos(ray.z()));
  ASSERT_NEAR(theta, spec.fov / 2.0, 1e-3);
}

TEST(CameraSpec, make_camera_fov) {
  CameraSpec spec;
  spec.model = "BrownConrady4";
  spec.dist = Vec4{0.1, 0.01, 0.001, 0.001}; // Tangential is ignored at edge
  spec.dist(2) = 0.0;
  spec.dist(3) = 0.0;
  check_fov(spec);

  spec.model = "KannalaBrandt4";
  spec.fov = 180.0;
  spec.dist = Vec4{0.01, 0.001, 0.0, 0.0};
  check_fov(spec);
}

TEST(CameraSpec, camera_ray_unit_and_center) {
  CameraSpec spec;
  const auto camera = make_camera(spec);
  const Vec3 ray =
      camera_ray(camera, Vec2{camera.intrinsic(2), camera.intrinsic(3)});
  ASSERT_NEAR((ray - Vec3{0, 0, 1}).norm(), 0.0, 1e-9);
  ASSERT_NEAR(camera_ray(camera, Vec2{10, 20}).norm(), 1.0, 1e-9);
}

TEST(CameraSpec, camera_is_monotonic) {
  CameraSpec spec;
  ASSERT_TRUE(camera_is_monotonic(make_camera(spec)));

  // Strong distortion folds the image over
  spec.dist = Vec4{-0.5, 0.0, 0.0, 0.0};
  ASSERT_FALSE(camera_is_monotonic(make_camera(spec)));
}

} // namespace cartesian
