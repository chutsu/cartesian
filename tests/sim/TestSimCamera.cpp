#include <gtest/gtest.h>

#include "sim/SimCamera.hpp"

namespace cartesian {

static void check_fov(const CameraGeometry &camera, const double fov) {
  // The horizontal image edge, through the principal point, sees `fov`
  const double w = camera.resolution.x();
  const Vec3 ray = camera.unit_ray(Vec2{w, camera.intrinsic(3)});
  ASSERT_NEAR(rad2deg(acos(ray.z())), fov / 2.0, 1e-3);
}

TEST(SimCamera, make_brown_conrady4_fov) {
  SimCameraParams params;
  params.fov = 90.0;
  const Vec4 dist{0.1, 0.01, 0.0, 0.0}; // Tangential is ignored at the edge
  const auto camera = make_brown_conrady4(params, dist);
  ASSERT_EQ(camera.camera_model->type(), "BrownConrady4");
  check_fov(camera, params.fov);
}

TEST(SimCamera, make_kannala_brandt4_fov) {
  SimCameraParams params;
  for (const double fov : {120.0, 180.0, 220.0}) {
    params.fov = fov;
    const auto camera = make_kannala_brandt4(params, Vec4{0.01, 0.001, 0, 0});
    ASSERT_EQ(camera.camera_model->type(), "KannalaBrandt4");
    check_fov(camera, fov);
  }
}

TEST(SimCamera, make_unified_camera_fov) {
  SimCameraParams params;
  for (const double fov : {120.0, 180.0, 200.0}) {
    params.fov = fov;
    const auto camera = make_unified_camera(params, 0.9, Vec4{0.0, 0.0, 0, 0});
    ASSERT_EQ(camera.camera_model->type(), "UnifiedCameraModel");
    ASSERT_EQ(camera.intrinsic.size(), 9);
    ASSERT_NEAR(camera.intrinsic(4), 0.9, 1e-12);
    check_fov(camera, fov);
  }
}

TEST(SimCamera, principal_point_and_focal_ratio) {
  SimCameraParams params;
  params.cx_offset = 4.0;
  params.cy_offset = -3.0;
  params.fy_ratio = 1.004;
  const auto camera = make_kannala_brandt4(params);
  ASSERT_NEAR(camera.intrinsic(2), params.resolution.x() / 2.0 + 4.0, 1e-12);
  ASSERT_NEAR(camera.intrinsic(3), params.resolution.y() / 2.0 - 3.0, 1e-12);
  ASSERT_NEAR(camera.intrinsic(1) / camera.intrinsic(0), 1.004, 1e-12);
}

TEST(SimCamera, unit_ray_and_center) {
  const auto camera = make_brown_conrady4(SimCameraParams{});
  const Vec3 ray =
      camera.unit_ray(Vec2{camera.intrinsic(2), camera.intrinsic(3)});
  ASSERT_NEAR((ray - Vec3{0, 0, 1}).norm(), 0.0, 1e-9);
  ASSERT_NEAR(camera.unit_ray(Vec2{10, 20}).norm(), 1.0, 1e-9);
}

TEST(SimCamera, camera_is_monotonic) {
  const SimCameraParams params;
  ASSERT_TRUE(camera_is_monotonic(make_brown_conrady4(params)));

  // Strong distortion folds the image over
  ASSERT_FALSE(
      camera_is_monotonic(make_brown_conrady4(params, Vec4{-0.5, 0, 0, 0})));
}

} // namespace cartesian
