#pragma once
#include <string>

#include "../core/Core.hpp"
#include "CameraGeometry.hpp"

namespace cartesian {

/** Camera model specification, used to build a ground truth camera */
struct CameraSpec {
  std::string model =
      "BrownConrady4"; // or "KannalaBrandt4", "UnifiedCameraModel"
  Vec2i resolution{752, 480};
  double fov = 90.0; // Horizontal field of view [deg]
  Vec4 dist{0,
            0,
            0,
            0};    // BrownConrady4 and UnifiedCameraModel: k1, k2, p1, p2.
                   // KannalaBrandt4: k1..k4
  double xi = 1.0; // UnifiedCameraModel
  double cx_offset = 0.0; // Principal point offset from image center [px]
  double cy_offset = 0.0;
  double fy_ratio = 1.0; // fy / fx
};

/**
 * Make a camera such that the horizontal image edge, through the principal
 * point, sees exactly `fov` degrees (distortion included).
 */
CameraGeometry make_camera(const CameraSpec &spec);

/**
 * Unit ray (camera frame) through image point `z`. Unlike the camera models'
 * `back_project()` this also works for rays 90 degrees or more off the optical
 * axis of fisheye lenses.
 */
Vec3 camera_ray(const CameraGeometry &camera, const Vec2 &z);

/**
 * Check the camera's projection is monotonic from the principal point to the
 * image corners, i.e. the distortion does not fold the image over.
 */
bool camera_is_monotonic(const CameraGeometry &camera);

} // namespace cartesian
