#pragma once
#include "../camera/CameraGeometry.hpp"
#include "../core/Core.hpp"

namespace cartesian {

/** Radius of the undistorted pinhole image plane at the horizontal edge */
double pinhole_edge_radius(const double fov);

/** Radius of the undistorted UCM normalized image plane at the horizontal edge */
double ucm_edge_radius(const double fov, const double xi);

/** Check the projection is monotonic out to the image corners (no fold over) */
bool camera_is_monotonic(const CameraGeometry &camera);

/**
 * Simulated camera: image size, field of view and principal point, from which
 * a ground truth CameraGeometry can be made.
 *
 * The cameras are made such that the horizontal image edge, through the
 * principal point, sees exactly `fov` degrees (distortion included).
 */
struct SimCamera {
  Vec2i resolution{752, 480}; // Image size [px]
  double fov = 90.0;          // Horizontal field of view [deg]
  double cx_offset = 0.0;     // Principal point offset from image center [px]
  double cy_offset = 0.0;     // Principal point offset from image center [px]
  double fy_ratio = 1.0;      // fy / fx

  /** Make a BrownConrady4 camera, `dist` is k1, k2, p1, p2 */
  CameraGeometry make_brown_conrady4(const Vec4 &dist = Vec4::Zero()) const;

  /** Make a KannalaBrandt4 camera, `dist` is k1..k4 */
  CameraGeometry make_kannala_brandt4(const Vec4 &dist = Vec4::Zero()) const;

  /** Make a UnifiedCameraModel camera, `dist` is k1, k2, p1, p2 */
  CameraGeometry make_unified_camera(const double xi,
                                     const Vec4 &dist = Vec4::Zero()) const;
};

} // namespace cartesian
