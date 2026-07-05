#pragma once
#include <ceres/ceres.h>

#include "../core/Core.hpp"
#include "../camera/CameraGeometry.hpp"

#include "CalibTarget.hpp"

namespace cartesian {

/** SolvePnp */
struct SolvePnp {
  const CameraGeometry *camera_geometry = nullptr;

  SolvePnp() = delete;
  SolvePnp(const std::shared_ptr<CameraGeometry> &camera_geometry);
  SolvePnp(const CameraGeometry &camera_geometry);
  virtual ~SolvePnp() = default;

  /** Estimate relative pose */
  int estimate(const Vec2s &keypoints,
               const Vec3s &object_points,
               Mat4 &T_camera_object);

  /** Estimate relative pose */
  static int estimate(const std::shared_ptr<CameraGeometry> &camera_geometry,
                      const std::shared_ptr<CalibTarget> &calib_target,
                      Mat4 &T_camera_target);
};

} // namespace cartesian
