#pragma once
#include "CameraModel.hpp"

namespace cartesian {

/**
 * Unified Camera Model
 *
 * Projects a point onto the unit sphere, and then onto the image plane from a
 * point `xi` above the center of the sphere, which covers pinhole (xi = 0),
 * fisheye and catadioptric cameras (xi = 1 is a stereographic projection).
 * The projected point is distorted with radial-tangential distortion.
 *
 * Parameters: fx, fy, cx, cy, xi, k1, k2, p1, p2
 *
 * Source:
 *
 * C. Mei and P. Rives. Single view point omnidirectional
 * camera calibration from planar grids. In Proceedings 2007
 * IEEE International Conference on Robotics and Automation,
 * pages 3945–3950, April 2007
 *
 */
struct UnifiedCameraModel : CameraModel {
  UnifiedCameraModel() = default;
  virtual ~UnifiedCameraModel() = default;

  virtual std::string type() const override;

  /**
   * Project point `p_C`. Points up to (and beyond) 90 degrees off the optical
   * axis can be projected, depending on xi.
   * @returns 0 if the point projects inside the image, -1 otherwise.
   */
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;

  /** Jacobian of the projection w.r.t the point (2x3) */
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;

  /** Jacobian of the projection w.r.t the parameters (2x9) */
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;

  /**
   * Back project pixel `x` to a unit ray. Unlike the other camera models, the
   * ray is a unit vector, and can point 90 degrees or more off the optical axis.
   */
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;

  /**
   * Undistort pixel `z` to a pinhole camera pixel with the same fx, fy, cx
   * and cy. Only valid for rays less than 90 degrees off the optical axis.
   */
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace cartesian
