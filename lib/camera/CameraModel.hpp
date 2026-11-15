#pragma once

#include "../core/Core.hpp"

namespace cartesian {

/** Camera Model, the interface of the camera projection models */
struct CameraModel {
  CameraModel() = default;
  virtual ~CameraModel() = default;

  /** Return camera model type, e.g. "pinhole" */
  virtual std::string type() const = 0;

  /** Project `p_C` to `z_hat`, returns 0 if inside the image, else -1 */
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const = 0;

  /** Jacobian of the projection w.r.t. the point `p_C` (2x3) */
  virtual MatX project_jacobian(const VecX &params, const Vec3 &p_C) const = 0;

  /** Jacobian of the projection w.r.t. the camera parameters (2xN) */
  virtual MatX params_jacobian(const VecX &params, const Vec3 &p_C) const = 0;

  /** Back project pixel `x` to a ray in the camera frame */
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const = 0;

  /** Undistort pixel `z` to the pixel of a pinhole camera with the same K */
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const = 0;
};

} // namespace cartesian
