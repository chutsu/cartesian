#pragma once
#include "../core/Core.hpp"
#include "CameraModel.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// PINHOLE UTILS                                                             //
///////////////////////////////////////////////////////////////////////////////

/**
 * Estimate pinhole focal length using image size in pixels and field of view
 * in radians.
 */
double pinhole_focal(const int image_size, const double fov);

/** Form pinhole camera matrix K = [fx 0 cx; 0 fy cy; 0 0 1]. */
Mat3 pinhole_K(const double fx,
               const double fy,
               const double cx,
               const double cy);

/**
 * Project 3D point p_C in the camera frame, using proj_params (fx, fy, cx, cy)
 * where the camera resolution is res. Returns 0 or 1 for success or failure.
 */
int pinhole_project(const Vec2i &res,
                    const Vec4 &proj_params,
                    const Vec3 &p_C,
                    Vec2 &z_hat);

/** Form pinhole projection jacobian (2x3) of (x/z, y/z) w.r.t. `p_C`. */
Mat<2, 3> pinhole_project_jacobian(const Vec3 &p_C);

/** Form pinhole point jacobian (2x2), diag(fx, fy) from `proj_params`. */
Mat2 pinhole_point_jacobian(const Vec4 &proj_params);

/** Form pinhole params jacobian (2x4) w.r.t. fx, fy, cx, cy at point `x`. */
Mat<2, 4> pinhole_params_jacobian(const Vec2 &x);

///////////////////////////////////////////////////////////////////////////////
// PINHOLE                                                                   //
///////////////////////////////////////////////////////////////////////////////

/** Pinhole Model, no distortion, parameters: fx, fy, cx, cy */
struct Pinhole : CameraModel {
  Pinhole() = default;
  virtual ~Pinhole() = default;

  /** Return camera model type, "pinhole" */
  virtual std::string type() const override;

  /** Project `p_C` to `z_hat`, returns 0 if in front and in image, else -1 */
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;

  /** Jacobian of the projection w.r.t. the point `p_C` (2x3) */
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;

  /** Jacobian of the projection w.r.t. the parameters fx, fy, cx, cy (2x4) */
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;

  /** Back project pixel `x` to a ray (x/z, y/z, 1), not normalized */
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;

  /** Undistort pixel `z`, returns `z` as the pinhole has no distortion */
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace cartesian
