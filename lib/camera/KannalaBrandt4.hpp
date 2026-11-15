#include "CameraModel.hpp"
#include "Pinhole.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// EQUIDISTANT DISTORTION                                                    //
///////////////////////////////////////////////////////////////////////////////

/** Distort normalized point `p` with equidistant distortion k1..k4 */
Vec2 equi4_distort(const Vec4 &dist_params, const Vec2 &p);

/** Undistort distorted normalized point `p0`, inverse of `equi4_distort` */
Vec2 equi4_undistort(const Vec4 &dist_params, const Vec2 &p0);

/** Jacobian of `equi4_distort` w.r.t. the normalized point `p` (2x2) */
Mat2 equi4_point_jacobian(const Vec4 &dist_params, const Vec2 &p);

/** Jacobian of `equi4_distort` w.r.t. the distortion k1..k4 (2x4) */
MatX equi4_params_jacobian(const Vec4 &dist_params, const Vec2 &p);

///////////////////////////////////////////////////////////////////////////////
// KANNALA-BRANDT4 CAMERA MODEL                                              //
///////////////////////////////////////////////////////////////////////////////

/**
 * Kanala-Brandt Camera Model
 *
 * Projection and back projection are computed from the angle off the optical
 * axis, so rays 90 degrees or more off the optical axis are supported (the
 * back projected ray is a unit vector). `undistort()` is only valid for rays
 * less than 90 degrees off the optical axis.
 *
 * Source:
 *
 *   Kannala, J. and Brandt, S.S., 2006. A generic camera model and calibration
 *   method for conventional, wide-angle, and fish-eye lenses. IEEE transactions
 *   on pattern analysis and machine intelligence, 28(8), pp.1335-1340.
 *
 */
struct KannalaBrandt4 : CameraModel {
  KannalaBrandt4() = default;
  virtual ~KannalaBrandt4() = default;

  /** Return camera model type, "KannalaBrandt4" */
  virtual std::string type() const override;

  /** Project `p_C` to `z_hat`, returns 0 if inside the image, else -1 */
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;

  /** Jacobian of the projection w.r.t. the point `p_C` (2x3) */
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;

  /** Jacobian of the projection w.r.t. fx, fy, cx, cy, k1..k4 (2x8) */
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;

  /** Back project pixel `x` to a unit ray, can be 90 deg or more off axis */
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;

  /** Undistort pixel `z` to a pinhole pixel, valid under 90 deg off axis */
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace cartesian
