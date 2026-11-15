#include "CameraModel.hpp"
#include "Pinhole.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// EQUIDISTANT DISTORTION                                                    //
///////////////////////////////////////////////////////////////////////////////

Vec2 equi4_distort(const Vec4 &dist_params, const Vec2 &p);
Vec2 equi4_undistort(const Vec4 &dist_params, const Vec2 &p0);
Mat2 equi4_point_jacobian(const Vec4 &dist_params, const Vec2 &p);
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

  virtual std::string type() const override;
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace cartesian
