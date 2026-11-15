#include "CameraModel.hpp"
#include "Pinhole.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// RADIAL-TANGENTIAL DISTORTION                                              //
///////////////////////////////////////////////////////////////////////////////

/** Distort normalized point `p` with radial-tangential k1, k2, p1, p2 */
Vec2 radtan4_distort(const Vec4 &dist_params, const Vec2 &p);

/** Undistort distorted normalized point `p0`, inverse of `radtan4_distort` */
Vec2 radtan4_undistort(const Vec4 &dist_params, const Vec2 &p0);

/** Jacobian of `radtan4_distort` w.r.t. the normalized point `p` (2x2) */
Mat2 radtan4_point_jacobian(const Vec4 &dist_params, const Vec2 &p);

/** Jacobian of `radtan4_distort` w.r.t. the distortion k1, k2, p1, p2 (2x4) */
MatX radtan4_params_jacobian(const Vec4 &dist_params, const Vec2 &p);

///////////////////////////////////////////////////////////////////////////////
// BROWN-CONRADY4 CAMERA MODEL                                               //
///////////////////////////////////////////////////////////////////////////////

/**
 * Brown-Conrady Camera Model
 *
 * Parameters: fx, fy, cx, cy, k1, k2, p1, p2
 *
 * Source:
 *
 *   Brown, D., 1996. Decentering distortion of lenses. Photogrammetric
 *   engineering, 32(3), pp.444-462.
 *
 *   Conrady, A.E., 1919. Decentred lens-systems. Monthly notices of the
 *   royal astronomical society, 79(5), pp.384-390.
 *
 */
struct BrownConrady4 : CameraModel {
  BrownConrady4() = default;
  virtual ~BrownConrady4() = default;

  /** Return camera model type, "BrownConrady4" */
  virtual std::string type() const override;

  /** Project `p_C` to `z_hat`, returns 0 if in front and in image, else -1 */
  virtual int project(const Vec2i &res,
                      const VecX &params,
                      const Vec3 &p_C,
                      Vec2 &z_hat) const override;

  /** Jacobian of the projection w.r.t. the point `p_C` (2x3) */
  virtual MatX project_jacobian(const VecX &params,
                                const Vec3 &p_C) const override;

  /** Jacobian of the projection w.r.t. fx, fy, cx, cy, k1, k2, p1, p2 (2x8) */
  virtual MatX params_jacobian(const VecX &params,
                               const Vec3 &p_C) const override;

  /** Back project pixel `x` to a ray (x/z, y/z, 1), not normalized */
  virtual void back_project(const VecX &params,
                            const Vec2 &x,
                            Vec3 &ray) const override;

  /** Undistort pixel `z` to the pixel of a pinhole camera with the same K */
  virtual Vec2 undistort(const VecX &params, const Vec2 &z) const override;
};

} // namespace cartesian
