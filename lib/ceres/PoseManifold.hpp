#pragma once
#include <ceres/ceres.h>

#include "../core/Core.hpp"

namespace cartesian {

/** Pose Manifold **/
class PoseManifold : public ceres::Manifold {
public:
  PoseManifold() = default;
  virtual ~PoseManifold() = default;

  /** Plus */
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;

  /** Minus Jacobian */
  virtual bool PlusJacobian(const double *x, double *jacobian) const;

  /** Minus */
  virtual bool Minus(const double *x,
                     const double *delta,
                     double *x_minus_delta) const;

  /** Minus Jacobian */
  virtual bool MinusJacobian(const double *x, double *jacobian) const;

  /** Return Ambient size */
  virtual int AmbientSize() const;

  /** Return Tangent size */
  virtual int TangentSize() const;
};

/** Tilt Manifold
 *
 * Pose manifold that only estimates roll and pitch (tilt) of the pose, in the
 * world frame. Translation and yaw (rotation about gravity) are held fixed.
 * This removes the gauge freedom for a target pose T_WT0 in visual-inertial
 * calibration, where gravity is the only thing that makes tilt observable.
 *
 * Plus:   q' = Exp([d0, d1, 0]) * q,  r' = r
 *
 * Note: the Jacobian follows the same convention as `PoseManifold`, where the
 * residual Jacobians w.r.t. the ambient space are [J_r, J_alpha_local, 0].
 */
class TiltManifold : public ceres::Manifold {
public:
  TiltManifold() = default;
  virtual ~TiltManifold() = default;

  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;
  virtual bool PlusJacobian(const double *x, double *jacobian) const;
  virtual bool Minus(const double *x,
                     const double *delta,
                     double *x_minus_delta) const;
  virtual bool MinusJacobian(const double *x, double *jacobian) const;
  virtual int AmbientSize() const;
  virtual int TangentSize() const;
};

} // namespace cartesian
