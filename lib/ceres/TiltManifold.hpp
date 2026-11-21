#pragma once
#include <ceres/ceres.h>

#include "../core/Core.hpp"

namespace cartesian {

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

  /** Apply the tilt `delta` (roll, pitch in the world frame) to the pose `x` */
  virtual bool Plus(const double *x,
                    const double *delta,
                    double *x_plus_delta) const;
  /** Jacobian of `Plus` w.r.t. `delta` at the pose `x` (7x2, row major) */
  virtual bool PlusJacobian(const double *x, double *jacobian) const;
  /** Not implemented, returns false */
  virtual bool Minus(const double *x,
                     const double *delta,
                     double *x_minus_delta) const;
  /** Not implemented, returns false */
  virtual bool MinusJacobian(const double *x, double *jacobian) const;
  /** Size of the pose: position and quaternion */
  virtual int AmbientSize() const;
  /** Size of the tilt: roll and pitch */
  virtual int TangentSize() const;
};

} // namespace cartesian
