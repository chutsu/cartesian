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

} // namespace cartesian
