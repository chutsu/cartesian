#pragma once
#include "Core.hpp"

namespace cartesian {
namespace SE3 {

/**
 * Tangent vectors are ordered translation first, `xi = [rho, phi]`, where
 * `rho` is the translational and `phi` the rotational part. The same ordering
 * as the pose perturbation used in the error terms.
 */

/** Logarithm map, transform T to tangent vector xi = [rho, phi] */
Vec6 log(const Mat4 &T);

/** Exponential map, tangent vector xi = [rho, phi] to transform T */
Mat4 exp(const Vec6 &xi);

/** Adjoint of transform T, Ad(T) exp(xi) = exp(Ad(T) xi) T */
Mat<6, 6> adjoint(const Mat4 &T);

/** Left jacobian of SE(3) */
Mat<6, 6> left_jacobian(const Vec6 &xi);

/** Inverse of the left jacobian of SE(3) */
Mat<6, 6> left_jacobian_inv(const Vec6 &xi);

/** Right jacobian of SE(3), exp(xi + dxi) ~ exp(xi) exp(Jr(xi) dxi) */
Mat<6, 6> right_jacobian(const Vec6 &xi);

/** Inverse of the right jacobian of SE(3) */
Mat<6, 6> right_jacobian_inv(const Vec6 &xi);

} // namespace SE3
} // namespace cartesian
