#pragma once
#include "Core.hpp"

namespace cartesian {
namespace SO3 {

/**
 * Logarithm map, rotation matrix `R` to rotation vector `w` (axis * angle)
 *
 * Accurate for small angles and close to pi. The angle is in [0, pi].
 */
Vec3 log(const Mat3 &R);

/** Exponential map, rotation vector `w` (axis * angle) to rotation matrix */
Mat3 exp(const Vec3 &w);

/** Right jacobian of SO(3), exp(w + dw) ~ exp(w) exp(Jr(w) dw) */
Mat3 right_jacobian(const Vec3 &w);

/** Inverse of the right jacobian of SO(3) */
Mat3 right_jacobian_inv(const Vec3 &w);

/**
 * Solve the rotation `X` of the hand-eye problem, A_k X = X B_k
 *
 * `A` and `B` are the relative poses between consecutive measurements of two
 * rigidly attached sensors, keyed by timestamp. Only the timestamps found in
 * both are used. With the rotation vectors a_k = log(A_k) and b_k = log(B_k),
 * a_k = X b_k, which is solved in the least squares sense over all pairs
 * (Kabsch) by the SVD of sum_k a_k b_k'.
 *
 * Only the rotation is solved, the translation is not observable from the
 * rotation vectors.
 */
Mat3 solve_handeye(const std::map<timestamp_t, Mat4> &A,
                   const std::map<timestamp_t, Mat4> &B);

} // namespace SO3
} // namespace cartesian
