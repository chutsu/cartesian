#pragma once
#include <memory>
#include <ceres/ceres.h>
#include <set>

#include "../core/Core.hpp"
#include "ParamBlock.hpp"
#include "ResidualBlock.hpp"

namespace cartesian {

/**
 * Marginalization prior
 *
 * Marginalizes the parameters `marg_params` out of `factors` with the Schur
 * complement of their Gauss-Newton Hessian, and keeps the result as a
 * linearized prior on the parameters that remain:
 *
 *   e(x) = r0 + J0 * dx
 *
 * where `dx` is the difference of the remaining parameters from their value at
 * the time of marginalization (the linearization point, first estimate
 * jacobians). The factors can be removed from the problem afterwards, the
 * prior replaces them. A prior can be marginalized again as one of the factors
 * of the next one, to slide a window over a long sequence.
 *
 * Parameters in `fixed_params` are constants of the problem, they are left out
 * of the Hessian and of the prior. `losses` are the robust losses of the
 * factors (nullptr for none), empty for none at all. They are applied as
 * iteratively reweighted least squares, at the linearization point, so that an
 * outlier does not get the full weight of a quadratic in the prior.
 */
class MarginalError : public ResidualBlock {
private:
  std::vector<int> offsets_; // Offset of each parameter in the tangent space
  std::vector<VecX> x0_;     // Linearization point of the parameters
  VecX r0_;                  // Linearized residuals at x0
  MatX J0_;                  // Linearized jacobians at x0

  MarginalError(const std::vector<double *> &param_ptrs,
                const std::vector<ParamBlock::Type> &param_types,
                const std::vector<int> &offsets,
                const std::vector<VecX> &x0,
                const VecX &r0,
                const MatX &J0);

  /** Difference of the parameters from the linearization point */
  VecX delta_chi(double const *const *params) const;

public:
  /**
   * Marginalize `marg_params` out of `factors`. Returns nullptr if there is
   * nothing left to put a prior on.
   */
  static std::shared_ptr<MarginalError>
  create(const ResidualBlockPtrs &factors,
         const std::vector<double *> &marg_params,
         const std::set<double *> &fixed_params = {},
         const std::vector<const ceres::LossFunction *> &losses = {});

  /** Size of the prior, the dimension of the tangent space of the parameters */
  int size() const { return static_cast<int>(r0_.size()); }

  /** Evaluate the prior and its jacobians w.r.t. the minimal parameters */
  bool eval(double const *const *params,
            double *res,
            double **jacs = nullptr) const override;
};

} // namespace cartesian
