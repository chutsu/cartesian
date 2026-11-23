#include "MarginalError.hpp"

#include <algorithm>
#include <map>

namespace cartesian {

static bool is_pose(const ParamBlock::Type type) {
  return type == ParamBlock::POSE || type == ParamBlock::EXTRINSIC ||
         type == ParamBlock::FIDUCIAL;
}

MarginalError::MarginalError(const std::vector<double *> &param_ptrs,
                             const std::vector<ParamBlock::Type> &param_types,
                             const std::vector<int> &offsets,
                             const std::vector<VecX> &x0,
                             const VecX &r0,
                             const MatX &J0)
    : ResidualBlock{"MarginalError",
                    param_ptrs,
                    param_types,
                    static_cast<int>(r0.size())},
      offsets_{offsets}, x0_{x0}, r0_{r0}, J0_{J0} {}

std::shared_ptr<MarginalError>
MarginalError::create(const ResidualBlockPtrs &factors,
                      const std::vector<double *> &marg_params,
                      const std::set<double *> &fixed_params,
                      const std::vector<const ceres::LossFunction *> &losses) {
  // Collect the parameters of the factors, the ones to marginalize go first
  std::map<double *, ParamBlock::Type> types;
  std::vector<double *> order;
  for (const auto &factor : factors) {
    const auto ptrs = factor->get_param_ptrs();
    const auto &param_types = factor->get_param_types();
    for (size_t i = 0; i < ptrs.size(); i++) {
      if (fixed_params.count(ptrs[i]) == 0 && types.count(ptrs[i]) == 0) {
        types[ptrs[i]] = param_types[i];
        order.push_back(ptrs[i]);
      }
    }
  }
  const std::set<double *> marg_set{marg_params.begin(), marg_params.end()};
  std::vector<double *> marg, remain;
  for (auto ptr : order) {
    (marg_set.count(ptr) ? marg : remain).push_back(ptr);
  }

  // Index of each parameter in the Hessian
  std::map<double *, int> index;
  int m = 0;
  int r = 0;
  for (auto ptr : marg) {
    index[ptr] = m;
    m += ParamBlock::get_local_size(types.at(ptr));
  }
  for (auto ptr : remain) {
    index[ptr] = m + r;
    r += ParamBlock::get_local_size(types.at(ptr));
  }
  if (r == 0) {
    return nullptr;
  }

  // Gauss-Newton: H = J' * J, b = -J' * e
  MatX H = MatX::Zero(m + r, m + r);
  VecX b = VecX::Zero(m + r);
  for (size_t k = 0; k < factors.size(); k++) {
    const auto &factor = factors[k];
    const auto ptrs = factor->get_param_ptrs();
    const auto &param_types = factor->get_param_types();
    const int r_size = factor->num_residuals();

    std::vector<MatXRowMajor> jacs;
    std::vector<double *> jac_ptrs;
    for (size_t i = 0; i < ptrs.size(); i++) {
      const int local_size = ParamBlock::get_local_size(param_types[i]);
      jacs.push_back(MatXRowMajor::Zero(r_size, local_size));
    }
    for (auto &jac : jacs) {
      jac_ptrs.push_back(jac.data());
    }
    VecX e = VecX::Zero(r_size);
    if (factor->eval(ptrs.data(), e.data(), jac_ptrs.data()) == false) {
      return nullptr;
    }

    // Robust loss, reweight the residual and its jacobians by sqrt(rho')
    if (k < losses.size() && losses[k] != nullptr) {
      double rho[3];
      losses[k]->Evaluate(e.squaredNorm(), rho);
      const double w = std::sqrt(std::max(rho[1], 0.0));
      e *= w;
      for (auto &jac : jacs) {
        jac *= w;
      }
    }

    for (size_t i = 0; i < ptrs.size(); i++) {
      if (index.count(ptrs[i]) == 0) {
        continue;
      }
      const int idx_i = index.at(ptrs[i]);
      const int size_i = jacs[i].cols();
      b.segment(idx_i, size_i) -= jacs[i].transpose() * e;
      for (size_t j = 0; j < ptrs.size(); j++) {
        if (index.count(ptrs[j]) == 0) {
          continue;
        }
        const int idx_j = index.at(ptrs[j]);
        const int size_j = jacs[j].cols();
        H.block(idx_i, idx_j, size_i, size_j) +=
            jacs[i].transpose() * jacs[j];
      }
    }
  }

  // Schur complement, with the pseudo inverse of Hmm since the marginalized
  // parameters may be only partially observable
  MatX Hrr = H.block(m, m, r, r);
  VecX br = b.segment(m, r);
  if (m > 0) {
    const MatX Hmm = 0.5 * (H.block(0, 0, m, m) + H.block(0, 0, m, m).transpose());
    const Eigen::SelfAdjointEigenSolver<MatX> eig(Hmm);
    // Relative to the largest eigenvalue, the imu factors have a wide range
    const double tol_m = std::numeric_limits<double>::epsilon() * m *
                         eig.eigenvalues().maxCoeff();
    VecX inv = VecX::Zero(m);
    for (int i = 0; i < m; i++) {
      inv(i) = eig.eigenvalues()(i) > tol_m ? 1.0 / eig.eigenvalues()(i) : 0.0;
    }
    const MatX Hmm_inv = eig.eigenvectors() * inv.asDiagonal() *
                         eig.eigenvectors().transpose();
    const MatX Hrm = H.block(m, 0, r, m);
    const MatX Hmr = H.block(0, m, m, r);
    Hrr -= Hrm * Hmm_inv * Hmr;
    br -= Hrm * Hmm_inv * b.segment(0, m);
  }
  Hrr = 0.5 * (Hrr + Hrr.transpose());

  // Factor the result into a prior, H = J0' * J0 and J0' * r0 = -b, with the
  // pivoted LDLT, H = P' * L * D * L' * P so J0 = sqrt(D) * L' * P. Note that
  // the permutation applies to the columns of L' as P', not P, in Eigen
  const Eigen::LDLT<MatX> ldlt(Hrr);
  const VecX D = ldlt.vectorD();
  const double tol = std::numeric_limits<double>::epsilon() * r * D.maxCoeff();
  VecX sqrt_D = VecX::Zero(r);
  for (int i = 0; i < r; i++) {
    sqrt_D(i) = D(i) > tol ? std::sqrt(D(i)) : 0.0;
  }
  MatX J0 = ldlt.matrixU();
  J0 = J0 * ldlt.transpositionsP().transpose();
  J0 = sqrt_D.asDiagonal() * J0;
  const VecX z = ldlt.matrixL().solve(VecX(ldlt.transpositionsP() * br));
  VecX r0 = VecX::Zero(r);
  for (int i = 0; i < r; i++) {
    r0(i) = sqrt_D(i) > 0.0 ? -z(i) / sqrt_D(i) : 0.0;
  }

  // Linearization point of the parameters that remain
  std::vector<ParamBlock::Type> remain_types;
  std::vector<int> offsets;
  std::vector<VecX> x0;
  for (auto ptr : remain) {
    const auto type = types.at(ptr);
    remain_types.push_back(type);
    offsets.push_back(index.at(ptr) - m);
    x0.push_back(Eigen::Map<VecX>(ptr, ParamBlock::get_param_size(type)));
  }

  return std::shared_ptr<MarginalError>(
      new MarginalError(remain, remain_types, offsets, x0, r0, J0));
}

VecX MarginalError::delta_chi(double const *const *params) const {
  VecX dx = VecX::Zero(r0_.size());
  for (size_t i = 0; i < param_ptrs_.size(); i++) {
    const auto type = param_types_[i];
    const int size = ParamBlock::get_param_size(type);
    const Eigen::Map<const VecX> x(params[i], size);
    const VecX &x_0 = x0_[i];

    if (is_pose(type)) {
      // Same convention as the PoseManifold, q = q0 * dq
      const Quat q{x(6), x(3), x(4), x(5)};
      const Quat q0{x_0(6), x_0(3), x_0(4), x_0(5)};
      const Quat dq = q0.inverse() * q;
      dx.segment<3>(offsets_[i]) = x.head<3>() - x_0.head<3>();
      dx.segment<3>(offsets_[i] + 3) = 2.0 * (dq.w() < 0 ? -1.0 : 1.0) * dq.vec();
    } else {
      dx.segment(offsets_[i], size) = x - x_0;
    }
  }
  return dx;
}

bool MarginalError::eval(double const *const *params,
                         double *res,
                         double **jacs) const {
  Eigen::Map<VecX> e(res, r0_.size());
  e = r0_ + J0_ * delta_chi(params);

  // First estimate jacobians
  if (jacs != nullptr) {
    for (size_t i = 0; i < param_ptrs_.size(); i++) {
      if (jacs[i] == nullptr) {
        continue;
      }
      const int local_size = ParamBlock::get_local_size(param_types_[i]);
      Eigen::Map<MatXRowMajor> J(jacs[i], r0_.size(), local_size);
      J = J0_.middleCols(offsets_[i], local_size);
    }
  }

  return true;
}

} // namespace cartesian
