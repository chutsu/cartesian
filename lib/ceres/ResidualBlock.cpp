#include "ResidualBlock.hpp"

namespace cartesian {

static int compare_jacobian(const std::string &jac_name,
                            const MatX &fdiff,
                            const MatX &jac,
                            const double tol,
                            const bool print) {
  // Pre-check
  if (jac.size() == 0) {
    LOG_ERROR("Provided analytical jacobian is empty!");
    return false;
  } else if (fdiff.size() == 0) {
    LOG_ERROR("Provided numerical jacobian is empty!");
    return false;
  } else if (fdiff.rows() != jac.rows()) {
    LOG_ERROR("rows(fdiff) != rows(jac)");
    return false;
  } else if (fdiff.cols() != jac.cols()) {
    LOG_ERROR("cols(fdiff) != cols(jac)");
    return false;
  }

  // Check if any of the values are beyond the tol
  const MatX delta = (fdiff - jac);
  bool failed = false;
  for (long i = 0; i < delta.rows(); i++) {
    for (long j = 0; j < delta.cols(); j++) {
      if (fabs(delta(i, j)) >= tol) {
        failed = true;
      }
    }
  }

  // Print result
  int retval = 0;
  if (failed) {
    if (print) {
      LOG_ERROR("Check [%s] failed!\n", jac_name.c_str());
      print_matrix("num diff jac", fdiff);
      print_matrix("analytical jac", jac);
      print_matrix("difference matrix", delta);
      // exit(-1);
    }
    retval = -1;

  } else {
    if (print) {
      printf("Check [%s] passed!\n", jac_name.c_str());
      print_matrix("num diff jac", fdiff);
      print_matrix("analytical jac", jac);
      print_matrix("difference matrix", delta);
    }
    retval = 0;
  }

  return retval;
}

ResidualBlock::ResidualBlock(const std::string &type,
                             const std::vector<double *> &param_ptrs,
                             const std::vector<ParamBlock::Type> &param_types,
                             const int num_residuals)
    : type_{type}, param_ptrs_{param_ptrs}, param_types_{param_types} {
  // Ceres
  set_num_residuals(num_residuals);
  auto block_sizes = mutable_parameter_block_sizes();
  for (auto param_type : param_types) {
    auto param_size = ParamBlock::get_param_size(param_type);
    block_sizes->push_back(param_size);
  }
}

std::string ResidualBlock::get_type() const { return type_; }

int ResidualBlock::get_num_params() const { return param_ptrs_.size(); }

const std::vector<ParamBlock::Type> &ResidualBlock::get_param_types() const {
  return param_types_;
}

std::vector<double *> ResidualBlock::get_param_ptrs() const {
  return param_ptrs_;
}

bool ResidualBlock::Evaluate(double const *const *params,
                             double *res,
                             double **jacs) const {
  // Setup
  const int r_size = num_residuals();
  const int num_params = get_num_params();

  // Malloc minimal jacobians
  double **min_jacs = nullptr;
  if (jacs != nullptr) {
    min_jacs = (double **) calloc(num_params, sizeof(double *));
    for (int i = 0; i < num_params; i++) {
      const int local_size = ParamBlock::get_local_size(param_types_[i]);
      const auto min_jac_size = r_size * local_size;
      min_jacs[i] = (double *) calloc(min_jac_size, sizeof(double));
    }
  }

  // Evaulate
  if (res != nullptr) {
    const bool status = eval(params, res, min_jacs);
    if (status == false) {
      for (int i = 0; i < num_params; ++i) {
        free(min_jacs[i]);
      }
      free(min_jacs);
      return false;
    }
  }

  // Copy over to output jacobians
  if (jacs != nullptr) {
    for (int param_idx = 0; param_idx < num_params; ++param_idx) {
      if (jacs[param_idx] == nullptr) {
        continue;
      }

      const ParamBlock::Type param_type = param_types_[param_idx];
      const size_t param_size = ParamBlock::get_param_size(param_type);
      const size_t local_size = ParamBlock::get_local_size(param_type);

      if (param_size == local_size) {
        const auto jac_size = r_size * param_size;
        for (size_t i = 0; i < jac_size; ++i) {
          jacs[param_idx][i] = min_jacs[param_idx][i];
        }
      } else if (param_type == ParamBlock::Type::POSE ||
                 param_type == ParamBlock::Type::EXTRINSIC ||
                 param_type == ParamBlock::Type::FIDUCIAL) {
        Eigen::Map<MatXRowMajor> J(jacs[param_idx], r_size, param_size);
        Eigen::Map<MatXRowMajor> J_min(min_jacs[param_idx], r_size, local_size);
        J.setZero();
        J.block(0, 0, r_size, local_size) = J_min;
      } else {
        throw std::logic_error("Not implemented!");
      }
    }

    // Clean up
    for (int i = 0; i < num_params; ++i) {
      free(min_jacs[i]);
    }
    free(min_jacs);
  }

  return true;
}

bool ResidualBlock::check_jacobian(const int param_idx,
                                   const std::string &jac_name,
                                   const double step,
                                   const double tol,
                                   const bool verbose) const {
  // Setup
  const int r_size = num_residuals();
  const size_t num_params = param_ptrs_.size();

  // Jacobians
  double **jac_ptrs = (double **) calloc(num_params, sizeof(double *));
  for (size_t i = 0; i < num_params; i++) {
    const int local_size = ParamBlock::get_local_size(param_types_[i]);
    const auto jac_size = r_size * local_size;
    jac_ptrs[i] = (double *) calloc(jac_size, sizeof(double));
  }

  // Base-line
  VecX r = zeros(r_size, 1);
  eval(param_ptrs_.data(), r.data(), jac_ptrs);

  // Finite difference
  double *param = param_ptrs_[param_idx];
  const auto param_type = param_types_[param_idx];
  const int param_local_size = ParamBlock::get_local_size(param_type);
  MatX fdiff = zeros(r_size, param_local_size);
  VecX r_fd = zeros(r_size, 1);
  for (int i = 0; i < param_local_size; i++) {
    ParamBlock::perturb(param_type, i, step, param);
    Evaluate(param_ptrs_.data(), r_fd.data(), nullptr);
    fdiff.col(i) = (r_fd - r) / step;
    ParamBlock::perturb(param_type, i, -step, param);
  }

  // Check jacobian
  Eigen::Map<MatXRowMajor> J(jac_ptrs[param_idx], r_size, param_local_size);
  const int retval = compare_jacobian(jac_name, fdiff, J, tol, verbose);

  // Clean up
  for (size_t i = 0; i < num_params; i++) {
    free(jac_ptrs[i]);
  }
  free(jac_ptrs);

  return (retval == 0) ? true : false;
}

} // namespace cartesian
