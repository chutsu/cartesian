#include "CalibCameraError.hpp"

namespace cartesian {

CalibCameraError::CalibCameraError(
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const Vec2 &z,
    const Mat2 &covar)
    : ResidualBlock{"CalibCameraError", param_ptrs, param_types, 2},
      camera_geometry_{camera_geometry}, z_{z}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<CalibCameraError>
CalibCameraError::create(const std::shared_ptr<CameraGeometry> &camera,
                         const std::shared_ptr<CalibTargetGeometry> &target,
                         const int point_id,
                         double *T_C0T0,
                         const Vec2 &z,
                         const Mat2 &covar) {
  std::vector<double *> param_ptrs;
  param_ptrs.push_back(T_C0T0);                          // Relative pose T_C0T0
  param_ptrs.push_back(target->points[point_id].data()); // Target point p_Tj
  param_ptrs.push_back(target->extrinsic.data()); // Target extrinsic T_T0Tj
  param_ptrs.push_back(camera->extrinsic.data()); // Camera extrinsic T_C0Ci
  param_ptrs.push_back(camera->intrinsic.data()); // Camera intrinsic

  std::vector<ParamBlock::Type> param_types;
  param_types.push_back(ParamBlock::POSE);      // Relative pose T_C0T0
  param_types.push_back(ParamBlock::POINT);     // Target point p_Tj
  param_types.push_back(ParamBlock::EXTRINSIC); // Target extrinsic T_T0Tj
  param_types.push_back(ParamBlock::EXTRINSIC); // Camera extrinsic T_C0Ci
  switch (camera->intrinsic.size()) {
    case 8:
      param_types.push_back(ParamBlock::INTRINSIC8); // Camera intrinsic
      break;
    case 9:
      param_types.push_back(ParamBlock::INTRINSIC9); // Camera intrinsic
      break;
    default:
      FATAL("Unsupported number of camera intrinsic parameters [%ld]!",
            camera->intrinsic.size());
  }

  return std::make_shared<CalibCameraError>(camera,
                                            param_ptrs,
                                            param_types,
                                            z,
                                            covar);
}

bool CalibCameraError::valid() const { return valid_; }

Mat2 CalibCameraError::get_covariance_matrix() const { return covar_; }

bool CalibCameraError::get_residuals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool CalibCameraError::get_reproj_error(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool CalibCameraError::eval(double const *const *params,
                            double *res,
                            double **jacs) const {
  // Map parameters out
  const Mat4 T_C0T0 = tf(params[0]);
  const Eigen::Map<const Vec3> p_Tj(params[1], 3);
  const Mat4 T_T0Tj = tf(params[2]);
  const Mat4 T_C0Ci = tf(params[3]);
  const int intrinsic_size = camera_geometry_->intrinsic.size();
  const Eigen::Map<const VecX> intrinsic(params[4], intrinsic_size);

  // Transform and project point to image plane
  valid_ = true;
  // -- Transform point from fiducial frame to camera-n
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_CiTj = T_CiC0 * T_C0T0 * T_T0Tj;
  const Vec3 p_Ci = tf_point(T_CiTj, p_Tj);
  // -- Project point from camera frame to image plane
  const auto camera_model = camera_geometry_->camera_model;
  const Vec2i resolution = camera_geometry_->resolution;
  Vec2 z_hat;
  if (camera_model->project(resolution, intrinsic, p_Ci, z_hat) != 0) {
    valid_ = false;
  }

  // Residual
  Eigen::Map<Vec2> r(res);
  r = sqrt_info_ * (z_ - z_hat);
  residuals_ = z_ - z_hat;

  // Jacobians
  const MatX Jh = camera_model->project_jacobian(intrinsic, p_Ci);
  const MatX Jhw = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr) {
    return true;
  }

  // -- Jacobians w.r.t relative camera pose T_C0T0
  if (jacs[0]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();

    if (valid_) {
      const Mat3 C_CiC0 = tf_rot(T_CiC0);
      const Mat3 C_C0T0 = tf_rot(T_C0T0);
      const Vec3 p_T0 = tf_point(T_T0Tj, p_Tj);
      J.block(0, 0, 2, 3) = Jhw * C_CiC0;
      J.block(0, 3, 2, 3) = Jhw * C_CiC0 * -C_C0T0 * skew(p_T0);
    }
  }

  // -- Jacobians w.r.t target point p_Tj
  if (jacs[1]) {
    Eigen::Map<Mat<2, 3, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();

    if (valid_) {
      const Mat3 C_CiTj = tf_rot(T_CiTj);
      J = Jhw * C_CiTj;
    }
  }

  // -- Jacobians w.r.t target extrinsic T_T0Tj
  if (jacs[2]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();

    if (valid_) {
      const Mat3 C_CiT0 = tf_rot(T_CiC0 * T_C0T0);
      const Mat3 C_T0Tj = tf_rot(T_T0Tj);
      J.block(0, 0, 2, 3) = Jhw * C_CiT0;
      J.block(0, 3, 2, 3) = Jhw * C_CiT0 * -C_T0Tj * skew(p_Tj);
    }
  }

  // -- Jacobians w.r.t camera extrinsic T_C0Ci
  if (jacs[3]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();

    if (valid_) {
      const Mat3 C_CiC0 = tf_rot(T_CiC0);
      const Mat3 C_C0Ci = C_CiC0.transpose();
      const Vec3 p_C0 = tf_point(T_C0T0 * T_T0Tj, p_Tj);
      const Vec3 r_C0Ci = tf_trans(T_C0Ci);

      J.block(0, 0, 2, 3) = Jhw * -C_CiC0;
      J.block(0, 3, 2, 3) = Jhw * -C_CiC0 * skew(p_C0 - r_C0Ci) * -C_C0Ci;
    }
  }

  // -- Jacobians w.r.t camera intrinsic
  if (jacs[4]) {
    Eigen::Map<MatXRowMajor> J(jacs[4], 2, intrinsic_size);
    J.setZero();

    if (valid_) {
      J = -1 * sqrt_info_ * camera_model->params_jacobian(intrinsic, p_Ci);
    }
  }

  return true;
}

} // namespace cartesian
