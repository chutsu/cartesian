#include "CalibCameraImuError.hpp"
#include "core/SO3.hpp"

namespace cartesian {

CalibCameraImuError::CalibCameraImuError(
    const Mode mode,
    const timestamp_t ts_km1,
    const timestamp_t ts_k,
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const Vec2 &z_k,
    const Mat2 &covar)
    : ResidualBlock{"CalibCameraImuError", param_ptrs, param_types, 2},
      mode_{mode}, ts_km1_{ts_km1}, ts_k_{ts_k},
      camera_geometry_{camera_geometry}, z_k_{z_k}, covar_{covar},
      info_{covar.inverse()}, sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<CalibCameraImuError>
CalibCameraImuError::create(CalibCameraImuError::Mode mode,
                            const timestamp_t ts_km1,
                            const timestamp_t ts_k,
                            const std::shared_ptr<CameraGeometry> &camera,
                            const std::shared_ptr<ImuGeometry> &imu,
                            const std::shared_ptr<CalibTargetGeometry> &target,
                            double *sensor_pose_km1,
                            double *sensor_pose_k,
                            double *target_pose,
                            double *time_delay,
                            const int point_id,
                            const Vec2 &z_k,
                            const Mat2 &covar) {
  if (mode == STANDARD) {
    std::vector<double *> param_ptrs;
    param_ptrs.push_back(sensor_pose_k); // Sensor pose T_WS at k
    param_ptrs.push_back(target_pose);   // Target pose T_WT0
    param_ptrs.push_back(target->points[point_id].data()); // Target point p_Tj
    param_ptrs.push_back(target->extrinsic.data()); // Target extrinsic T_T0Tj
    param_ptrs.push_back(imu->extrinsic.data());    // Imu extrinsic T_C0S
    param_ptrs.push_back(camera->extrinsic.data()); // Camera extrinsic T_C0Ci
    param_ptrs.push_back(camera->intrinsic.data()); // Camera intrinsic

    std::vector<ParamBlock::Type> param_types;
    param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS at k
    param_types.push_back(ParamBlock::POSE);       // Target pose T_WT0
    param_types.push_back(ParamBlock::POINT);      // Target point p_Tj
    param_types.push_back(ParamBlock::EXTRINSIC);  // Target extrinsic T_T0Tj
    param_types.push_back(ParamBlock::EXTRINSIC);  // Imu extrinsic T_C0S
    param_types.push_back(ParamBlock::EXTRINSIC);  // Camera extrinsic T_C0Ci
    param_types.push_back(ParamBlock::INTRINSIC8); // Camera intrinsic

    return std::make_shared<CalibCameraImuError>(STANDARD,
                                                 ts_km1,
                                                 ts_k,
                                                 camera,
                                                 param_ptrs,
                                                 param_types,
                                                 z_k,
                                                 covar);

  } else if (mode == POSE_INTERP) {
    std::vector<double *> param_ptrs;
    param_ptrs.push_back(sensor_pose_k); // Sensor pose T_WS at k
    param_ptrs.push_back(target_pose);   // Target pose T_WT0
    param_ptrs.push_back(target->points[point_id].data()); // Target point p_Tj
    param_ptrs.push_back(target->extrinsic.data()); // Target extrinsic T_T0Tj
    param_ptrs.push_back(imu->extrinsic.data());    // Imu extrinsic T_C0S
    param_ptrs.push_back(camera->extrinsic.data()); // Camera extrinsic T_C0Ci
    param_ptrs.push_back(camera->intrinsic.data()); // Camera intrinsic
    param_ptrs.push_back(time_delay);               // Time-delay
    param_ptrs.push_back(sensor_pose_km1);          // Sensor pose T_WS at k-1

    std::vector<ParamBlock::Type> param_types;
    param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS at k
    param_types.push_back(ParamBlock::POSE);       // Target pose T_WT0
    param_types.push_back(ParamBlock::POINT);      // Target point p_Tj
    param_types.push_back(ParamBlock::EXTRINSIC);  // Target extrinsic T_T0Tj
    param_types.push_back(ParamBlock::EXTRINSIC);  // Imu extrinsic T_C0S
    param_types.push_back(ParamBlock::EXTRINSIC);  // Camera extrinsic T_C0Ci
    param_types.push_back(ParamBlock::INTRINSIC8); // Camera intrinsic
    param_types.push_back(ParamBlock::TIME_DELAY); // Time-delay
    param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS at k-1

    return std::make_shared<CalibCameraImuError>(POSE_INTERP,
                                                 ts_km1,
                                                 ts_k,
                                                 camera,
                                                 param_ptrs,
                                                 param_types,
                                                 z_k,
                                                 covar);
  }

  throw std::runtime_error("Invalid CalibCameraImuError mode!");
}

bool CalibCameraImuError::valid() const { return valid_; }

Mat2 CalibCameraImuError::get_covariance_matrix() const { return covar_; }

bool CalibCameraImuError::get_residuals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool CalibCameraImuError::get_reproj_error(double *error) const {
  *error = residuals_.norm();
  return valid_;
}

bool CalibCameraImuError::eval_std(double const *const *params,
                                   double *res,
                                   double **jacs) const {
  // Map parameters out
  const Mat4 T_WS_k = tf(params[0]);               // Sensor pose
  const Mat4 T_WT0 = tf(params[1]);                // Target pose
  const Eigen::Map<const Vec3> p_Tj(params[2], 3); // Target point
  const Mat4 T_T0Tj = tf(params[3]);               // Target extrinsic
  const Mat4 T_C0S = tf(params[4]);                // Imu extrinsic
  const Mat4 T_C0Ci = tf(params[5]);               // Camera extrinsic
  Eigen::Map<const VecX> intrinsic(params[6], 8);  // Camera parameters

  // Transform and project point to image plane
  // -- Transform point from target frame to camera
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_SW = tf_inv(T_WS_k);
  const Mat4 T_CiTj = T_CiC0 * T_C0S * T_SW * T_WT0 * T_T0Tj;
  const Vec3 p_Ci = tf_point(T_CiTj, p_Tj);
  // -- Project point from camera frame to image plane
  valid_ = true;
  const auto camera_model = camera_geometry_->camera_model;
  const Vec2i resolution = camera_geometry_->resolution;
  Vec2 z_hat;
  if (camera_model->project(resolution, intrinsic, p_Ci, z_hat) != 0) {
    valid_ = false;
    Eigen::Map<Vec2> r(res);
    r.setZero();
    residuals_.setZero();
    return true;
  }

  // Residual
  Eigen::Map<Vec2> r(res);
  r = sqrt_info_ * (z_k_ - z_hat);
  residuals_ = z_k_ - z_hat;

  // Jacobians
  const MatX Jh = camera_model->project_jacobian(intrinsic, p_Ci);
  const MatX Jhw = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr || !valid_) {
    return true;
  }

  // -- Jacobians w.r.t imu pose T_WS_k
  if (jacs[0]) {
    const Vec3 r_WS = tf_trans(T_WS_k);
    const Mat3 C_WS = tf_rot(T_WS_k);
    const Mat4 T_CiS = T_CiC0 * T_C0S;
    const Mat4 T_CiW = T_CiS * T_SW;
    const Mat3 C_CiW = tf_rot(T_CiW);
    const Vec3 p_W = tf_point(T_WT0 * T_T0Tj, p_Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * -C_CiW;
      J.block<2, 3>(0, 3) = Jhw * -C_CiW * skew(p_W - r_WS) * -C_WS;
    }
  }

  // -- Jacobians w.r.t target pose T_WT0
  if (jacs[1]) {
    const Mat4 T_CiS = T_CiC0 * T_C0S;
    const Mat3 C_CiW = tf_rot(T_CiS * T_SW);
    const Mat3 C_WT0 = tf_rot(T_WT0);
    const Vec3 p_T0 = tf_point(T_T0Tj, p_Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * C_CiW;
      J.block<2, 3>(0, 3) = Jhw * C_CiW * -C_WT0 * skew(p_T0);
    }
  }

  // -- Jacobians w.r.t target point p_Tj
  if (jacs[2]) {
    Eigen::Map<Mat<2, 3, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();

    if (valid_) {
      const Mat3 C_CiTj = tf_rot(T_CiTj);
      J = Jhw * C_CiTj;
    }
  }

  // -- Jacobians w.r.t target extrinsic T_T0Tj
  if (jacs[3]) {
    const Mat4 T_CiT0 = T_CiC0 * T_C0S * T_SW * T_WT0;
    const Mat3 C_CiT0 = tf_rot(T_CiT0);
    const Mat3 C_T0Tj = tf_rot(T_T0Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * C_CiT0;
      J.block<2, 3>(0, 3) = Jhw * C_CiT0 * -C_T0Tj * skew(p_Tj);
    }
  }

  // -- Jacobians w.r.t imu extrinsic T_C0S
  if (jacs[4]) {
    const Vec3 p_S = tf_point(T_WS_k.inverse() * T_WT0 * T_T0Tj, p_Tj);
    const Mat3 C_C0S = tf_rot(T_C0S);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[4]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * C_CiC0;
      J.block<2, 3>(0, 3) = Jhw * C_CiC0 * -C_C0S * skew(p_S);
    }
  }

  // -- Jacobians w.r.t camera extrinsic T_C0Ci
  if (jacs[5]) {
    const Vec3 p_C0 = tf_point(T_C0S * T_WS_k.inverse() * T_WT0 * T_T0Tj, p_Tj);
    const Vec3 r_C0Ci = tf_trans(T_C0Ci);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);
    const Mat3 C_C0Ci = C_CiC0.transpose();

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[5]);
    J.setZero();
    if (valid_) {
      J.block<2, 3>(0, 0) = Jhw * -C_CiC0;
      J.block<2, 3>(0, 3) = Jhw * -C_CiC0 * skew(p_C0 - r_C0Ci) * -C_C0Ci;
    }
  }

  // -- Jacobians w.r.t intrinsic
  if (jacs[6]) {
    Eigen::Map<Mat<2, 8, Eigen::RowMajor>> J(jacs[6]);
    J.setZero();
    if (valid_) {
      J = -1.0 * sqrt_info_ * camera_model->params_jacobian(intrinsic, p_Ci);
    }
  }

  return true;
}

bool CalibCameraImuError::eval_pi(double const *const *params,
                                  double *res,
                                  double **jacs) const {
  // Map parameters out
  const Mat4 T_WS_k = tf(params[0]);               // Sensor pose
  const Mat4 T_WT0 = tf(params[1]);                // Target pose
  const Eigen::Map<const Vec3> p_Tj(params[2], 3); // Target point
  const Mat4 T_T0Tj = tf(params[3]);               // Target extrinsic
  const Mat4 T_C0S = tf(params[4]);                // Imu extrinsic
  const Mat4 T_C0Ci = tf(params[5]);               // Camera extrinsic
  Eigen::Map<const VecX> intrinsic(params[6], 8);  // Camera parameters
  const double time_delay = params[7][0];          // Time delay
  const Mat4 T_WS_km1 = tf(params[8]);

  // Extrapolate the pose at k, to k + time_delay, from the motion between k-1
  // and k (constant velocity, constant angular velocity)
  //
  //   r_delayed = r_k + alpha * (r_k - r_km1)
  //   C_delayed = C_k * Exp(alpha * w),  w = Log(C_km1' * C_k)
  //
  // where alpha = time_delay / dt, and w the rotation from k-1 to k in the
  // sensor frame
  const double dt = ts2sec(ts_k_ - ts_km1_);
  const double alpha = time_delay / dt;
  const Vec3 r_km1 = tf_trans(T_WS_km1);
  const Vec3 r_k = tf_trans(T_WS_k);
  const Mat3 C_km1 = tf_rot(T_WS_km1);
  const Mat3 C_k = tf_rot(T_WS_k);
  const Mat3 C_km1_k = C_km1.transpose() * C_k;
  const Vec3 w = SO3::log(C_km1_k);
  const Mat3 R_alpha = SO3::exp(alpha * w);
  const Mat3 C_delayed = C_k * R_alpha;
  const Vec3 r_delayed = r_k + alpha * (r_k - r_km1);
  const Mat4 T_WS_delayed = tf(C_delayed, r_delayed);

  // Transform and project point to image plane
  valid_ = true;
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_SW = tf_inv(T_WS_delayed);
  const Mat4 T_CiTj = T_CiC0 * T_C0S * T_SW * T_WT0 * T_T0Tj;
  const Vec3 p_Ci = tf_point(T_CiTj, p_Tj);
  const auto camera_model = camera_geometry_->camera_model;
  const Vec2i resolution = camera_geometry_->resolution;
  Vec2 z_hat;
  if (camera_model->project(resolution, intrinsic, p_Ci, z_hat) != 0) {
    valid_ = false;
    Eigen::Map<Vec2> r(res);
    r.setZero();
    residuals_.setZero();
    return true;
  }

  // Residuals
  Eigen::Map<Vec2> r(res);
  r = sqrt_info_ * (z_k_ - z_hat);
  residuals_ = z_k_ - z_hat;

  // Jacobians
  const Mat4 T_CiS = T_CiC0 * T_C0S;
  const Mat4 T_CiW = T_CiS * T_SW;
  const Mat3 C_CiW = tf_rot(T_CiW);
  const Vec3 p_W = tf_point(T_WT0 * T_T0Tj, p_Tj);
  const MatX Jh = camera_model->project_jacobian(intrinsic, p_Ci);
  const MatX Jhw = -1.0 * sqrt_info_ * Jh;
  // Evaluate now: `auto` would keep an expression that refers to the temporary
  // matrix of skew(), which is gone after this statement
  const Mat<2, 3> J_std_trans = Jhw * -C_CiW;
  const Mat<2, 3> J_std_rot =
      Jhw * -C_CiW * skew(p_W - r_delayed) * -C_delayed;
  if (jacs == nullptr || !valid_) {
    return true;
  }

  // -- Jacobians w.r.t sensor pose T_WS_k
  //
  // A rotation perturbation of C_k (C_k Exp(dtheta)) changes w by
  // Jr^-1(w) dtheta, so the delayed rotation changes, in its own frame, by
  //
  //   R_alpha' dtheta + alpha Jr(alpha w) Jr^-1(w) dtheta
  //
  // For C_km1 the same, with C_km1' C_k Exp(-dtheta) = Exp(-C_km1_k' dtheta)
  const Mat3 J_r_alpha_w = SO3::right_jacobian(alpha * w);
  const Mat3 J_r_inv_w = SO3::right_jacobian_inv(w);
  const Mat3 D = alpha * J_r_alpha_w * J_r_inv_w;
  if (jacs[0]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();
    J.block<2, 3>(0, 0) = J_std_trans * (1.0 + alpha);
    J.block<2, 3>(0, 3) = J_std_rot * (R_alpha.transpose() + D);
  }

  // -- Jacobians w.r.t target pose T_WT0
  if (jacs[1]) {
    const Mat3 C_WT0 = tf_rot(T_WT0);
    const Vec3 p_T0 = tf_point(T_T0Tj, p_Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * C_CiW;
    J.block<2, 3>(0, 3) = Jhw * C_CiW * -C_WT0 * skew(p_T0);
  }

  // -- Jacobians w.r.t target point p_Tj
  if (jacs[2]) {
    Eigen::Map<Mat<2, 3, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();
    const Mat3 C_CiTj = tf_rot(T_CiTj);
    J = Jhw * C_CiTj;
  }

  // -- Jacobians w.r.t target extrinsic T_T0Tj
  if (jacs[3]) {
    const Mat4 T_CiT0 = T_CiC0 * T_C0S * T_SW * T_WT0;
    const Mat3 C_CiT0 = tf_rot(T_CiT0);
    const Mat3 C_T0Tj = tf_rot(T_T0Tj);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * C_CiT0;
    J.block<2, 3>(0, 3) = Jhw * C_CiT0 * -C_T0Tj * skew(p_Tj);
  }

  // -- Jacobians w.r.t imu extrinsic T_C0S
  if (jacs[4]) {
    const Vec3 p_S = tf_point(T_SW * T_WT0 * T_T0Tj, p_Tj);
    const Mat3 C_C0S = tf_rot(T_C0S);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[4]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * C_CiC0;
    J.block<2, 3>(0, 3) = Jhw * C_CiC0 * -C_C0S * skew(p_S);
  }

  // -- Jacobians w.r.t camera extrinsic T_C0Ci
  if (jacs[5]) {
    const Vec3 p_C0 = tf_point(T_C0S * T_SW * T_WT0 * T_T0Tj, p_Tj);
    const Vec3 r_C0Ci = tf_trans(T_C0Ci);
    const Mat3 C_CiC0 = tf_rot(T_CiC0);
    const Mat3 C_C0Ci = C_CiC0.transpose();

    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[5]);
    J.setZero();
    J.block<2, 3>(0, 0) = Jhw * -C_CiC0;
    J.block<2, 3>(0, 3) = Jhw * -C_CiC0 * skew(p_C0 - r_C0Ci) * -C_C0Ci;
  }

  // -- Jacobians w.r.t intrinsic
  if (jacs[6]) {
    Eigen::Map<Mat<2, 8, Eigen::RowMajor>> J(jacs[6]);
    J.setZero();
    J = -1.0 * sqrt_info_ * camera_model->params_jacobian(intrinsic, p_Ci);
  }

  // -- Time-delay Jacobian
  if (jacs[7]) {
    Eigen::Map<Vec2> J(jacs[7]);
    J.setZero();
    const Vec3 v_trans = (r_k - r_km1) / dt;
    const Vec3 d_rot_dt = J_r_alpha_w * w / dt;
    J = J_std_trans * v_trans + J_std_rot * d_rot_dt;
  }

  // -- Jacobian w.r.t. Sensor pose T_WS_km1
  if (jacs[8]) {
    Eigen::Map<Mat<2, 6, Eigen::RowMajor>> J(jacs[8]);
    J.setZero();
    J.block<2, 3>(0, 0) = J_std_trans * (-alpha);
    J.block<2, 3>(0, 3) = J_std_rot * (-D * C_km1_k.transpose());
  }

  return true;
}

bool CalibCameraImuError::eval(double const *const *params,
                               double *res,
                               double **jacs) const {
  switch (mode_) {
    case STANDARD: return eval_std(params, res, jacs);
    case POSE_INTERP: return eval_pi(params, res, jacs);
    default: throw std::runtime_error("Invalid mode!");
  }
}

} // namespace cartesian
