#include "ImuError.hpp"

namespace cartesian {

void ImuError::propagate() {
  // Pre-integrate around the biases of the first state
  Eigen::Map<const VecX> sb_i(param_ptrs_[1], 9);
  const Vec3 ba_i{sb_i[3], sb_i[4], sb_i[5]};
  const Vec3 bg_i{sb_i[6], sb_i[7], sb_i[8]};
  const ImuPreintegrate pint{imu_params, imu_buffer, ba_i, bg_i};

  dq_ = pint.dq;
  dr_ = pint.dr;
  dv_ = pint.dv;
  ba_ = pint.ba;
  bg_ = pint.bg;
  Dt_ = pint.Dt;
  state_F_ = pint.state_F;
  state_P_ = pint.state_P;

  // The covariance is ordered like F and G (position, rotation, velocity,
  // accel bias, gyro bias), but the residual is ordered (position, velocity,
  // rotation, accel bias, gyro bias). Swap the rotation and velocity blocks
  // before inverting, so the square root information whitens the residual.
  MatX P = state_P_;
  P.block<3, 15>(3, 0).swap(P.block<3, 15>(6, 0)); // Swap rows
  P.block<15, 3>(0, 3).swap(P.block<15, 3>(0, 6)); // Swap columns
  sqrt_info_ = P.inverse().llt().matrixU();
}

ImuError::ImuError(const std::vector<double *> &param_ptrs_,
                   const std::vector<ParamBlock::Type> &param_types_,
                   const ImuParams &imu_params_,
                   const ImuBuffer &imu_buffer_)
    : ResidualBlock{"ImuError", param_ptrs_, param_types_, 15},
      imu_params{imu_params_}, imu_buffer{imu_buffer_} {
  assert(imu_params.acc_noise_density > 0);
  assert(imu_params.gyr_noise_density > 0);
  assert(imu_params.bias_walk_acc > 0);
  assert(imu_params.bias_walk_gyr > 0);
  assert(imu_params.g.norm() > 0);
  propagate();
}

void ImuError::set_sqrt_info(const MatX &sqrt_info) { sqrt_info_ = sqrt_info; }

MatX ImuError::get_matrix_f() const { return state_F_; }

MatX ImuError::get_matrix_p() const { return state_P_; }

Quat ImuError::get_relative_rotation() const { return dq_; }

Vec3 ImuError::get_relative_position() const { return dr_; }

Vec3 ImuError::get_relative_velocity() const { return dv_; }

std::shared_ptr<ImuError> ImuError::create(const ImuParams &imu_params_,
                                           const ImuBuffer &imu_buffer_,
                                           double *pose_i,
                                           double *sb_i,
                                           double *pose_j,
                                           double *sb_j) {
  std::vector<double *> param_ptrs = {pose_i, sb_i, pose_j, sb_j};
  std::vector<ParamBlock::Type> param_types = {ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES,
                                               ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES};
  return std::make_shared<ImuError>(param_ptrs,
                                    param_types,
                                    imu_params_,
                                    imu_buffer_);
}

bool ImuError::eval(double const *const *params,
                    double *res,
                    double **jacs) const {
  // Map parameters out
  const Mat4 T_i = tf(params[0]);
  Eigen::Map<const VecX> sb_i(params[1], 9);
  const Mat4 T_j = tf(params[2]);
  Eigen::Map<const VecX> sb_j(params[3], 9);

  const Quat &q_i = tf_quat(T_i);
  const Mat3 &C_i = tf_rot(T_i);
  const Vec3 &r_i = tf_trans(T_i);
  const Vec3 &v_i = sb_i.segment<3>(0);
  const Vec3 &ba_i = sb_i.segment<3>(3);
  const Vec3 &bg_i = sb_i.segment<3>(6);

  const Quat &q_j = tf_quat(T_j);
  const Mat3 &C_j = tf_rot(T_j);
  const Vec3 &r_j = tf_trans(T_j);
  const Vec3 &v_j = sb_j.segment<3>(0);
  const Vec3 &ba_j = sb_j.segment<3>(3);
  const Vec3 &bg_j = sb_j.segment<3>(6);

  // Correct the relative position, velocity and orientation
  // -- Extract jacobians from error-state jacobian
  const Mat3 &dr_dba = state_F_.block<3, 3>(0, 9);
  const Mat3 &dr_dbg = state_F_.block<3, 3>(0, 12);
  const Mat3 &dq_dbg = state_F_.block<3, 3>(3, 12);
  const Mat3 &dv_dba = state_F_.block<3, 3>(6, 9);
  const Mat3 &dv_dbg = state_F_.block<3, 3>(6, 12);
  const Vec3 &dba = ba_i - ba_;
  const Vec3 &dbg = bg_i - bg_;
  const Vec3 &dhtheta = 0.5 * (dq_dbg * dbg);
  const Quat dq_theta{1.0, dhtheta.x(), dhtheta.y(), dhtheta.z()};

  // -- Correct the relative position, velocity and rotation
  const auto dr = dr_ + dr_dba * dba + dr_dbg * dbg;
  const auto dv = dv_ + dv_dba * dba + dv_dbg * dbg;
  const auto dq = dq_ * dq_theta;

  // Form residuals
  const double Dt_sq = Dt_ * Dt_;
  const Vec3 &g = imu_params.g;
  const Mat3 &C_iT = C_i.transpose();
  const Quat &dq_inv = dq.inverse();
  const Quat &qi_inv = q_i.inverse();

  const Vec3 &dr_est = C_iT * ((r_j - r_i) - (v_i * Dt_) + (0.5 * g * Dt_sq));
  const Vec3 &dv_est = C_iT * ((v_j - v_i) + (g * Dt_));
  const Quat &dq_est = qi_inv * q_j;

  const Vec3 &err_pos = dr_est - dr;
  const Vec3 &err_vel = dv_est - dv;
  const Vec3 &err_rot = 2.0 * (dq_inv * dq_est).vec();
  const Vec3 &err_ba = ba_j - ba_i;
  const Vec3 &err_bg = bg_j - bg_i;

  Eigen::Map<VecX> r(res, 15);
  r.segment(0, 3) = err_pos;
  r.segment(3, 3) = err_vel;
  r.segment(6, 3) = err_rot;
  r.segment(9, 3) = err_ba;
  r.segment(12, 3) = err_bg;
  r = sqrt_info_ * r;

  // Form Jacobians
  if (jacs == nullptr) {
    return true;
  }

  // Jacobian w.r.t pose i
  if (jacs[0]) {
    const Quat q_ji{C_j.transpose() * C_i};
    const Mat4 q_left_dq_right = quat_left(q_ji) * quat_right(dq);
    const Mat3 dtheta_dCi = -(q_left_dq_right).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 6, Eigen::RowMajor>> J(jacs[0]);
    J.setZero();
    J.block<3, 3>(0, 0) = -C_iT;        // dr w.r.t r_i
    J.block<3, 3>(0, 3) = skew(dr_est); // dr w.r.t C_i
    J.block<3, 3>(3, 3) = skew(dv_est); // dv w.r.t C_i
    J.block<3, 3>(6, 3) = dtheta_dCi;   // dtheta w.r.t C_i
    J = sqrt_info_ * J;
  }

  // Jacobian w.r.t speed and biases i
  if (jacs[1]) {
    const Quat dq_ji{C_j.transpose() * C_i * dq.toRotationMatrix()};
    const Mat3 dQ_left_xyz = quat_left(dq_ji).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 9, Eigen::RowMajor>> J(jacs[1]);
    J.setZero();
    J.block<3, 3>(0, 0) = -C_i.transpose() * Dt_; // dr w.r.t v_i
    J.block<3, 3>(0, 3) = -dr_dba;                // dr w.r.t ba
    J.block<3, 3>(0, 6) = -dr_dbg;                // dr w.r.t bg
    J.block<3, 3>(3, 0) = -C_i.transpose();       // dv w.r.t v_i
    J.block<3, 3>(3, 3) = -dv_dba;                // dv w.r.t ba
    J.block<3, 3>(3, 6) = -dv_dbg;                // dv w.r.t bg
    J.block<3, 3>(6, 6) = -dQ_left_xyz * dq_dbg;  // dtheta w.r.t C_i
    J.block<3, 3>(9, 3) = -eye(3);
    J.block<3, 3>(12, 6) = -eye(3);
    J = sqrt_info_ * J;
  }

  // Jacobian w.r.t. pose j
  if (jacs[2]) {
    const Quat error_rot = dq.inverse() * (q_i.inverse() * q_j);
    const Mat3 dtheta_dCj = quat_left(error_rot).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 6, Eigen::RowMajor>> J(jacs[2]);
    J.setZero();
    J.block<3, 3>(0, 0) = C_i.transpose(); // dr w.r.t r_j
    J.block<3, 3>(6, 3) = dtheta_dCj;      // dtheta w.r.t C_j
    J = sqrt_info_ * J;
  }

  //  Jacobian w.r.t. speed and biases j
  if (jacs[3]) {
    Eigen::Map<Mat<15, 9, Eigen::RowMajor>> J(jacs[3]);
    J.setZero();
    J.block<3, 3>(3, 0) = C_i.transpose(); // dv w.r.t v_j
    J.block<3, 3>(9, 3) = eye(3);
    J.block<3, 3>(12, 6) = eye(3);
    J = sqrt_info_ * J;
  }

  return true;
}

} // namespace cartesian
