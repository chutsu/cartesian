#include "ImuPreintegrate.hpp"

#include <cassert>
#include <cmath>

namespace cartesian {

ImuPreintegrate::ImuPreintegrate(const ImuParams &imu_params,
                                 const ImuBuffer &imu_buffer,
                                 const Vec3 &ba_i,
                                 const Vec3 &bg_i) {
  ba = ba_i;
  bg = bg_i;

  // Form Q
  auto formQ = [](const ImuParams &imu_params) {
    const Mat3 I3 = eye(3);
    MatX Q = zeros(18, 18);
    Q.block<3, 3>(0, 0) = pow(imu_params.acc_noise_density, 2) * I3;
    Q.block<3, 3>(3, 3) = pow(imu_params.gyr_noise_density, 2) * I3;
    Q.block<3, 3>(6, 6) = pow(imu_params.acc_noise_density, 2) * I3;
    Q.block<3, 3>(9, 9) = pow(imu_params.gyr_noise_density, 2) * I3;
    Q.block<3, 3>(12, 12) = pow(imu_params.bias_walk_acc, 2) * I3;
    Q.block<3, 3>(15, 15) = pow(imu_params.bias_walk_gyr, 2) * I3;
    return Q;
  };

  // Form F
  auto formF = [](const ImuBuffer &imu_buffer,
                  const int k,
                  const Quat &dq_i,
                  const Quat &dq_j,
                  const Vec3 &ba_i,
                  const Vec3 &bg_i,
                  const double dt) {
    // Setup
    const Mat3 I3 = eye(3);
    const Vec3 w_k = imu_buffer.getGyr(k);
    const Vec3 w_kp1 = imu_buffer.getGyr(k + 1);
    const Vec3 a_k = imu_buffer.getAcc(k);
    const Vec3 a_kp1 = imu_buffer.getAcc(k + 1);
    const Mat3 gyr_x = skew(0.5 * (w_k + w_kp1) - bg_i);
    const Mat3 acc_i_x = skew(a_k - ba_i);
    const Mat3 acc_j_x = skew(a_kp1 - ba_i);
    const Mat3 dC_i = dq_i.toRotationMatrix();
    const Mat3 dC_j = dq_j.toRotationMatrix();
    const double dt_sq = dt * dt;

    // Form transition matrix F
    MatX F = zeros(15, 15);
    // -- F row block 1
    F.block<3, 3>(0, 0) = I3;
    F.block<3, 3>(0, 3) = -0.25 * dC_i * acc_i_x * dt_sq;
    F.block<3, 3>(0, 3) += -0.25 * dC_j * acc_j_x * (I3 - gyr_x * dt) * dt_sq;
    F.block<3, 3>(0, 6) = I3 * dt;
    F.block<3, 3>(0, 9) = -0.25 * (dC_i + dC_j) * dt_sq;
    F.block<3, 3>(0, 12) = 0.25 * -dC_j * acc_j_x * dt_sq * -dt;
    // -- F row block 2
    F.block<3, 3>(3, 3) = I3 - gyr_x * dt;
    F.block<3, 3>(3, 12) = -I3 * dt;
    // -- F row block 3
    F.block<3, 3>(6, 3) = -0.5 * dC_i * acc_i_x * dt;
    F.block<3, 3>(6, 3) += -0.5 * dC_j * acc_j_x * (I3 - gyr_x * dt) * dt;
    F.block<3, 3>(6, 6) = I3;
    F.block<3, 3>(6, 9) = -0.5 * (dC_i + dC_j) * dt;
    F.block<3, 3>(6, 12) = 0.5 * -dC_j * acc_j_x * dt * -dt;
    // -- F row block 4
    F.block<3, 3>(9, 9) = I3;
    // -- F row block 5
    F.block<3, 3>(12, 12) = I3;

    return F;
  };

  // Form G
  auto formG = [](const ImuBuffer &imu_buffer,
                  const int k,
                  const Quat &dq_i,
                  const Quat &dq_j,
                  const Vec3 &ba_i,
                  const double dt) {
    const Mat3 I3 = eye(3);
    const Vec3 a_k = imu_buffer.getAcc(k);
    const Mat3 acc_i_x = skew(a_k - ba_i);
    const Mat3 dC_i = dq_i.toRotationMatrix();
    const Mat3 dC_j = dq_j.toRotationMatrix();
    const double dt_sq = dt * dt;

    MatX G = zeros(15, 18);
    // -- G row block 1
    G.block<3, 3>(0, 0) = 0.25 * dC_i * dt_sq;
    G.block<3, 3>(0, 3) = 0.25 * -dC_j * acc_i_x * dt_sq * 0.5 * dt;
    G.block<3, 3>(0, 6) = 0.25 * dC_j * acc_i_x * dt_sq;
    G.block<3, 3>(0, 9) = 0.25 * -dC_j * acc_i_x * dt_sq * 0.5 * dt;
    // -- G row block 2
    G.block<3, 3>(3, 3) = I3 * dt;
    G.block<3, 3>(3, 9) = I3 * dt;
    // -- G row block 3
    G.block<3, 3>(6, 0) = 0.5 * dC_i * dt;
    G.block<3, 3>(6, 3) = 0.5 * -dC_j * acc_i_x * dt * 0.5 * dt;
    G.block<3, 3>(6, 6) = 0.5 * dC_j * dt;
    G.block<3, 3>(6, 9) = 0.5 * -dC_j * acc_i_x * dt * 0.5 * dt;
    // -- G row block 4
    G.block<3, 3>(9, 12) = I3 * dt;
    // -- G row block 5
    G.block<3, 3>(12, 15) = I3 * dt;

    return G;
  };

  // Noise matrix Q
  const MatX &Q = formQ(imu_params);

  // Pre-integrate imu measuremenets
  for (int k = 0; k < (imu_buffer.size() - 1); k++) {
    // Timestep
    const timestamp_t ts_i = imu_buffer.getTimestamp(k);
    const timestamp_t ts_j = imu_buffer.getTimestamp(k + 1);
    const double dt = ts2sec(ts_j - ts_i);
    const double dt_sq = dt * dt;

    // Setup
    const Quat dq_i = dq;
    const Vec3 dr_i = dr;
    const Vec3 dv_i = dv;
    const Vec3 ba_i = ba;
    const Vec3 bg_i = bg;

    // Gyroscope measurement
    const Vec3 w_k = imu_buffer.getGyr(k);
    const Vec3 w_kp1 = imu_buffer.getGyr(k + 1);
    const Vec3 w = 0.5 * (w_k + w_kp1) - bg_i;
    const Quat dq_perturb = quat_delta(w * dt);
    const Quat dq_j = dq_i * dq_perturb;
    assert(std::abs(dq_j.norm() - 1.0) < 1e-9); // Rotating with a unit quaternion

    // Accelerometer measurement
    const Vec3 a_k = imu_buffer.getAcc(k);
    const Vec3 a_kp1 = imu_buffer.getAcc(k + 1);
    const Vec3 acc_i = dq_i * (a_k - ba_i);
    const Vec3 acc_j = dq_j * (a_kp1 - ba_i);
    const Vec3 a = 0.5 * (acc_i + acc_j);

    // Propagate IMU state using mid-point method
    const Vec3 dr_j = dr_i + dv_i * dt + 0.5 * a * dt_sq;
    const Vec3 dv_j = dv_i + a * dt;
    const Vec3 ba_j = ba_i;
    const Vec3 bg_j = bg_i;

    // Continuous time transition matrix F and input matrix G
    const MatX F = formF(imu_buffer, k, dq_i, dq_j, ba_i, bg_i, dt);
    const MatX G = formG(imu_buffer, k, dq_i, dq_j, ba_i, dt);

    // Map results
    dq = dq_j;
    dr = dr_j;
    dv = dv_j;
    ba = ba_j;
    bg = bg_j;

    // Update
    state_F = F * state_F;
    // The noise parameters are continuous time densities, so the variance of
    // a sample is Q / dt, and G is proportional to dt
    state_P = F * state_P * F.transpose();
    if (dt > 0.0) {
      state_P += G * Q * G.transpose() / dt;
    }
    Dt += dt;
  }

  // Enforce semi-positive-definite on the state covariance matrix
  state_P = (state_P + state_P.transpose()) / 2.0;
  sqrt_info = state_P.inverse().llt().matrixU();
}

void ImuPreintegrate::reset() {
  dq = Quat{1.0, 0.0, 0.0, 0.0}; // Relative rotation
  dr = Vec3{0.0, 0.0, 0.0};      // Relative position
  dv = Vec3{0.0, 0.0, 0.0};      // Relative velocity
  ba = Vec3{0.0, 0.0, 0.0};
  bg = Vec3{0.0, 0.0, 0.0};

  Dt = 0.0;                // Preintegration time period [s]
  state_F = eye(15);         // State jacobian
  state_P = zeros(15, 15); // State covariance
  sqrt_info = eye(15, 15);   // Square root information
}

} // namespace cartesian
