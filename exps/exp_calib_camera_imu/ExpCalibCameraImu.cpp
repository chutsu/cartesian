/**
 * Camera-IMU calibration benchmark: the time delay
 *
 * Simulates a camera-IMU calibration dataset (a camera moving in front of an
 * AprilGrid, with the IMU measurements it would make) where the IMU time stamps
 * are late by a known time delay, and puts it through `CalibCameraImu` with
 * each of the time delay methods, to see how well the time delay and the
 * camera-IMU extrinsic are recovered, and how that depends on the delay and on
 * the noise of the pixels and of the IMU.
 *
 * Settings are read from a YAML file, see config.yaml.
 *
 * Usage: exp_calib_camera_imu <config.yaml>
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <map>
#include <random>

#include "calib/AprilGrid.hpp"
#include "calib/CalibCameraImu.hpp"
#include "calib/CalibTargetGeometry.hpp"
#include "camera/CameraGeometry.hpp"
#include "ceres/ResidualBlock.hpp"
#include "core/SO3.hpp"
#include "imu/ImuBuffer.hpp"
#include "imu/ImuGeometry.hpp"
#include "imu/ImuParams.hpp"
#include "imu/ImuPreintegrate.hpp"
#include "sim/SimCalib.hpp"

using namespace cartesian;

/******************************************************************************
 * SETTINGS
 *****************************************************************************/

/** Noise of the simulated measurements */
struct NoiseLevel {
  std::string name;
  double pixel;     // Std of the noise of the corners [px]
  double imu;       // Noise of the IMU, as a fraction of its noise density
  bool clean() const { return pixel == 0.0 && imu == 0.0; }
};

struct Settings {
  fs::path output_dir;
  unsigned int seed;
  int num_seeds;
  double camera_rate;
  std::string trajectory;
  double radius;
  double duration;
  int max_iters;
  double tolerance_ms;
  Vec3 init_position;
  Vec3 init_rotation_deg;
  std::vector<std::string> td_methods;
  std::vector<double> true_delays_ms;
  std::vector<NoiseLevel> noise_levels;

  Settings(const std::string &config_file) {
    config_t config{config_file};
    parse(config, "output_dir", output_dir);
    parse(config, "seed", seed);
    parse(config, "num_seeds", num_seeds);
    parse(config, "camera_rate", camera_rate);
    parse(config, "trajectory.type", trajectory);
    parse(config, "trajectory.radius", radius);
    parse(config, "trajectory.duration", duration);
    parse(config, "max_iters", max_iters);
    parse(config, "tolerance_ms", tolerance_ms);
    parse(config, "init_extrinsic.position", init_position);
    parse(config, "init_extrinsic.rotation_deg", init_rotation_deg);
    parse(config, "td_methods", td_methods, false);
    parse(config, "true_delays_ms", true_delays_ms, false);

    for (const auto &node : config.root["noise_levels"]) {
      NoiseLevel level;
      level.name = node.first.as<std::string>();
      level.pixel = node.second["pixel"].as<double>();
      level.imu = node.second["imu"].as<double>();
      noise_levels.push_back(level);
    }
  }
};

/******************************************************************************
 * CAMERA ERROR WITH THE PIXEL VELOCITY
 *****************************************************************************/

/**
 * Camera error with the pixel velocity
 *
 * The reprojection error of a target point measured at `z_k` in the view at
 * `ts_k`, with the measurement corrected for the time delay `td` [s] between
 * the camera and the imu by the velocity of the pixel, the difference between
 * its measurement in this view and in the previous one at `ts_km1`:
 *
 *   z_corrected = z_k - td * (z_k - z_km1) / (ts_k - ts_km1)
 *
 * Parameter blocks: sensor pose at k, target pose, target point, target
 * extrinsic, imu extrinsic, camera extrinsic, camera intrinsics and the time
 * delay.
 */
class CalibCameraImuErrorPixelVelocity : public ResidualBlock {
private:
  std::shared_ptr<CameraGeometry> camera_geometry_;
  Vec2 z_k_;
  Vec2 v_k_; // Pixel velocity [px / s]
  Mat2 covar_;
  Mat2 info_;
  Mat2 sqrt_info_;
  mutable bool valid_ = true;
  mutable Vec2 residuals_;

public:
  CalibCameraImuErrorPixelVelocity(
      const std::shared_ptr<CameraGeometry> &camera_geometry,
      const std::vector<double *> &param_ptrs,
      const std::vector<ParamBlock::Type> &param_types,
      const Vec2 &z_k,
      const Vec2 &v_k,
      const Mat2 &covar);

  /** Create residual block of the point `point_id`, seen at `z_km1` at `ts_km1` */
  static std::shared_ptr<CalibCameraImuErrorPixelVelocity>
  create(const timestamp_t ts_km1,
         const timestamp_t ts_k,
         const std::shared_ptr<CameraGeometry> &camera,
         const std::shared_ptr<ImuGeometry> &imu,
         const std::shared_ptr<CalibTargetGeometry> &target,
         double *sensor_pose_k,
         double *target_pose,
         double *time_delay,
         const int point_id,
         const Vec2 &z_km1,
         const Vec2 &z_k,
         const Mat2 &covar = eye(2));

  bool valid() const;
  bool get_residuals(Vec2 &r) const;

  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

CalibCameraImuErrorPixelVelocity::CalibCameraImuErrorPixelVelocity(
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const std::vector<double *> &param_ptrs,
    const std::vector<ParamBlock::Type> &param_types,
    const Vec2 &z_k,
    const Vec2 &v_k,
    const Mat2 &covar)
    : ResidualBlock{"CalibCameraImuErrorPixelVelocity",
                    param_ptrs,
                    param_types,
                    2},
      camera_geometry_{camera_geometry}, z_k_{z_k}, v_k_{v_k}, covar_{covar}, info_{covar.inverse()},
      sqrt_info_{info_.llt().matrixU()} {}

std::shared_ptr<CalibCameraImuErrorPixelVelocity>
CalibCameraImuErrorPixelVelocity::create(
    const timestamp_t ts_km1,
    const timestamp_t ts_k,
    const std::shared_ptr<CameraGeometry> &camera,
    const std::shared_ptr<ImuGeometry> &imu,
    const std::shared_ptr<CalibTargetGeometry> &target,
    double *sensor_pose_k,
    double *target_pose,
    double *time_delay,
    const int point_id,
    const Vec2 &z_km1,
    const Vec2 &z_k,
    const Mat2 &covar) {
  std::vector<double *> param_ptrs;
  param_ptrs.push_back(sensor_pose_k); // Sensor pose T_WS at k
  param_ptrs.push_back(target_pose);   // Target pose T_WT0
  param_ptrs.push_back(target->points[point_id].data()); // Target point p_Tj
  param_ptrs.push_back(target->extrinsic.data()); // Target extrinsic T_T0Tj
  param_ptrs.push_back(imu->extrinsic.data());    // Imu extrinsic T_C0S
  param_ptrs.push_back(camera->extrinsic.data()); // Camera extrinsic T_C0Ci
  param_ptrs.push_back(camera->intrinsic.data()); // Camera intrinsic
  param_ptrs.push_back(time_delay);               // Time-delay

  std::vector<ParamBlock::Type> param_types;
  param_types.push_back(ParamBlock::POSE);       // Sensor pose T_WS at k
  param_types.push_back(ParamBlock::POSE);       // Target pose T_WT0
  param_types.push_back(ParamBlock::POINT);      // Target point p_Tj
  param_types.push_back(ParamBlock::EXTRINSIC);  // Target extrinsic T_T0Tj
  param_types.push_back(ParamBlock::EXTRINSIC);  // Imu extrinsic T_C0S
  param_types.push_back(ParamBlock::EXTRINSIC);  // Camera extrinsic T_C0Ci
  param_types.push_back(ParamBlock::INTRINSIC8); // Camera intrinsic
  param_types.push_back(ParamBlock::TIME_DELAY); // Time-delay

  const Vec2 v_k = (z_k - z_km1) / ts2sec(ts_k - ts_km1);
  return std::make_shared<CalibCameraImuErrorPixelVelocity>(camera,
                                                            param_ptrs,
                                                            param_types,
                                                            z_k,
                                                            v_k,
                                                            covar);
}

bool CalibCameraImuErrorPixelVelocity::valid() const { return valid_; }

bool CalibCameraImuErrorPixelVelocity::get_residuals(Vec2 &r) const {
  r = residuals_;
  return valid_;
}

bool CalibCameraImuErrorPixelVelocity::eval(double const *const *params,
                                            double *res,
                                            double **jacs) const {
  // Map shared parameters
  const Mat4 T_WS_k = tf(params[0]);               // Sensor pose
  const Mat4 T_WT0 = tf(params[1]);                // Target pose
  const Eigen::Map<const Vec3> p_Tj(params[2], 3); // Target point
  const Mat4 T_T0Tj = tf(params[3]);               // Target extrinsic
  const Mat4 T_C0S = tf(params[4]);                // Imu extrinsic
  const Mat4 T_C0Ci = tf(params[5]);               // Camera extrinsic
  Eigen::Map<const VecX> intrinsic(params[6], 8);  // Camera parameters
  const double time_delay = params[7][0];          // Time delay

  // Transform and project point to image plane
  valid_ = true;
  const Mat4 T_CiC0 = tf_inv(T_C0Ci);
  const Mat4 T_SW_k = tf_inv(T_WS_k);
  const Mat4 T_CiTj = T_CiC0 * T_C0S * T_SW_k * T_WT0 * T_T0Tj;
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
  const Vec2 z_corrected = z_k_ - (time_delay * v_k_);
  Eigen::Map<Vec2> r(res);
  r = sqrt_info_ * (z_corrected - z_hat);
  residuals_ = z_corrected - z_hat;

  // Jacobians
  const MatX Jh = camera_model->project_jacobian(intrinsic, p_Ci);
  const MatX Jhw = -1.0 * sqrt_info_ * Jh;
  if (jacs == nullptr || !valid_) {
    return true;
  }

  // -- Jacobians w.r.t imu pose T_WS
  if (jacs[0]) {
    const Vec3 r_WS = tf_trans(T_WS_k);
    const Mat3 C_WS = tf_rot(T_WS_k);
    const Mat4 T_CiS = T_CiC0 * T_C0S;
    const Mat4 T_CiW = T_CiS * T_SW_k;
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
    const Mat3 C_CiW = tf_rot(T_CiS * T_SW_k);
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
    const Mat4 T_CiT0 = T_CiC0 * T_C0S * T_SW_k * T_WT0;
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

  // -- Time-delay Jacobian (mode-specific)
  if (jacs[7]) {
    Eigen::Map<Vec2> J(jacs[7]);
    J.setZero();
    J = -sqrt_info_ * v_k_;
  }

  return true;
}

/******************************************************************************
 * IMU ERROR WITH A TIME DELAY
 *****************************************************************************/

/**
 * Imu error with a time delay
 *
 * Same as `ImuError`, with a 5th parameter block: the time delay `td` [s]
 * between the imu and the poses. The time stamp of an imu measurement is the
 * physical time + `td`, so the measurements that belong to the poses `ts_i` and
 * `ts_j` are the ones in the window [ts_i + td, ts_j + td].
 *
 * `imu_buffer` has to have measurements before `ts_i` and after `ts_j`, as much
 * as the largest time delay (the ends are held when it does not). On every
 * evaluation the measurements of the shifted window are pre-integrated, so the
 * noise of the measurements in the window is the noise the covariance expects.
 * The covariance, and the jacobians with respect to the biases, are the ones of
 * the window without the shift.
 *
 * Parameter blocks: pose i, speed and biases i, pose j, speed and biases j, time
 * delay. Residuals: position, velocity, rotation, accelerometer bias and
 * gyroscope bias.
 */
struct ImuErrorTimeDelay : ResidualBlock {
  // Imu parameters and data
  const ImuParams imu_params;
  const ImuBuffer imu_buffer;
  const timestamp_t ts_i_;
  const timestamp_t ts_j_;

  // Pre-integration of the window without the shift, at the biases of state i
  Quat dq_{1.0, 0.0, 0.0, 0.0}; // Relative rotation
  Vec3 dr_{0.0, 0.0, 0.0};      // Relative position
  Vec3 dv_{0.0, 0.0, 0.0};      // Relative velocity
  Vec3 ba_{0.0, 0.0, 0.0};      // Accelerometer bias
  Vec3 bg_{0.0, 0.0, 0.0};      // Gyroscope bias
  double Dt_ = 0.0;             // Duration of the window [s]
  MatX state_F_ = eye(15);      // State jacobian (r, theta, v, ba, bg)
  MatX state_P_ = zeros(15, 15); // State covariance (r, theta, v, ba, bg)
  MatX sqrt_info_ = eye(15, 15); // Square root information (r, v, theta, ba, bg)

  ImuErrorTimeDelay(const std::vector<double *> &param_ptrs,
                    const std::vector<ParamBlock::Type> &param_types,
                    const ImuParams &imu_params,
                    const ImuBuffer &imu_buffer,
                    const timestamp_t ts_i,
                    const timestamp_t ts_j);

  /** Create residual block */
  static std::shared_ptr<ImuErrorTimeDelay> create(const ImuParams &imu_params,
                                                   const ImuBuffer &imu_buffer,
                                                   const timestamp_t ts_i,
                                                   const timestamp_t ts_j,
                                                   double *pose_i,
                                                   double *sb_i,
                                                   double *pose_j,
                                                   double *sb_j,
                                                   double *time_delay);

  /** Manually set square root information */
  void set_sqrt_info(const MatX &sqrt_info);

  /**
   * Pre-integrate the window shifted by the time delay `td` [s], at the biases
   * the error was linearized at: the relative rotation, position and velocity,
   * and the angular velocity and acceleration (biases removed) at the start
   * and the end of the window.
   */
  void shifted_preintegration(const double td,
                              Quat &dq,
                              Vec3 &dr,
                              Vec3 &dv,
                              Vec3 &w_start,
                              Vec3 &a_start,
                              Vec3 &w_end,
                              Vec3 &a_end) const;

  /** Evaluate with minimial Jacobians */
  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;

private:
  /** Pre-integrate the window without the shift */
  void propagate();
};

namespace {

/** Imu measurements at times [s] relative to a time stamp */
struct Window {
  std::vector<double> t;
  std::vector<Vec3> acc;
  std::vector<Vec3> gyr;
};

/**
 * Imu measurements between `t_start` and `t_end` [s after `ts_ref`]: the ones
 * of the buffer in between, and the interpolated ones at the two ends. The
 * measurements are held constant beyond the first and last of the buffer.
 *
 * Times are relative and in double precision, so the window can be moved by
 * less than a nanosecond.
 */
Window make_window(const ImuBuffer &buffer,
                   const timestamp_t ts_ref,
                   const double t_start,
                   const double t_end) {
  const int n = buffer.size();
  assert(n > 0);
  assert(t_end > t_start);

  std::vector<double> times(n);
  for (int k = 0; k < n; ++k) {
    times[k] = ts2sec(buffer.getTimestamp(k) - ts_ref);
  }

  auto value_at = [&](const double t, Vec3 &acc, Vec3 &gyr) {
    if (t <= times.front()) {
      acc = buffer.getAcc(0);
      gyr = buffer.getGyr(0);
      return;
    }
    if (t >= times.back()) {
      acc = buffer.getAcc(n - 1);
      gyr = buffer.getGyr(n - 1);
      return;
    }
    int k = 0;
    while (times[k + 1] <= t) {
      ++k;
    }
    const double alpha = (t - times[k]) / (times[k + 1] - times[k]);
    acc = (1.0 - alpha) * buffer.getAcc(k) + alpha * buffer.getAcc(k + 1);
    gyr = (1.0 - alpha) * buffer.getGyr(k) + alpha * buffer.getGyr(k + 1);
  };

  Window w;
  Vec3 acc;
  Vec3 gyr;
  value_at(t_start, acc, gyr);
  w.t.push_back(t_start);
  w.acc.push_back(acc);
  w.gyr.push_back(gyr);
  for (int k = 0; k < n; ++k) {
    if (times[k] > t_start && times[k] < t_end) {
      w.t.push_back(times[k]);
      w.acc.push_back(buffer.getAcc(k));
      w.gyr.push_back(buffer.getGyr(k));
    }
  }
  value_at(t_end, acc, gyr);
  w.t.push_back(t_end);
  w.acc.push_back(acc);
  w.gyr.push_back(gyr);
  return w;
}

/**
 * One mid-point step of `dt` [s] of the relative rotation, position and
 * velocity. The same scheme as `ImuPreintegrate`, which the covariance and the
 * jacobians with respect to the biases come from.
 */
void step(const Quat &dq_i,
          const Vec3 &dr_i,
          const Vec3 &dv_i,
          const Vec3 &ba,
          const Vec3 &bg,
          const Vec3 &w_k,
          const Vec3 &w_kp1,
          const Vec3 &a_k,
          const Vec3 &a_kp1,
          const double dt,
          Quat &dq_j,
          Vec3 &dr_j,
          Vec3 &dv_j) {
  const Vec3 w = 0.5 * (w_k + w_kp1) - bg;
  dq_j = dq_i * quat_delta(w * dt);

  const Vec3 acc_i = dq_i * (a_k - ba);
  const Vec3 acc_j = dq_j * (a_kp1 - ba);
  const Vec3 a = 0.5 * (acc_i + acc_j);
  dr_j = dr_i + dv_i * dt + 0.5 * a * dt * dt;
  dv_j = dv_i + a * dt;
}

} // namespace

ImuErrorTimeDelay::ImuErrorTimeDelay(
    const std::vector<double *> &param_ptrs_,
    const std::vector<ParamBlock::Type> &param_types_,
    const ImuParams &imu_params_,
    const ImuBuffer &imu_buffer_,
    const timestamp_t ts_i,
    const timestamp_t ts_j)
    : ResidualBlock{"ImuErrorTimeDelay", param_ptrs_, param_types_, 15},
      imu_params{imu_params_}, imu_buffer{imu_buffer_}, ts_i_{ts_i},
      ts_j_{ts_j} {
  assert(param_ptrs_.size() == 5);
  assert(imu_params.noise_acc > 0);
  assert(imu_params.noise_gyr > 0);
  assert(imu_params.noise_ba > 0);
  assert(imu_params.noise_bg > 0);
  assert(imu_params.g.norm() > 0);
  propagate();
}

std::shared_ptr<ImuErrorTimeDelay>
ImuErrorTimeDelay::create(const ImuParams &imu_params,
                          const ImuBuffer &imu_buffer,
                          const timestamp_t ts_i,
                          const timestamp_t ts_j,
                          double *pose_i,
                          double *sb_i,
                          double *pose_j,
                          double *sb_j,
                          double *time_delay) {
  std::vector<double *> param_ptrs = {pose_i, sb_i, pose_j, sb_j, time_delay};
  std::vector<ParamBlock::Type> param_types = {ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES,
                                               ParamBlock::POSE,
                                               ParamBlock::SPEED_BIASES,
                                               ParamBlock::TIME_DELAY};
  return std::make_shared<ImuErrorTimeDelay>(
      param_ptrs, param_types, imu_params, imu_buffer, ts_i, ts_j);
}

void ImuErrorTimeDelay::set_sqrt_info(const MatX &sqrt_info) {
  sqrt_info_ = sqrt_info;
}

void ImuErrorTimeDelay::propagate() {
  // Pre-integrate around the biases of the first state, with the measurements
  // of the window of the poses
  Eigen::Map<const VecX> sb_i(param_ptrs_[1], 9);
  const Vec3 ba_i{sb_i[3], sb_i[4], sb_i[5]};
  const Vec3 bg_i{sb_i[6], sb_i[7], sb_i[8]};

  const Window w = make_window(imu_buffer, ts_i_, 0.0, ts2sec(ts_j_ - ts_i_));
  ImuBuffer window_buffer;
  for (size_t k = 0; k < w.t.size(); ++k) {
    window_buffer.add(ts_i_ + sec2ts(w.t[k]), w.acc[k], w.gyr[k]);
  }
  const ImuPreintegrate pint{imu_params, window_buffer, ba_i, bg_i};

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

void ImuErrorTimeDelay::shifted_preintegration(const double td,
                                               Quat &dq,
                                               Vec3 &dr,
                                               Vec3 &dv,
                                               Vec3 &w_start,
                                               Vec3 &a_start,
                                               Vec3 &w_end,
                                               Vec3 &a_end) const {
  const Window w = make_window(imu_buffer, ts_i_, td, td + Dt_);

  dq = Quat{1.0, 0.0, 0.0, 0.0};
  dr = Vec3::Zero();
  dv = Vec3::Zero();
  for (size_t k = 0; k + 1 < w.t.size(); ++k) {
    Quat dq_j;
    Vec3 dr_j;
    Vec3 dv_j;
    step(dq,
         dr,
         dv,
         ba_,
         bg_,
         w.gyr[k],
         w.gyr[k + 1],
         w.acc[k],
         w.acc[k + 1],
         w.t[k + 1] - w.t[k],
         dq_j,
         dr_j,
         dv_j);
    dq = dq_j;
    dr = dr_j;
    dv = dv_j;
  }

  w_start = w.gyr.front() - bg_;
  a_start = w.acc.front() - ba_;
  w_end = w.gyr.back() - bg_;
  a_end = w.acc.back() - ba_;
}

bool ImuErrorTimeDelay::eval(double const *const *params,
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

  // Relative rotation, position and velocity of the window, shifted by the time
  // delay
  const double td = params[4][0];
  Quat dq_u;
  Vec3 dr_u;
  Vec3 dv_u;
  Vec3 w_start;
  Vec3 a_start;
  Vec3 w_end;
  Vec3 a_end;
  shifted_preintegration(td, dq_u, dr_u, dv_u, w_start, a_start, w_end, a_end);

  // Correct for the biases to first order, with the jacobians of the
  // pre-integration
  const Mat3 dr_dba = state_F_.block<3, 3>(0, 9);
  const Mat3 dr_dbg = state_F_.block<3, 3>(0, 12);
  const Mat3 dq_dbg = state_F_.block<3, 3>(3, 12);
  const Mat3 dv_dba = state_F_.block<3, 3>(6, 9);
  const Mat3 dv_dbg = state_F_.block<3, 3>(6, 12);
  const Vec3 dba = ba_i - ba_;
  const Vec3 dbg = bg_i - bg_;
  const Vec3 dhtheta = 0.5 * (dq_dbg * dbg);
  const Quat dq_theta{1.0, dhtheta.x(), dhtheta.y(), dhtheta.z()};
  const Quat dq = dq_u * dq_theta;
  const Vec3 dr = dr_u + dr_dba * dba + dr_dbg * dbg;
  const Vec3 dv = dv_u + dv_dba * dba + dv_dbg * dbg;

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

  // Jacobian w.r.t. time delay
  //
  // Moving the window by h moves its start and end, which removes the first
  // measurement and adds the last one, and rotates the frame of the relative
  // motion by w_start h, so
  //
  //   dq(td + h) = Exp(-w_start h) dq(td) Exp(w_end h)
  //   dv(td + h) = Exp(-w_start h) (dv(td) + h (dC a_end - a_start))
  //   dr(td + h) = Exp(-w_start h) (dr(td) + h (dv(td) - a_start Dt))
  if (jacs[4]) {
    const Mat3 dC = dq_u.toRotationMatrix();
    const Vec3 dr_td = dv_u - a_start * Dt_ - w_start.cross(dr_u);
    const Vec3 dv_td = dC * a_end - a_start - w_start.cross(dv_u);

    // dq' = dq Exp(eps), error = 2 vec(dq'^-1 dq_est) = 2 vec(Exp(-eps) q_e)
    const Vec3 eps = -dq.toRotationMatrix().transpose() * w_start +
                     dq_theta.toRotationMatrix().transpose() * w_end;
    const Quat q_e = dq.inverse() * (q_i.inverse() * q_j);
    const Mat3 derr_deps = -quat_right(q_e).block<3, 3>(1, 1);

    Eigen::Map<Mat<15, 1>> J(jacs[4]);
    J.setZero();
    J.segment<3>(0) = -dr_td;          // dr w.r.t time delay
    J.segment<3>(3) = -dv_td;          // dv w.r.t time delay
    J.segment<3>(6) = derr_deps * eps; // dtheta w.r.t time delay
    J = sqrt_info_ * J;
  }

  return true;
}

/******************************************************************************
 * CALIBRATION
 *****************************************************************************/

/**
 * `CalibCameraImu` with the time delay methods of the benchmark:
 *
 * - POSE_INTERP: `CalibCameraImu` as it is
 * - PIXEL_VELOCITY: the camera errors are `CalibCameraImuErrorPixelVelocity`
 * - IMU: the camera errors do not use the time delay, and the imu error of the
 *   window between two views is an `ImuErrorTimeDelay`. It needs imu
 *   measurements after the window, so it is added once they have arrived: feed
 *   the imu measurements with `add_imu_measurement()`, and the rest is added by
 *   `solve()`.
 */
struct ExpCalibCameraImu : CalibCameraImu {
  enum class Method { POSE_INTERP, PIXEL_VELOCITY, IMU };
  Method method = Method::POSE_INTERP;

  /**
   * With `IMU`: imu measurements [s] kept before and after the window between
   * two views, the largest time delay that can be estimated
   */
  double imu_margin = 0.05;

  /** Window between two views, waiting for the imu measurements after it */
  struct Pending {
    timestamp_t ts_km1;
    timestamp_t ts_k;
    double *pose_km1;
    double *sb_km1;
    double *pose_k;
    double *sb_k;
  };
  std::deque<Pending> pending;
  ImuBuffer imu_history; // Measurements the windows are made of

  /** Set the time delay method */
  void set_method(const Method m) {
    method = m;
    estimate_time_delay = (m == Method::POSE_INTERP);
  }

  /** Add imu measurement */
  void add_imu_measurement(const timestamp_t ts,
                           const Vec3 &acc,
                           const Vec3 &gyr) {
    if (method == Method::IMU) {
      imu_history.add(ts, acc, gyr);
    }
    add_measurement(ts, acc, gyr);
    if (method == Method::IMU) {
      add_pending(false);
    }
  }

  /** Make the camera error of a point */
  ResidualBlockPtr
  create_camera_error(const timestamp_t ts_km1,
                      const timestamp_t ts_k,
                      const std::shared_ptr<CameraGeometry> &camera,
                      const std::shared_ptr<ImuGeometry> &imu,
                      const std::shared_ptr<CalibTargetGeometry> &target,
                      double *pose_km1,
                      double *pose_k,
                      double *target_pose,
                      double *time_delay,
                      const int point_id,
                      const Vec2 &z_km1,
                      const Vec2 &z_k) override {
    if (method != Method::PIXEL_VELOCITY) {
      return CalibCameraImu::create_camera_error(ts_km1,
                                                 ts_k,
                                                 camera,
                                                 imu,
                                                 target,
                                                 pose_km1,
                                                 pose_k,
                                                 target_pose,
                                                 time_delay,
                                                 point_id,
                                                 z_km1,
                                                 z_k);
    }
    return CalibCameraImuErrorPixelVelocity::create(ts_km1,
                                                    ts_k,
                                                    camera,
                                                    imu,
                                                    target,
                                                    pose_k,
                                                    target_pose,
                                                    time_delay,
                                                    point_id,
                                                    z_km1,
                                                    z_k);
  }

  /** Add the imu error between two views */
  void add_imu_error(const timestamp_t ts_km1,
                     const timestamp_t ts_k) override {
    if (method != Method::IMU) {
      CalibCameraImu::add_imu_error(ts_km1, ts_k);
      return;
    }

    // Needs the measurements after ts_k, which are not there yet
    pending.push_back({ts_km1,
                       ts_k,
                       get_pose_ptr(ts_km1),
                       get_speed_and_biases_ptr(ts_km1),
                       get_pose_ptr(ts_k),
                       get_speed_and_biases_ptr(ts_k)});
  }

  /**
   * Add the imu errors of the windows that have the imu measurements after
   * them, or all of them with `all`, with the measurements there are
   */
  void add_pending(const bool all) {
    const int imu_id = 0;
    const auto &imu_params = get_imu_geometry(imu_id)->imu_params;
    const timestamp_t margin = sec2ts(imu_margin);

    while (pending.empty() == false) {
      const auto p = pending.front();

      // Wait for the imu measurements after the window
      if (all == false && imu_history.timestamps.back() < p.ts_k + margin) {
        break;
      }

      // Measurements around the window, as far as there are
      const timestamp_t ts_start =
          std::max(p.ts_km1 - margin, imu_history.timestamps.front());
      const timestamp_t ts_end =
          std::min(p.ts_k + margin, imu_history.timestamps.back());
      const auto imu_data = imu_history.extract(ts_start, ts_end);

      auto resblock = ImuErrorTimeDelay::create(imu_params,
                                                imu_data,
                                                p.ts_km1,
                                                p.ts_k,
                                                p.pose_km1,
                                                p.sb_km1,
                                                p.pose_k,
                                                p.sb_k,
                                                get_time_delay_ptr());
      imu_errors.push_back(resblock);
      add_residual_block(resblock.get());

      // Keep the measurements the next window needs
      imu_history.trim(p.ts_k - margin);
      pending.pop_front();
    }
  }

  /** Solve */
  void solve() {
    if (method == Method::IMU) {
      add_pending(true);
    }
    CalibCameraImu::solve();
  }

private:
  std::vector<std::shared_ptr<ImuErrorTimeDelay>> imu_errors;
};

static ExpCalibCameraImu::Method parse_method(const std::string &name) {
  if (name == "pixel_velocity") {
    return ExpCalibCameraImu::Method::PIXEL_VELOCITY;
  } else if (name == "pose_interp") {
    return ExpCalibCameraImu::Method::POSE_INTERP;
  } else if (name == "imu") {
    return ExpCalibCameraImu::Method::IMU;
  }
  FATAL("Unknown td_method [%s]!", name.c_str());
}

/******************************************************************************
 * RUN
 *****************************************************************************/

struct Result {
  std::string method;
  std::string noise;
  double true_delay_ms;
  int seed;
  double delay_ms;       // Estimated time delay
  double ext_pos_mm;     // Error of the camera-IMU extrinsic: position
  double ext_rot_deg;    // Error of the camera-IMU extrinsic: rotation
  int iterations;
  double cost;
  double solve_s;
};

/** Simulate and calibrate one dataset */
static Result run(const Settings &s,
                  const std::string &method,
                  const NoiseLevel &noise,
                  const double true_delay_ms,
                  const int seed) {
  // Simulated data. The IMU time stamps are the physical time + time delay.
  SimCalib sim;
  sim.sim_camimu_calib(s.camera_rate,
                       s.trajectory,
                       s.radius,
                       s.duration,
                       true_delay_ms * 1e-3);
  const double imu_rate = [&]() {
    const auto first = sim.imu_acc.begin();
    return 1.0 / ts2sec(std::next(first)->first - first->first);
  }();

  // The IMU parameters of the calibration are the noise of the data, or the
  // parameters of the simulation if the data has none
  ImuParams imu_params = sim.imus.at(0).imu_params;
  const double imu_scale = (noise.imu > 0.0) ? noise.imu : 1.0;
  imu_params.noise_acc *= imu_scale;
  imu_params.noise_gyr *= imu_scale;
  imu_params.noise_ba *= imu_scale;
  imu_params.noise_bg *= imu_scale;

  ExpCalibCameraImu calib;
  calib.verbose = false;
  calib.max_iters = s.max_iters;
  calib.set_method(parse_method(method));
  calib.add_target(sim.target_configs[0], sim.targets.at(0).extrinsic);
  for (auto &[camera_id, camera] : sim.cameras) {
    calib.add_camera(camera.camera_id,
                     camera.camera_model->type(),
                     camera.resolution,
                     camera.intrinsic,
                     camera.extrinsic);
  }
  calib.add_imu(0, imu_params, sim.imus.at(0).extrinsic);

  // Measurements, with noise
  std::mt19937 gen{s.seed + static_cast<unsigned int>(seed)};
  std::normal_distribution<double> normal{0.0, 1.0};
  const auto timeline = sim.get_timeline();
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        auto target = target_event->calib_target;
        if (noise.pixel > 0.0) {
          const auto src = std::dynamic_pointer_cast<AprilGrid>(target);
          std::vector<int> tag_ids;
          std::vector<int> corner_ids;
          Vec2s keypoints;
          Vec3s object_points;
          src->get_measurements(tag_ids, corner_ids, keypoints, object_points);
          auto noisy = std::make_shared<AprilGrid>(ts,
                                                   target_event->camera_id,
                                                   sim.target_configs[0]);
          for (size_t i = 0; i < tag_ids.size(); ++i) {
            const Vec2 n{normal(gen), normal(gen)};
            noisy->add(tag_ids[i], corner_ids[i], keypoints[i] + noise.pixel * n);
          }
          target = noisy;
        }
        calib.add_measurement(ts, target_event->camera_id, target);
      }

      if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
        // Noise density to the noise of a sample
        const double acc_std = imu_params.noise_acc * std::sqrt(imu_rate);
        const double gyr_std = imu_params.noise_gyr * std::sqrt(imu_rate);
        Vec3 acc = imu_event->acc;
        Vec3 gyr = imu_event->gyr;
        if (noise.imu > 0.0) {
          acc += acc_std * Vec3{normal(gen), normal(gen), normal(gen)};
          gyr += gyr_std * Vec3{normal(gen), normal(gen), normal(gen)};
        }
        calib.add_imu_measurement(ts, acc, gyr);
      }
    }
  }

  // Initial IMU extrinsic: the ground truth, moved and rotated
  const Vec7 gt_extrinsic = sim.imus.at(0).extrinsic;
  const Mat4 T_gt = tf(gt_extrinsic);
  const Vec3 rpy = deg2rad(s.init_rotation_deg);
  const Mat4 T_init = tf(tf_rot(T_gt) * euler321(rpy),
                         tf_trans(T_gt) + s.init_position);
  const Vec7 init = tf_vec(T_init);
  double *extrinsic = calib.imu_geometries.at(0)->extrinsic.data();
  for (int i = 0; i < 7; ++i) {
    extrinsic[i] = init[i];
  }

  // Calibrate
  const auto t0 = std::chrono::steady_clock::now();
  calib.solve();
  const double solve_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();

  // Results
  const Mat4 T_est = tf(calib.imu_geometries.at(0)->extrinsic);
  const Mat3 dC = tf_rot(T_gt).transpose() * tf_rot(T_est);
  const double cos_angle = std::min(1.0, std::max(-1.0, 0.5 * (dC.trace() - 1.0)));

  Result r;
  r.method = method;
  r.noise = noise.name;
  r.true_delay_ms = true_delay_ms;
  r.seed = seed;
  r.delay_ms = calib.time_delay_ * 1e3;
  r.ext_pos_mm = (tf_trans(T_est) - tf_trans(T_gt)).norm() * 1e3;
  r.ext_rot_deg = rad2deg(std::acos(cos_angle));
  r.iterations = calib.num_iters;
  r.cost = calib.final_cost;
  r.solve_s = solve_s;
  return r;
}

static void save_results(const fs::path &path,
                         const std::vector<Result> &results) {
  std::ofstream f{path};
  f << "method,noise,true_delay_ms,seed,delay_ms,error_ms,ext_pos_mm,"
       "ext_rot_deg,iterations,cost,solve_s\n";
  for (const auto &r : results) {
    f << r.method << "," << r.noise << "," << r.true_delay_ms << "," << r.seed
      << "," << r.delay_ms << "," << r.delay_ms - r.true_delay_ms << ","
      << r.ext_pos_mm << "," << r.ext_rot_deg << "," << r.iterations << ","
      << r.cost << "," << r.solve_s << "\n";
  }
}

/******************************************************************************
 * MAIN
 *****************************************************************************/

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("Usage: %s <config.yaml>\n", argv[0]);
    return -1;
  }
  const Settings s{argv[1]};
  fs::create_directories(s.output_dir);

  std::vector<Result> results;
  for (const auto &noise : s.noise_levels) {
    // Without noise every seed is the same data
    const int num_seeds = noise.clean() ? 1 : s.num_seeds;
    for (const double true_delay_ms : s.true_delays_ms) {
      for (const auto &method : s.td_methods) {
        for (int seed = 0; seed < num_seeds; ++seed) {
          const auto r = run(s, method, noise, true_delay_ms, seed);
          results.push_back(r);
          printf("%-10s %-15s true %+6.1f ms  seed %d  estimate %+7.2f ms  "
                 "error %+7.2f ms  (%d iterations, %.1f s)\n",
                 noise.name.c_str(),
                 method.c_str(),
                 true_delay_ms,
                 seed,
                 r.delay_ms,
                 r.delay_ms - true_delay_ms,
                 r.iterations,
                 r.solve_s);
          fflush(stdout);
          save_results(s.output_dir / "results.csv", results);
        }
      }
    }
  }

  // Summary
  printf("\n%-10s %-15s %10s %10s %10s %8s\n",
         "noise",
         "method",
         "rms [ms]",
         "mean [ms]",
         "max [ms]",
         "failed");
  for (const auto &noise : s.noise_levels) {
    for (const auto &method : s.td_methods) {
      double sum_sq = 0.0;
      double sum = 0.0;
      double max_abs = 0.0;
      int n = 0;
      int failed = 0;
      for (const auto &r : results) {
        if (r.noise != noise.name || r.method != method) {
          continue;
        }
        const double e = r.delay_ms - r.true_delay_ms;
        sum_sq += e * e;
        sum += e;
        max_abs = std::max(max_abs, std::abs(e));
        failed += (std::abs(e) > s.tolerance_ms);
        ++n;
      }
      printf("%-10s %-15s %10.2f %+10.2f %10.2f %5d/%d\n",
             noise.name.c_str(),
             method.c_str(),
             std::sqrt(sum_sq / n),
             sum / n,
             max_abs,
             failed,
             n);
    }
  }
  return 0;
}
