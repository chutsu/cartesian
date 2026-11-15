#include <gtest/gtest.h>

#include "calib/LissajousTrajectory.hpp"
#include "calib/AprilGridConfig.hpp"
#include "core/Logger.hpp"

namespace cartesian {

static Mat3 Exp(const Vec3 &phi) {
  const double norm = phi.norm();

  // Small angle approx
  if (norm < 1e-3) {
    return Mat3{eye(3) + skew(phi)};
  }

  // Exponential map from so(3) to SO(3)
  const Mat3 phi_skew = skew(phi);
  Mat3 C = eye(3);
  C += (sin(norm) / norm) * phi_skew;
  C += ((1 - cos(norm)) / (norm * norm)) * (phi_skew * phi_skew);

  return C;
}

static AprilGridConfig setup_target_config() {
  const int target_id = 0;
  const int tag_rows = 8;
  const int tag_cols = 10;
  const double tag_size = 0.08;
  const double tag_spacing = 0.3;
  const int tag_id_offset = 0;
  const AprilGridConfig target_config{
      target_id,
      tag_rows,
      tag_cols,
      tag_size,
      tag_spacing,
      tag_id_offset,
  };
  return target_config;
}

static Mat4 setup_target_pose(const AprilGridConfig &config) {
  const auto target_center = config.get_center();
  const Vec3 target_pos{1.0, target_center.x(), 0.0};
  const Vec3 target_euler{M_PI / 2.0, 0.0, -M_PI / 2.0};
  const Mat3 target_rot = euler321(target_euler);
  const Mat4 T_WTj = tf(target_rot, target_pos);
  return T_WTj;
}

static Mat4 setup_target_center_pose(const AprilGridConfig &config) {
  const double calib_width = config.get_width_height().x();
  const double calib_height = config.get_width_height().y();
  const Mat3 C_TO = eye(3);
  const Vec3 r_TO{calib_width / 2.0, calib_height / 2.0, 1.0};
  const Mat4 T_TO = tf(C_TO, r_TO);
  return T_TO;
}

TEST(LissajousTrajectory, construct) {
  // Setup
  const std::string traj_type = "fig8-horiz";
  const AprilGridConfig config = setup_target_config();
  const Mat4 &T_WT = setup_target_pose(config);
  const Mat4 &T_TO = setup_target_center_pose(config);
  const timestamp_t ts_start = 0;
  const double calib_width = config.get_width_height().x();
  const double calib_height = config.get_width_height().y();
  const double R = 0.5;
  const double T = 1.0;
  LissajousTrajectory
      traj{traj_type, ts_start, T_WT, T_TO, calib_width, calib_height, R, T};

  // Test propagate body acceleration and angular velocity
  {
    // Imu rate and time variables
    const double imu_hz = 1000.0;
    const double dt = 1.0 / imu_hz;
    timestamp_t ts_k = 0;
    timestamp_t ts_end = sec2ts(T);
    double path_length = 0.0;

    // Initialize position, velocity and attidue
    const Mat4 T_WS_init = traj.get_pose(0);
    Vec3 r_WS = tf_trans(T_WS_init);
    Mat3 C_WS = tf_rot(T_WS_init);
    Vec3 v_WS = traj.get_velocity(0);

    while (ts_k <= ts_end) {
      // Get sensor acceleration and angular velocity in world frame
      const auto a_WS_W = traj.get_acceleration(ts_k);
      const auto w_WS_W = traj.get_angular_velocity(ts_k);

      // Transform acceleration and angular velocity to body frame
      const Vec3 g{0.0, 0.0, 9.81};
      const Vec3 a_WS_S = C_WS.transpose() * (a_WS_W + g);
      const Vec3 w_WS_S = C_WS.transpose() * w_WS_W;

      // Propagate simulated ideal IMU measurements
      const double dt_sq = dt * dt;
      // -- Position at time k
      const Vec3 r_WS_km1 = r_WS;
      r_WS += v_WS * dt;
      r_WS += 0.5 * -g * dt_sq;
      r_WS += 0.5 * C_WS * a_WS_S * dt_sq;
      path_length += (r_WS - r_WS_km1).norm();
      // -- velocity at time k
      v_WS += C_WS * a_WS_S * dt - g * dt;
      // -- Attitude at time k
      C_WS = C_WS * Exp(w_WS_S * dt);

      // Update
      ts_k += sec2ts(dt);
    }

    // Assert
    const Mat4 T_WS_est = tf(C_WS, r_WS);
    const Vec3 r_est = tf_trans(T_WS_est);
    const Vec3 r_gnd = tf_trans(traj.get_pose(ts_end));
    const Vec3 rpy_est = quat2euler(tf_quat(T_WS_est));
    const Vec3 rpy_gnd = quat2euler(tf_quat(traj.get_pose(ts_end)));

    ASSERT_TRUE((r_est - r_gnd).norm() < 1e-2);
    ASSERT_TRUE((rpy_est - rpy_gnd).norm() < 1e-2);
    ASSERT_TRUE(path_length > 1.0);
  }

  // Debug
  bool debug = false;
  if (debug) {
    Logger log;
    const Vec3i red{255.0, 0.0, 0.0};
    const Vec3i green{0.0, 255.0, 0.0};
    const Vec3i blue{0.0, 0.0, 255.0};

    // Plot velocity, acceleration and angular velocity
    log.init_series_line("vel/x", red, 1.0f);
    log.init_series_line("vel/y", green, 1.0f);
    log.init_series_line("vel/z", blue, 1.0f);
    log.init_series_line("acc/x", red, 1.0f);
    log.init_series_line("acc/y", green, 1.0f);
    log.init_series_line("acc/z", blue, 1.0f);
    log.init_series_line("angvel/x", red, 1.0f);
    log.init_series_line("angvel/y", green, 1.0f);
    log.init_series_line("angvel/z", blue, 1.0f);

    double time = 0;
    const double imu_hz = 1000.0;
    const double dt = 1.0 / imu_hz;
    std::map<timestamp_t, Mat4> poses;
    while (time <= T) {
      const timestamp_t ts = sec2ts(time);
      poses[ts] = traj.get_pose(ts);
      const Vec3 v_WS = traj.get_velocity(ts);
      const Vec3 a_WS = traj.get_acceleration(ts);
      const Vec3 w_WS = traj.get_angular_velocity(ts);

      log.log_scalar("vel/x", ts, v_WS.x());
      log.log_scalar("vel/y", ts, v_WS.y());
      log.log_scalar("vel/z", ts, v_WS.z());

      log.log_scalar("acc/x", ts, a_WS.x());
      log.log_scalar("acc/y", ts, a_WS.y());
      log.log_scalar("acc/z", ts, a_WS.z());

      log.log_scalar("angvel/x", ts, w_WS.x());
      log.log_scalar("angvel/y", ts, w_WS.y());
      log.log_scalar("angvel/z", ts, w_WS.z());

      time += dt;
    }

    // Log poses, trajectory and target
    log.log_poses("pose", poses, 0.1);
    log.log_trajectory("trajectory", poses);
    log.log_target("calib_target", config, T_WT);
  }
}

} // namespace cartesian
