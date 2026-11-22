#include "SimImu.hpp"

namespace cartesian {

SimImu::SimImu(const SimTrajectory &trajectory,
               const double imu_rate,
               const Vec3 &g) {
  timestamp_t ts = 0;
  timestamp_t dt_ns = sec2ts(1.0 / imu_rate);
  while (ts <= sec2ts(trajectory.get_duration())) {
    const double time_s = ts2sec(ts);
    const Mat4 T_WS = trajectory.get_pose(time_s);
    const Vec3 v_WS = trajectory.get_velocity(time_s);
    const Vec3 a_S_WS = trajectory.get_body_acceleration(time_s, g);
    const Vec3 w_S_WS = trajectory.get_body_angular_velocity(time_s);

    timestamps.push_back(ts);
    poses[ts] = T_WS;
    vel[ts] = v_WS;
    imu_acc[ts] = a_S_WS;
    imu_gyr[ts] = w_S_WS;
    ts += dt_ns;
  }
}

void SimImu::Sensor::reset() {
  started = false;
  b_g = zeros(3, 1);
  b_a = zeros(3, 1);
  ts_prev = 0;
}

void SimImu::Sensor::measurement(std::default_random_engine &rndeng,
                                 const timestamp_t &ts,
                                 const Mat4 &T_WS_W,
                                 const Vec3 &w_WS_W,
                                 const Vec3 &a_WS_W,
                                 Vec3 &a_WS_S,
                                 Vec3 &w_WS_S) {
  // Delta time according to sample rate
  double dt = 1.0 / rate;

  // Check consistency of time increments:
  if (started == false) {
    if (fabs(ts2sec(ts - ts_prev) - dt) < (dt / 2.0)) {
      FATAL("Inconsisten sample rate with parameter setting: %f < %f",
            fabs(double(ts - ts_prev) * 1.0e-9 - dt),
            dt / 2.0);
    }
  }

  // IMU initialised?
  if (started == false) {
    // Stationary properties of an Ornstein-Uhlenbeck process
    b_g = mvn(rndeng) * sigma_gw_c * sqrt(tau_g / 2.0);
    b_a = mvn(rndeng) * sigma_aw_c * sqrt(tau_a / 2.0);
    started = true;

  } else {
    // Propagate biases (slow moving signal)
    const Vec3 w_g = mvn(rndeng); // Gyro white noise
    b_g += -b_g / tau_g * dt + w_g * sigma_gw_c * sqrt(dt);
    const Vec3 w_a = mvn(rndeng); // Accel white noise
    b_a += -b_a / tau_a * dt + w_a * sigma_aw_c * sqrt(dt);
  }

  // Compute gyro measurement
  const Mat3 C_SW = tf_rot(T_WS_W).transpose();
  const Vec3 w_g = mvn(rndeng); // Gyro white noise
  // w_WS_S = C_SW * w_WS_W + b_g + w_g * sigma_g_c * sqrt(dt);
  UNUSED(w_g);
  w_WS_S = C_SW * w_WS_W;

  // Compute accel measurement
  const Vec3 gravity{0.0, 0.0, g}; // Gravity vector
  const Vec3 w_a = mvn(rndeng);  // Accel white noise
  // a_WS_S = C_SW * (a_WS_W + gravity) + b_a + w_a * sigma_a_c * sqrt(dt);
  UNUSED(w_a);
  a_WS_S = C_SW * (a_WS_W + gravity);

  ts_prev = ts;
}

int SimImu::get_num_measurements() const { return timestamps.size(); }

ImuBuffer SimImu::form_imu_buffer(const int start_index,
                                  const int end_index) const {
  ImuBuffer buffer;
  for (int i = 0; i < (end_index - start_index); i++) {
    const timestamp_t ts = timestamps.at(start_index + i);
    const Vec3 acc = imu_acc.at(ts);
    const Vec3 gyr = imu_gyr.at(ts);
    buffer.add(ts, acc, gyr);
  }
  return buffer;
}

void SimImu::save(const std::string &save_path) const {
  // Open csv file
  FILE *csv_file = fopen(save_path.c_str(), "w");

  // Write header
  fprintf(csv_file, "ts,");
  fprintf(csv_file, "x,y,z,");
  fprintf(csv_file, "qx,qy,qz,qw,");
  fprintf(csv_file, "vx,vy,vz,");
  fprintf(csv_file, "ax,ay,az,");
  fprintf(csv_file, "wx,wy,wz\n");

  // Write data
  for (int k = 0; k < get_num_measurements(); k++) {
    const timestamp_t ts = timestamps.at(k);
    const Mat4 pose = poses.at(ts);
    const Vec3 r = tf_trans(pose);
    const Quat q = tf_quat(pose);
    const Vec3 v = vel.at(ts);
    const Vec3 acc = imu_acc.at(ts);
    const Vec3 gyr = imu_gyr.at(ts);

    fprintf(csv_file, "%ld,", ts);
    fprintf(csv_file, "%f,%f,%f,", r.x(), r.y(), r.z());
    fprintf(csv_file, "%f,%f,%f,%f,", q.w(), q.x(), q.y(), q.z());
    fprintf(csv_file, "%f,%f,%f,", v.x(), v.y(), v.z());
    fprintf(csv_file, "%f,%f,%f,", acc.x(), acc.y(), acc.z());
    fprintf(csv_file, "%f,%f,%f\n", gyr.x(), gyr.y(), gyr.z());
  }

  fclose(csv_file);
}

} // namespace cartesian
