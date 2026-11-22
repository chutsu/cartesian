#pragma once
#include "../imu/imu.hpp"
#include "SimTrajectory.hpp"

namespace cartesian {

struct SimImu {
  /** Simulated IMU sensor model: noise parameters and bias state */
  struct Sensor {
    // IMU parameters
    double rate = 0.0;       // IMU rate [Hz]
    double tau_a = 3600.0;   // Reversion time constant for accel [s]
    double tau_g = 3600.0;   // Reversion time constant for gyro [s]
    double sigma_g_c = 0.0;  // Gyro noise density [rad/s/sqrt(Hz)]
    double sigma_a_c = 0.0;  // Accel noise density [m/s^s/sqrt(Hz)]
    double sigma_gw_c = 0.0; // Gyro drift noise density [rad/s^s/sqrt(Hz)]
    double sigma_aw_c = 0.0; // Accel drift noise density [m/s^2/sqrt(Hz)]
    double g = 9.81;         // Gravity vector [ms-2]

    // IMU flags and biases
    bool started = false;
    Vec3 b_g = zeros(3, 1);
    Vec3 b_a = zeros(3, 1);
    timestamp_t ts_prev = 0;

    /** Reset sensor state */
    void reset();

    /** Simulate IMU measurement */
    void measurement(std::default_random_engine &rndeng,
                     const timestamp_t &ts,
                     const Mat4 &T_WS_W,
                     const Vec3 &w_WS_W,
                     const Vec3 &a_WS_W,
                     Vec3 &a_WS_S,
                     Vec3 &w_WS_S);
  };

  // Data
  std::vector<timestamp_t> timestamps;
  std::map<timestamp_t, Mat4> poses;
  std::map<timestamp_t, Vec3> vel;
  std::map<timestamp_t, Vec3> imu_acc;
  std::map<timestamp_t, Vec3> imu_gyr;

  /**
   * Simulate IMU measurements along `trajectory` at `imu_rate` [Hz], where `g`
   * is the gravity vector in the world frame.
   */
  SimImu(const SimTrajectory &trajectory,
         const double imu_rate = 200.0,
         const Vec3 &g = Vec3{0.0, 0.0, 9.81});
  virtual ~SimImu() = default;

  /** Return number of measurements */
  int get_num_measurements() const;

  /** Extract subset of data */
  ImuBuffer form_imu_buffer(const int start_index, const int end_index) const;

  /** Save simulation data */
  void save(const std::string &save_path) const;
};

} // namespace cartesian
