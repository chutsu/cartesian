#pragma once
#include "../core/Core.hpp"

namespace cartesian {

struct ImuParams {
  int imu_id = 0;
  double acc_noise_density = 0.0; // Accelerometer noise density [m / s^2 / sqrt(Hz)]
  double gyr_noise_density = 0.0; // Gyroscope noise density [rad / s / sqrt(Hz)]
  double bias_walk_acc = 0.0;  // Accelerometer bias random walk [m / s^3 / sqrt(Hz)]
  double bias_walk_gyr = 0.0;  // Gyroscope bias random walk [rad / s^2 / sqrt(Hz)]
  Vec3 g{0.0, 0.0, 9.81};
};

} // namespace cartesian
