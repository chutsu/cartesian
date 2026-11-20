#pragma once
#include "../core/Core.hpp"

namespace cartesian {

struct ImuParams {
  int imu_id = 0;
  double noise_acc; // Accelerometer noise density [m / s^2 / sqrt(Hz)]
  double noise_gyr; // Gyroscope noise density [rad / s / sqrt(Hz)]
  double noise_ba;  // Accelerometer bias random walk [m / s^3 / sqrt(Hz)]
  double noise_bg;  // Gyroscope bias random walk [rad / s^2 / sqrt(Hz)]
  Vec3 g{0.0, 0.0, 9.81};
};

} // namespace cartesian
