#pragma once
#include "../core/Core.hpp"

namespace cartesian {

/** Interface for a simulated sensor trajectory */
struct SimTrajectory {
  SimTrajectory() = default;
  virtual ~SimTrajectory() = default;

  /** Return trajectory duration [s] */
  virtual double get_duration() const = 0;

  /** Return sensor pose T_WS at time `time_s` */
  virtual Mat4 get_pose(const double time_s) const = 0;

  /** Return sensor velocity v_WS expressed in the world frame */
  virtual Vec3 get_velocity(const double time_s) const = 0;

  /** Return accelerometer reading (specific force) in the sensor frame */
  virtual Vec3 get_body_acceleration(const double time_s,
                                     const Vec3 &g) const = 0;

  /** Return gyroscope reading (angular velocity) in the sensor frame */
  virtual Vec3 get_body_angular_velocity(const double time_s) const = 0;
};

} // namespace cartesian
