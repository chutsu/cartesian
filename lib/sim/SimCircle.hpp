#pragma once
#include "SimTrajectory.hpp"

namespace cartesian {

/** Sensor moving at constant speed around a circle in the world XY plane */
struct SimCircle : SimTrajectory {
  double circle_r = 5.0;                             // Circle radius [m]
  double circle_v = 1.0;                             // Linear speed [m / s]
  double circle_dist = 2.0 * M_PI * circle_r;        // Circumference [m]
  double time_taken = circle_dist / circle_v;        // Time for one lap [s]
  double w = -2.0 * M_PI * (1.0 / time_taken);       // Angular rate [rad / s]
  double theta_init = M_PI;                          // Initial angle on circle [rad]
  double yaw_init = M_PI / 2.0;                      // Initial sensor yaw [rad]

  /** Return sensor position r_WS in the world frame */
  Vec3 get_position(const double time_s) const;

  /** Return sensor orientation C_WS */
  Mat3 get_rotation(const double time_s) const;

  /** Return sensor linear acceleration a_WS in the world frame */
  Vec3 get_acceleration(const double time_s) const;

  /** Return sensor angular velocity w_WS in the world frame */
  Vec3 get_angular_velocity(const double time_s) const;

  /** Return time taken to complete one lap [s] */
  double get_duration() const override;

  /** Return sensor pose T_WS */
  Mat4 get_pose(const double time_s) const override;

  /** Return sensor linear velocity v_WS in the world frame */
  Vec3 get_velocity(const double time_s) const override;

  /** Return accelerometer reading (specific force) in the sensor frame */
  Vec3 get_body_acceleration(const double time_s,
                             const Vec3 &g) const override;

  /** Return gyroscope reading (angular velocity) in the sensor frame */
  Vec3 get_body_angular_velocity(const double time_s) const override;
};

} // namespace cartesian
