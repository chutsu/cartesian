#include "SimCircle.hpp"

namespace cartesian {

double SimCircle::get_duration() const { return time_taken; }

Vec3 SimCircle::get_position(const double time_s) const {
  const double theta = theta_init + w * time_s;
  const double rx = circle_r * cos(theta);
  const double ry = circle_r * sin(theta);
  const double rz = 0.0;
  const Vec3 r_WS{rx, ry, rz};
  return r_WS;
}

Mat3 SimCircle::get_rotation(const double time_s) const {
  const double yaw = yaw_init + w * time_s;
  const Mat3 C_WS = euler321(Vec3{0.0, 0.0, yaw});
  return C_WS;
}

Mat4 SimCircle::get_pose(const double time_s) const {
  const Vec3 r_WS = get_position(time_s);
  const Mat3 C_WS = get_rotation(time_s);
  const Mat4 T_WS = tf(C_WS, r_WS);
  return T_WS;
}

Vec3 SimCircle::get_velocity(const double time_s) const {
  const double theta = theta_init + w * time_s;
  const double vx = -circle_r * w * sin(theta);
  const double vy = circle_r * w * cos(theta);
  const double vz = 0.0;
  const Vec3 v_WS{vx, vy, vz};
  return v_WS;
}

Vec3 SimCircle::get_acceleration(const double time_s) const {
  const double theta = theta_init + w * time_s;
  const double ax = -circle_r * w * w * cos(theta);
  const double ay = -circle_r * w * w * sin(theta);
  const double az = 0.0;
  const Vec3 a_WS{ax, ay, az};
  return a_WS;
}

Vec3 SimCircle::get_angular_velocity(const double time_s) const {
  UNUSED(time_s);
  const double wx = 0.0;
  const double wy = 0.0;
  const double wz = w;
  const Vec3 w_WS{wx, wy, wz};
  return w_WS;
}

Vec3 SimCircle::get_body_acceleration(const double time_s,
                                      const Vec3 &g) const {
  const Mat4 &T_WS = get_pose(time_s);
  const Mat3 &C_WS = tf_rot(T_WS);
  const Vec3 &a_WS = get_acceleration(time_s);
  const Vec3 a_S_WS = C_WS.transpose() * (a_WS + g);
  return a_S_WS;
}

Vec3 SimCircle::get_body_angular_velocity(const double time_s) const {
  const Mat4 &T_WS = get_pose(time_s);
  const Mat3 &C_WS = tf_rot(T_WS);
  const Vec3 &w_WS = get_angular_velocity(time_s);
  const Vec3 &w_S_WS = C_WS.transpose() * w_WS;
  return w_S_WS;
}

} // namespace cartesian
