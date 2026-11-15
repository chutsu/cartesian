#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "sim/SimCircle.hpp"

namespace cartesian {

TEST(SimCircle, construct) {
  SimCircle sim;

  bool debug = false;
  if (debug) {
    std::vector<Vec3> pose_points;
    std::vector<Vec3> pose_colors;
    std::vector<double> pose_radii;

    std::map<timestamp_t, Mat4> poses;
    std::map<timestamp_t, double> vel_x;
    std::map<timestamp_t, double> vel_y;
    std::map<timestamp_t, double> vel_z;
    std::map<timestamp_t, double> acc_x;
    std::map<timestamp_t, double> acc_y;
    std::map<timestamp_t, double> acc_z;

    double time_s = 0.0;
    double dt = 0.01;
    while (time_s < sim.time_taken) {
      const Mat4 T_WS = sim.get_pose(time_s);
      const Vec3 v_WS = sim.get_velocity(time_s);
      const Vec3 a_WS = sim.get_acceleration(time_s);

      pose_points.push_back(tf_trans(T_WS));
      pose_colors.emplace_back(255.0, 0.0, 0.0);
      pose_radii.emplace_back(0.01);

      const timestamp_t ts = sec2ts(time_s);
      poses[ts] = T_WS;
      vel_x[ts] = v_WS.x();
      vel_y[ts] = v_WS.y();
      vel_z[ts] = v_WS.z();
      acc_x[ts] = a_WS.x();
      acc_y[ts] = a_WS.y();
      acc_z[ts] = a_WS.z();

      time_s += dt;
    }

    // Visualize
    Vec3i red{255, 0, 0};
    Vec3i green{0, 255, 0};
    Vec3i blue{0, 0, 255};

    Logger log;

    log.init_series_line("/vel/x", red, 0.5);
    log.init_series_line("/vel/y", green, 0.5);
    log.init_series_line("/vel/z", blue, 0.5);
    log.init_series_line("/acc/x", red, 0.5);
    log.init_series_line("/acc/y", green, 0.5);
    log.init_series_line("/acc/z", blue, 0.5);

    log.log_scalar("/vel/x", vel_x);
    log.log_scalar("/vel/y", vel_y);
    log.log_scalar("/vel/z", vel_z);
    log.log_scalar("/acc/x", acc_x);
    log.log_scalar("/acc/y", acc_y);
    log.log_scalar("/acc/z", acc_z);
    log.log_poses("/world/poses", poses, 0.1);
    log.log_trajectory("/world/traj", poses);
  }
}

} // namespace cartesian
