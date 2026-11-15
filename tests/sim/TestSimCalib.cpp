#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "sim/SimCalib.hpp"

namespace cartesian {

TEST(SimCalib, construct) {
  const double camera_rate = 10.0;
  const double sample_x = 1.0;
  const double sample_y = 1.0;
  const double sample_z = 0.3;
  const double sample_z_offset = 1.0;
  const int sample_num_x = 2;
  const int sample_num_y = 2;
  const int sample_num_z = 1;

  SimCalib sim;
  sim.sim_camera_calib(camera_rate,
                       sample_x,
                       sample_y,
                       sample_z,
                       sample_z_offset,
                       sample_num_x,
                       sample_num_y,
                       sample_num_z);

  bool debug = false;
  if (debug) {
    std::vector<Vec3> pose_points;
    std::vector<Vec3> pose_colors;
    std::vector<double> pose_radii;
    for (const auto &[ts, pose] : sim.camera_poses) {
      pose_points.push_back(tf_trans(pose));
      pose_colors.emplace_back(255.0, 0.0, 0.0);
      pose_radii.emplace_back(0.01);
    }

    Logger log;
    log.log_poses("/world/camera_poses", sim.camera_poses, 0.1);
    log.log_trajectory("/world/camera_trajectory", sim.camera_poses);
    log.log_target("/world/calib_target",
                   sim.target_configs.at(0),
                   sim.target_poses.at(0));
    // log.log_points("/world/candidate_camera_poses",
    //                pose_points,
    //                pose_colors,
    //                pose_radii);
    sim.save("/tmp/sim_calib_camera");
  }
}

} // namespace cartesian
