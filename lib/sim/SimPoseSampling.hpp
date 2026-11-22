#pragma once
#include <map>
#include <vector>

#include "../camera/CameraGeometry.hpp"
#include "../core/Core.hpp"
#include "SimTarget.hpp"

namespace cartesian {

/** Camera pose sampling around a calibration board */
struct SimPoseSampling {
  /** A sampled camera pose */
  struct CameraPose {
    Mat4 T_TC; // Camera pose in the target frame
    double dist = 0.0;
    double tilt = 0.0; // [deg]
    double roll = 0.0; // [deg]
  };

  unsigned int seed = 0;
  int num_poses = 0;
  double min_dist = 0.0; // Distance to board center [m]
  double max_dist = 0.0;
  double max_tilt = 0.0; // Angle between board normal and camera position [deg]

  /** Sample camera poses T_TC looking at the board center from the front */
  std::vector<CameraPose> sample_poses(const double width,
                                       const double height) const;

  /**
   * Sample camera poses T_TC with the board center placed at a random image
   * location, so that the board is observed across the entire image. Image
   * locations further than `max_theta` [deg] off the optical axis are avoided.
   */
  std::vector<CameraPose> sample_poses_spread(const double width,
                                              const double height,
                                              const CameraGeometry &camera,
                                              const double max_theta) const;

  /** Check the whole board is seen by the camera, and within `max_theta` */
  static bool board_is_visible(const std::map<int, Vec3> &points,
                               const CameraPose &pose,
                               const CameraGeometry &camera,
                               const double max_theta);

  /** Sample poses, keeping only those where the whole board is visible */
  std::vector<CameraPose>
  sample_poses_full_board(const double width,
                          const double height,
                          const std::map<int, Vec3> &points,
                          const CameraGeometry &camera,
                          const double max_theta) const;

  /** Sample poses, keeping only those where the whole `target` is visible */
  std::vector<CameraPose> sample_poses_full_board(const SimTarget &target,
                                                  const CameraGeometry &camera,
                                                  const double max_theta) const;
};

} // namespace cartesian
