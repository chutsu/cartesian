#pragma once
#include "../core/Core.hpp"
#include "../calib/calib.hpp"
#include "../camera/camera.hpp"
#include "../timeline/timeline.hpp"

namespace cartesian {

using AprilGridMap = std::map<int, std::shared_ptr<AprilGrid>>;
using CameraMap = std::map<int, AprilGridMap>;

struct SimCalib {
  /****************************************************************************
   * DATA
   ***************************************************************************/

  // Calibration targets and sensor geometries
  std::map<int, AprilGridConfig> target_configs;
  std::map<int, Mat4> target_poses;
  std::map<int, CalibTargetGeometry> targets;
  std::map<int, CameraGeometry> cameras;
  std::map<int, ImuGeometry> imus;

  // Simulated trajectory and measurements
  std::map<timestamp_t, Mat4> camera_poses;
  std::map<timestamp_t, Vec3> imu_vel;
  std::map<timestamp_t, Vec3> imu_acc;
  std::map<timestamp_t, Vec3> imu_gyr;
  std::map<timestamp_t, CameraMap> camera_views;

  /****************************************************************************
   * CONSTRUCTOR / DESTRUCTOR
   ***************************************************************************/

  SimCalib() = default;
  virtual ~SimCalib() = default;

  /****************************************************************************
   * SIMULATION SCENARIOS
   ***************************************************************************/

  void sim_camera_calib(const double camera_rate = 10.0,
                        const double sample_x = 1.0,
                        const double sample_y = 1.0,
                        const double sample_z = 0.3,
                        const double sample_z_offset = 1.0,
                        const int sample_num_x = 5,
                        const int sample_num_y = 7,
                        const int sample_num_z = 3);
  void sim_camimu_calib(const double camera_rate = 10.0,
                        const std::string traj_type = "fig8-horiz",
                        const double R = 0.5,
                        const double T = 10.0,
                        const double td_s = 0.0);

  /****************************************************************************
   * SETUP
   ***************************************************************************/

  void setup_calib_targets();
  void setup_camera_geometries();
  void setup_camera_poses(const double camera_rate,
                          const double sample_x,
                          const double sample_y,
                          const double sample_z,
                          const double sample_z_offset,
                          const int sample_num_x,
                          const int sample_num_y,
                          const int sample_num_z);
  void setup_camera_poses(const double camera_rate,
                          const std::string traj_type,
                          const double R,
                          const double T,
                          const double td_s = 0.0);
  void setup_imu_geometries();
  void setup_imu_poses();

  /****************************************************************************
   * SIMULATE
   ***************************************************************************/

  void simulate_camera_views();

  /****************************************************************************
   * TIMELINE
   ***************************************************************************/

  Timeline get_timeline() const;

  /****************************************************************************
   * SAVE
   ***************************************************************************/

  int save_target_configs(const fs::path &yaml_path) const;
  int save_target_poses(const fs::path &csv_path) const;
  int save_camera_geometries(const fs::path &yaml_path) const;
  int save_imu_geometries(const fs::path &yaml_path) const;
  int save_camera_poses(const fs::path &csv_path) const;
  int save_camera_views(const fs::path &save_dir) const;
  int save(const fs::path &save_dir) const;
};

} // namespace cartesian
