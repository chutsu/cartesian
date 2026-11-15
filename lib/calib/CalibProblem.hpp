#pragma once
#include <filesystem>

#include "../core/Core.hpp"
#include "../calib/CalibInit.hpp"
#include "../camera/CameraGeometry.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuGeometry.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/TiltManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

#include "SolvePnp.hpp"
#include "AprilGrid.hpp"
#include "AprilGridConfig.hpp"
#include "AprilGridDetector.hpp"
#include "CalibTargetGeometry.hpp"

namespace cartesian {

/** Calibration Problem */
struct CalibProblem {
  // Settings
  bool verbose = false;
  fs::path config_path;
  fs::path data_path;

  // Calibration Target
  std::map<int, AprilGridConfig> target_configs;

  // Data
  std::set<timestamp_t> timestamps;
  std::map<timestamp_t, Vec7> poses;
  std::map<timestamp_t, Vec9> speed_and_biases;
  Vec7 target_pose;
  double time_delay_ = 0.0;
  bool time_delay_added_ = false;

  std::map<int, CalibTargetData> camera_data;
  std::map<int, ImuBuffer> imu_data;
  std::map<int, CalibTargetGeometryPtr> target_geometries;
  std::map<int, CameraGeometryPtr> camera_geometries;
  std::map<int, ImuGeometryPtr> imu_geometries;

  // Ceres
  ceres::Problem::Options prob_options;
  std::shared_ptr<ceres::Problem> problem;
  PoseManifold pose_plus;
  TiltManifold tilt_plus;

  CalibProblem();
  CalibProblem(const std::string &config_path_);
  virtual ~CalibProblem() = default;

  /*****************************************************************************
   * Setup methods
   ****************************************************************************/

  /** Setup Solver */
  void setup_solver();

  /*****************************************************************************
   * Load methods
   ****************************************************************************/

  /** Load Camera Data */
  void load_camera_data(const int camera_id);

  /** Load IMU Data */
  void load_imu_data(const int imu_id);

  /*****************************************************************************
   * Camera methods
   ****************************************************************************/

  /** Add Camera */
  void add_camera(const int camera_id,
                  const std::string &camera_model,
                  const Vec2i &resolution,
                  const VecX &intrinsic,
                  const Vec7 &extrinsic);

  /** Add camera measurement */
  void add_camera_measurement(const timestamp_t ts,
                              const int camera_id,
                              const CalibTargetPtr &calib_target);

  /** Check if we have a camera measurement already */
  bool has_camera_measurement(const timestamp_t ts,
                              const int camera_id,
                              const int target_id) const;

  /** Get number of cameras */
  int get_num_cameras() const;

  /** Get all camera data */
  std::map<int, CalibTargetData> &get_all_camera_data();

  /** Get camera data */
  CalibTargetData &get_camera_data(const int camera_id);

  /** Get camera geometries */
  std::map<int, CameraGeometryPtr> &get_all_camera_geometries();

  /** Get camera geometry */
  CameraGeometryPtr &get_camera_geometry(const int camera_id);

  /** Initialize camera intrinsic */
  void initialize_camera_intrinsics();

  /** Initialize camera extrinsic */
  void initialize_camera_extrinsics();

  /*****************************************************************************
   * IMU methods
   ****************************************************************************/

  /** Add Imu */
  void add_imu(const int imu_id,
               const ImuParams &imu_params,
               const Vec7 &extrinsic);

  /** Get number of IMUs */
  int get_num_imus() const;

  /** Get all IMU data */
  std::map<int, ImuBuffer> &get_all_imu_data();

  /** Get IMU data */
  ImuBuffer &get_imu_data(const int imu_id);

  /** Get IMU geometries */
  std::map<int, ImuGeometryPtr> &get_all_imu_geometries();

  /** Get IMU geometry */
  ImuGeometryPtr &get_imu_geometry(const int imu_id);

  /*****************************************************************************
   * Calibration target methods
   ****************************************************************************/

  /** Add calibration target */
  void add_target(const AprilGridConfig &config, const Vec7 &extrinsic);

  /** Set target pose */
  void set_target_pose(const Mat4 &pose);

  /** Get target pose */
  Vec7 &get_target_pose();

  /** Get target pose pointer */
  double *get_target_pose_ptr();

  /** Get number of targets */
  int get_num_targets() const;

  /** Get calibration target geometries */
  std::map<int, CalibTargetGeometryPtr> &get_all_target_geometries();

  /** Get calibration target geometry */
  CalibTargetGeometryPtr &get_target_geometry(const int target_id);

  /** Get target point */
  Vec3 &get_target_point(const int target_id, const int point_id);

  /*****************************************************************************
   * Pose methods
   ****************************************************************************/

  /** Add pose */
  void add_pose(const timestamp_t ts, const Mat4 &pose);

  /** Get pose */
  Vec7 &get_pose(const timestamp_t ts);

  /** Get pose pointer  */
  double *get_pose_ptr(const timestamp_t ts);

  /*****************************************************************************
   * Speed and biases methods
   ****************************************************************************/

  /** Add speed and biases */
  void add_speed_and_biases(const timestamp_t ts,
                            const Vec3 &v_WS,
                            const Vec3 &bias_acc = zeros(3, 1),
                            const Vec3 &bias_gyr = zeros(3, 1));

  /** Get pose */
  Vec9 &get_speed_and_biases(const timestamp_t ts);

  /** Get pose pointer */
  double *get_speed_and_biases_ptr(const timestamp_t ts);

  /*****************************************************************************
   * Time-delay methods
   ****************************************************************************/

  /** Add time delay */
  void add_time_delay();

  /** Get time delay pointer */
  double *get_time_delay_ptr();

  /*****************************************************************************
   * Ceres
   ****************************************************************************/

  /** Add residual block */
  void add_residual_block(ResidualBlock *resblock);

  /*****************************************************************************
   * Misc methods
   ****************************************************************************/

  /** Print settings */
  void print_settings(FILE *fp) const;

  /** Print calibration target configs */
  void print_calib_target_configs(FILE *fp) const;

  /** Print target geometries */
  void print_target_geometries(FILE *fp, const bool max_digits = false) const;

  /** Print camera geometries */
  void print_camera_geometries(FILE *fp, const bool max_digits = false) const;

  /** Print IMU geometries */
  void print_imu_geometries(FILE *fp, const bool max_digits = false) const;

  /** Print time delay */
  void print_time_delay(FILE *fp) const;

  /** Print target points */
  void print_target_points(FILE *fp) const;

  /** Print summary */
  void print_summary(FILE *fp, const bool max_digits = false) const;

  /** Save results */
  void save_results(const std::string &save_path) const;
};

} // namespace cartesian
