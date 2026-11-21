#pragma once

#include "CalibProblem.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/ImuError.hpp"
#include "../ceres/CalibCameraImuError.hpp"

namespace cartesian {

/** Camera-IMU Calibrator **/
struct CalibCameraImu : CalibProblem {
  using CameraResiduals = std::map<int, ResidualBlockPtrs>;
  using ImuResiduals = std::map<int, ImuErrorPtr>;
  using CameraBuffers = std::map<int, CalibTargetMap>;
  std::map<timestamp_t, CameraResiduals> camera_resblocks;
  std::map<timestamp_t, ImuResiduals> imu_resblocks;

  bool verbose = true;
  bool imu_started = false;
  bool camera_started = false;
  bool initialized = false;

  bool estimate_time_delay = true;
  int max_iters = 100;
  int num_iters = 0;
  double final_cost = 0.0;
  ImuBuffer imu_buffer;
  CameraBuffers camera_buffers;
  CameraBuffers prev_camera_buffers;

  CalibCameraImu() = default;
  CalibCameraImu(const std::string &config_file);
  virtual ~CalibCameraImu() = default;

  /** Find target with the most observations */
  std::pair<int, int> find_optimal_target();

  /** Estimate sensor pose T_WS */
  int estimate_sensor_pose(Mat4 &T_WS);

  /** Estimate camera pose T_C0T0 */
  int estimate_camera_pose(Mat4 &T_C0T0);

  /** Add camera calibration view */
  virtual void add_view(const timestamp_t ts, const Mat4 &T_WS);

  /**
   * Make the camera error of the target point `point_id`, measured at `z_km1`
   * in the view at `ts_km1` and at `z_k` in the view at `ts_k`
   */
  virtual ResidualBlockPtr
  create_camera_error(const timestamp_t ts_km1,
                      const timestamp_t ts_k,
                      const std::shared_ptr<CameraGeometry> &camera,
                      const std::shared_ptr<ImuGeometry> &imu,
                      const std::shared_ptr<CalibTargetGeometry> &target,
                      double *pose_km1,
                      double *pose_k,
                      double *target_pose,
                      double *time_delay,
                      const int point_id,
                      const Vec2 &z_km1,
                      const Vec2 &z_k);

  /** Add the imu error between the views at `ts_km1` and `ts_k` */
  virtual void add_imu_error(const timestamp_t ts_km1, const timestamp_t ts_k);

  /** Initialize */
  void initialize(const timestamp_t ts);

  /** Add IMU measurement */
  void add_measurement(const timestamp_t ts,
                       const Vec3 &imu_acc,
                       const Vec3 &imu_gyr);

  /** Add camera measurement */
  void add_measurement(const timestamp_t ts,
                       const int camera_id,
                       const CalibTargetPtr &calib_target);

  /** Solve */
  void solve();
};

} // namespace cartesian
