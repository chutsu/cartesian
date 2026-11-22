#pragma once
#include <deque>
#include <map>
#include <set>

#include "../ceres/PoseManifold.hpp"
#include "../ceres/TiltManifold.hpp"
#include "../ceres/ReprojError.hpp"
#include "../ceres/ImuError.hpp"
#include "../imu/ImuParams.hpp"
#include "../imu/ImuBuffer.hpp"
#include "../imu/ImuGeometry.hpp"

#include "camera/CameraGeometry.hpp"
#include "SfmTrack.hpp"
#include "SfmTracker.hpp"

namespace cartesian {

/**
 * Visual-inertial structure from motion
 *
 * The body frame is the imu frame, the poses are T_WS. The camera extrinsic is
 * the pose of the camera in the imu frame T_SCi. The imu extrinsic is not used.
 *
 * Imu measurements and camera frames are added in time order. A camera frame
 * is processed once imu data at or after its timestamp has been added, since
 * the imu measurements between two frames are preintegrated. The first pose
 * is the origin, and the sensor is assumed to start at rest: the attitude comes
 * from the gravity direction and the gyroscope bias from the mean gyroscope
 * measurement of the first samples. Every new pose
 * is predicted with the imu, features are tracked in the images and
 * triangulated once they have parallax. The predictions drift quickly, so
 * the poses, speeds, biases and points are optimized every few frames, with
 * the reprojection and imu errors, and the next pose is predicted from the
 * optimized state. `solve()` optimizes everything until convergence.
 */
struct Sfm {
private:
  /*****************************************************************************
   * PRIVATE MEMBERS
   ****************************************************************************/

  struct PendingFrame {
    timestamp_t ts;
    int camera_id;
    cv::Mat image;
  };

  std::deque<PendingFrame> pending_frames_;
  std::map<int, SfmTracker> trackers_;

  /*****************************************************************************
   * FRAME PROCESSING METHODS
   ****************************************************************************/

  /** Add the pose and factors of a frame, and track its features */
  void process_frame(const PendingFrame &frame);

  /** Process the queued frames that have imu data at or after their time */
  void process_pending_frames();

  /** Add the pose, speed and biases at `ts`, predicted with the imu */
  bool add_node(const timestamp_t ts);

  /*****************************************************************************
   * VISUAL FACTOR METHODS
   ****************************************************************************/

  /** Add the reprojection factors of the features tracked at `ts` */
  void add_visual_factors(const int camera_id, const timestamp_t ts);

  /** Guess the point of a track, at `init_depth` along its first ray */
  bool initial_guess(const SfmTrack &track, Vec3 &p_W) const;

  /** Add the reprojection factor of the track measurement at `ts` */
  void add_reproj_factor(const SfmTrack &track, const timestamp_t ts);

  /*****************************************************************************
   * OPTIMIZATION METHODS
   ****************************************************************************/

  /** Optimize the problem for at most `max_iters` iterations */
  bool optimize(const int max_iters, const bool verbose);

public:
  /*****************************************************************************
   * SETTINGS
   ****************************************************************************/

  int max_features = 100;           // Max number of features tracked / camera
  int min_track_length = 3;         // Min number of views to triangulate
  double min_parallax_deg = 1.0;    // Min parallax to triangulate [deg]
  double max_reproj_error = 3.0;    // Max error of a triangulated point [px]
  double init_depth = 1.0;          // Depth of points before they have parallax
  bool estimate_camera = false;     // Optimize the camera intrinsics+extrinsics
  int optimize_every = 5;           // Optimize every n poses, 0 to never
  int optimize_min_points = 10;     // Min number of points to optimize
  int optimize_max_iters = 10;      // Max iterations of the optimization

  /*****************************************************************************
   * MEMBERS
   ****************************************************************************/

  // Solver
  ceres::Problem::Options prob_options;
  std::shared_ptr<ceres::Problem> problem;
  std::unique_ptr<ceres::LossFunction> loss;
  PoseManifold pose_plus;
  TiltManifold tilt_plus;
  ResidualBlockPtrs resblocks;
  double final_cost = 0.0;
  int num_iters = 0;

  // Sensors
  std::map<int, CameraGeometryPtr> camera_geoms;
  std::map<int, ImuGeometryPtr> imu_geoms;

  // Data
  std::map<size_t, Vec3> points;
  std::map<size_t, Vec7> poses;
  std::map<timestamp_t, Vec9> speed_biases; // Speed, accel and gyro bias
  std::vector<timestamp_t> timestamps;      // Timestamps of the poses
  ImuBuffer imu_buffer;

  /*****************************************************************************
   * SETUP METHODS
   ****************************************************************************/

  Sfm();
  ~Sfm() = default;

  /** Add camera, `extrinsic` is the camera pose in the imu frame T_SCi */
  void add_camera(const int camera_id,
                  const std::string &camera_model,
                  const Vec2i &resolution,
                  const VecX &intrinsic,
                  const Vec7 &extrinsic);

  /** Add Imu */
  void add_imu(const int imu_id,
               const ImuParams &imu_params,
               const Vec7 &extrinsic);

  /*****************************************************************************
   * DATA METHODS
   ****************************************************************************/

  /**
   * Add point p_W, the point id is the number of points before it was added,
   * so the first point has id 0, the second id 1, and so on
   */
  void add_point(const Vec3 &point);

  /** Add sensor pose T_WS at time `ts` */
  void add_pose(const timestamp_t ts, const Mat4 &pose);

  /** Add speed v_WS, accelerometer and gyroscope biases at time `ts` */
  void add_speed_and_biases(const timestamp_t ts,
                            const Vec3 &v_WS,
                            const Vec3 &bias_acc = Vec3::Zero(),
                            const Vec3 &bias_gyr = Vec3::Zero());

  /** Add a camera frame, tracks features and adds pose and visual factors */
  void add_camera_frame(const timestamp_t ts,
                        const int camera_id,
                        const cv::Mat &image);

  /** Add an imu measurement, adds the imu factors of the pending frames */
  void add_imu_measurement(const timestamp_t ts,
                           const Vec3 &imu_acc,
                           const Vec3 &imu_gyr);

  /** Camera pose T_WCi at time `ts` */
  Mat4 get_camera_pose(const timestamp_t ts, const int camera_id) const;

  /*****************************************************************************
   * OPTIMIZATION METHODS
   ****************************************************************************/

  /**
   * Optimize the poses, speeds and biases, and points. The first pose is held
   * fixed, and so are the camera parameters unless `estimate_camera` is set.
   * Returns true if the solver converged.
   */
  bool solve(const bool verbose = false);
};

} // namespace cartesian
