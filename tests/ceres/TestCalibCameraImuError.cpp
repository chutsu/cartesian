#include <gtest/gtest.h>

#include "calib/CalibProblem.hpp"
#include "ceres/CalibCameraImuError.hpp"

namespace cartesian {

static Vec7 setup_sensor_pose() {
  // Imu pose in world frame: T_WS
  // clang-format off
  Mat4 T_WS;
  T_WS <<  1.0, 0.0, 0.0, 0.001,
           0.0, 1.0, 0.0, 0.002,
           0.0, 0.0, 1.0, 0.003,
           0.0, 0.0, 0.0, 1.0;
  T_WS = tf_perturb_rot(T_WS, 0.01, 1);
  auto sensor_pose = tf_vec(T_WS);
  // ^ Note: Due to numerical stability issues the translation component
  // cannot be 0 for checking jacobians
  // clang-format on

  return sensor_pose;
}

static Vec7 setup_target_pose() {
  // Target pose: T_WT0
  // clang-format off
  Mat4 T_WT0;
  T_WT0 << 0.0,  0.0, -1.0, 0.5,
          -1.0,  0.0,  0.0, 0.01,
           0.0,  1.0,  0.0, 0.01,
           0.0,  0.0,  0.0, 1.0;
  T_WT0 = tf_perturb_rot(T_WT0, 0.01, 2);
  auto target_pose = tf_vec(T_WT0);
  // clang-format on

  return target_pose;
}

static Vec7 setup_target_extrinsic() {
  // Target extrinsics: T_T0Tj
  // clang-format off
  Mat4 T_T0Tj;
  T_T0Tj << 1.0, 0.0, 0.0, 0.01,
            0.0, 1.0, 0.0, 0.02,
            0.0, 0.0, 1.0, 0.03,
            0.0, 0.0, 0.0, 1.0;
  T_T0Tj = tf_perturb_rot(T_T0Tj, 0.01, 1);
  auto extrinsic = tf_vec(T_T0Tj);
  // clang-format on

  return extrinsic;
}

static Vec7 setup_camera_extrinsic() {
  // Camera extrinsics: T_C0Ci
  // clang-format off
  Mat4 T_C0Ci;
  T_C0Ci << 1.0, 0.0, 0.0, -0.01,
           0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 1.0;
  T_C0Ci = tf_perturb_rot(T_C0Ci, 0.01, 2);
  auto camera_extrinsic = tf_vec(T_C0Ci);
  // clang-format on

  return camera_extrinsic;
}

static Vec7 setup_imu_extrinsic() {
  // Imu extrinsics: T_C0S
  // clang-format off
  Mat4 T_C0S;
  T_C0S <<  0.0, -1.0,  0.0, 0.01,
            0.0,  0.0, -1.0, 0.01,
            1.0,  0.0,  0.0, 0.01,
            0.0,  0.0,  0.0, 1.0;
  T_C0S = tf_perturb_rot(T_C0S, -0.01, 2);
  auto imu_extrinsic = tf_vec(T_C0S);
  // clang-format on

  return imu_extrinsic;
}

static std::shared_ptr<ImuGeometry> setup_imu_geometry(const int imu_id,
                                                       const Vec7 &extrinsic) {
  ImuParams imu_params;
  imu_params.noise_acc = 0.08;
  imu_params.noise_gyr = 0.004;
  imu_params.noise_ba = 0.00004;
  imu_params.noise_bg = 2.0e-6;

  return std::make_shared<ImuGeometry>(imu_id, imu_params, extrinsic);
}

static std::shared_ptr<CameraGeometry>
setup_camera_geometry(const int camera_index, const Vec7 &extrinsic) {
  const Vec2i resolution{752, 480};
  const std::string camera_model = "BrownConrady4";

  // Intrinsic
  const double fx = 458.654;
  const double fy = 457.296;
  const double cx = 367.215;
  const double cy = 248.375;
  const double k1 = -0.28340811;
  const double k2 = 0.07395907;
  const double p1 = 0.00019359;
  const double p2 = 1.76187114e-05;

  VecX intrinsic;
  intrinsic.resize(8);
  intrinsic << fx, fy, cx, cy, k1, k2, p1, p2;

  return std::make_shared<CameraGeometry>(camera_index,
                                          camera_model,
                                          resolution,
                                          intrinsic,
                                          extrinsic);
}

static AprilGridConfig setup_aprilgrid_config() {
  AprilGridConfig config;
  config.target_id = 0;
  config.tag_rows = 10;
  config.tag_cols = 10;
  config.tag_size = 0.08;
  config.tag_spacing = 0.3;
  config.tag_id_offset = 0;
  return config;
}

static std::shared_ptr<CalibTargetGeometry>
setup_target_geometry(const AprilGridConfig &config, const Vec7 &extrinsic) {
  return std::make_shared<CalibTargetGeometry>(config.target_id,
                                               extrinsic,
                                               config.get_object_points());
}

static void simulate_camera_measurements(
    const AprilGridConfig &config,
    const std::shared_ptr<CalibTargetGeometry> &target_geometry,
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const Vec7 sensor_pose,
    const Vec7 target_pose,
    const Vec7 imu_extrinsic,
    std::vector<int> &point_ids,
    Vec2s &keypoints,
    Vec3s &object_points) {
  // Get camera parameters
  const auto res = camera_geometry->resolution;
  const auto camera = camera_geometry->camera_model;
  const auto params = camera_geometry->intrinsic;
  const Vec7 camera_extrinsic = camera_geometry->extrinsic;

  // Get target parameters
  const Vec7 target_extrinsic = target_geometry->extrinsic;
  const AprilGrid aprilgrid{0, 0, config};
  const int tag_rows = aprilgrid.get_tag_rows();
  const int tag_cols = aprilgrid.get_tag_cols();

  // Form fiducial target to camera pose T_CiF
  const Mat4 T_SW = tf(sensor_pose).inverse();
  const Mat4 T_WT0 = tf(target_pose);
  const Mat4 T_T0Tj = tf(target_extrinsic);
  const Mat4 T_C0S = tf(imu_extrinsic);
  const Mat4 T_CiC0 = tf(camera_extrinsic).inverse();
  const Mat4 T_CiTj = T_CiC0 * T_C0S * T_SW * T_WT0 * T_T0Tj;

  // Simulate camera measurements
  for (int tag_id = 0; tag_id < (tag_rows * tag_cols); ++tag_id) {
    for (int corner_index = 0; corner_index < 4; ++corner_index) {
      const Vec3 p_Tj = config.get_object_point(tag_id, corner_index);
      const Vec3 p_Ci = tf_point(T_CiTj, p_Tj);
      Vec2 z{0.0, 0.0};
      if (camera->project(res, params, p_Ci, z) != 0) {
        continue;
      }

      point_ids.push_back(tag_id * 4 + corner_index);
      object_points.push_back(p_Tj);
      keypoints.push_back(z);
    }
  }
}

TEST(CalibCameraImuError, evaluate) {
  // Setup
  auto target_config = setup_aprilgrid_config();
  auto sensor_pose = setup_sensor_pose();
  auto target_pose = setup_target_pose();
  auto imu_extrinsic = setup_imu_extrinsic();
  auto imu_geometry = setup_imu_geometry(0, imu_extrinsic);
  auto camera_extrinsic = setup_camera_extrinsic();
  auto camera_geometry = setup_camera_geometry(0, camera_extrinsic);
  auto target_extrinsic = setup_target_extrinsic();
  auto target_geometry = setup_target_geometry(target_config, target_extrinsic);

  // Simulate camera measurements
  std::vector<int> point_ids;
  Vec2s keypoints;
  Vec3s object_points;
  simulate_camera_measurements(target_config,
                               target_geometry,
                               camera_geometry,
                               sensor_pose,
                               target_pose,
                               imu_extrinsic,
                               point_ids,
                               keypoints,
                               object_points);

  // Create residual block
  Mat2 covar = eye(2);
  const CalibCameraImuError::Mode mode = CalibCameraImuError::PIXEL_VELOCITY;
  const timestamp_t ts_km1 = 0;
  const timestamp_t ts_k = 1e9; // 1 second apart (so v_k is small)
  double time_delay = 0.0;
  auto res = CalibCameraImuError::
      create(mode,
             ts_km1,
             ts_k,
             camera_geometry,
             imu_geometry,
             target_geometry,
             sensor_pose.data(), // T_WS at k-1
             sensor_pose.data(), // T_WS at k
             target_pose.data(), // T_WT0
             &time_delay,
             point_ids[0],
             keypoints[0], // z_km1 (same as z_k, no motion)
             keypoints[0], // z_k
             covar);

  // Check residual size and parameter block sizes
  auto block_sizes = res->parameter_block_sizes();
  ASSERT_EQ(res->num_residuals(), 2);
  ASSERT_EQ(block_sizes[0], 7); // Sensor pose
  ASSERT_EQ(block_sizes[1], 7); // Target pose
  ASSERT_EQ(block_sizes[2], 3); // Target point
  ASSERT_EQ(block_sizes[3], 7); // Target extrinsic
  ASSERT_EQ(block_sizes[4], 7); // Imu extrinsic
  ASSERT_EQ(block_sizes[5], 7); // Camera extrinsic
  ASSERT_EQ(block_sizes[6], 8); // Camera parameters
  ASSERT_EQ(block_sizes[7], 1); // Time-delay

  // Check param pointers
  auto param_ptrs = res->get_param_ptrs();
  ASSERT_EQ(param_ptrs[0], sensor_pose.data());
  ASSERT_EQ(param_ptrs[1], target_pose.data());
  ASSERT_EQ(param_ptrs[2], target_geometry->points[point_ids[0]].data());
  ASSERT_EQ(param_ptrs[3], target_geometry->extrinsic.data());
  ASSERT_EQ(param_ptrs[4], imu_geometry->extrinsic.data());
  ASSERT_EQ(param_ptrs[5], camera_geometry->extrinsic.data());
  ASSERT_EQ(param_ptrs[6], camera_geometry->intrinsic.data());
  ASSERT_EQ(param_ptrs[7], &time_delay);

  // Check Jacobians
  const double h = 1e-8;
  const double tol = 1e-4;
  const bool verbose = false;
  EXPECT_TRUE(res->check_jacobian(0, "J_sensor_pose", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(1, "J_target_pose", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(2, "J_target_point", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(3, "J_target_extrinsic", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(4, "J_imu_extrinsic", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(5, "J_camera_extrinsic", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(6, "J_camera", h, tol, verbose));
  EXPECT_TRUE(res->check_jacobian(7, "J_time_delay", h, tol, verbose));
}

} // namespace cartesian
