#include <gtest/gtest.h>

#include "calib/CalibProblem.hpp"
#include "ceres/CalibCameraError.hpp"

namespace cartesian {

static Vec7 setup_body_pose() {
  // Body pose in world frame: T_WC0
  // clang-format off
  Mat4 T_WC0;
  T_WC0 <<  1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
  T_WC0 = tf_perturb_rot(T_WC0, 0.01, 1);
  auto body_pose = tf_vec(T_WC0);
  // ^ Note: Due to numerical stability issues the translation component
  // cannot be 0 for checking jacobians
  // clang-format on

  return body_pose;
}

static Vec7 setup_target_pose() {
  // Target pose: T_WT0
  // clang-format off
  Mat4 T_WT0;
  T_WT0 <<  0.0,  0.0, -1.0, 0.5,
          -1.0,  0.0,  0.0, 0.001,
           0.0,  1.0,  0.0, 0.001,
           0.0,  0.0,  0.0, 1.0;
  T_WT0 = tf_perturb_rot(T_WT0, 0.01, 2);
  auto target_pose = tf_vec(T_WT0);
  // clang-format on

  return target_pose;
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

static Vec7 setup_target_extrinsic() {
  // Target extrinsics: T_T0Tj
  // clang-format off
  Mat4 T_T0Tj;
  T_T0Tj << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
  T_T0Tj = tf_perturb_rot(T_T0Tj, 0.01, 2);
  auto extrinsic = tf_vec(T_T0Tj);
  // clang-format on

  return extrinsic;
}

static std::shared_ptr<CameraGeometry>
setup_camera_geometry(const int camera_id, const Vec7 &extrinsic) {
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

  return std::make_shared<CameraGeometry>(camera_id,
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

void simulate_camera_measurements(
    const AprilGridConfig &config,
    const std::shared_ptr<CalibTargetGeometry> &target_geometry,
    const std::shared_ptr<CameraGeometry> &camera_geometry,
    const Vec7 body_pose,
    const Vec7 target_pose,
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

  // Form target target to camera pose T_CiF
  const Mat4 T_WC0 = tf(body_pose);
  const Mat4 T_WT0 = tf(target_pose);
  const Mat4 T_T0Tj = tf(target_extrinsic);
  const Mat4 T_C0Ci = tf(camera_extrinsic);
  const Mat4 T_CiTj = T_C0Ci.inverse() * T_WC0.inverse() * T_WT0 * T_T0Tj;

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

TEST(CalibCameraError, evaluate) {
  // Setup
  auto target_config = setup_aprilgrid_config();
  auto body_pose = setup_body_pose();
  auto target_pose = setup_target_pose();
  auto camera_extrinsic = setup_camera_extrinsic();
  auto target_extrinsic = setup_target_extrinsic();
  auto camera_geometry = setup_camera_geometry(0, camera_extrinsic);
  auto target_geometry = setup_target_geometry(target_config, target_extrinsic);

  // Simulate camera measurements
  std::vector<int> point_ids;
  Vec2s keypoints;
  Vec3s object_points;
  simulate_camera_measurements(target_config,
                               target_geometry,
                               camera_geometry,
                               body_pose,
                               target_pose,
                               point_ids,
                               keypoints,
                               object_points);

  // Create residual block
  const Mat2 covar = eye(2);
  const Mat4 T_WC0 = tf(body_pose);
  const Mat4 T_WT0 = tf(target_pose);
  const Mat4 T_BF = T_WC0.inverse() * T_WT0;
  VecX relpose = tf_vec(T_BF);
  auto resblock = CalibCameraError::create(camera_geometry,
                                           target_geometry,
                                           point_ids[0],
                                           relpose.data(),
                                           keypoints[0],
                                           covar);

  // Check residual size and parameter block sizes
  auto block_sizes = resblock->parameter_block_sizes();
  ASSERT_EQ(resblock->num_residuals(), 2);
  ASSERT_EQ(block_sizes[0], 7); // Relative camera pose T_C0T0
  ASSERT_EQ(block_sizes[1], 3); // Target point p_Tj
  ASSERT_EQ(block_sizes[2], 7); // Target extrinsic T_T0Tj
  ASSERT_EQ(block_sizes[3], 7); // Camera extrinsic
  ASSERT_EQ(block_sizes[4], 8); // Camera intrinsic

  // Check param pointers
  auto param_ptrs = resblock->get_param_ptrs();
  ASSERT_EQ(param_ptrs[0], relpose.data());
  ASSERT_EQ(param_ptrs[1], target_geometry->points[point_ids[0]].data());
  ASSERT_EQ(param_ptrs[2], target_geometry->extrinsic.data());
  ASSERT_EQ(param_ptrs[3], camera_geometry->extrinsic.data());
  ASSERT_EQ(param_ptrs[4], camera_geometry->intrinsic.data());

  // Check Jacobians
  const double h = 1e-8;
  const double tol = 1e-4;
  const bool debug = false;
  ASSERT_TRUE(resblock->check_jacobian(0, "J_relpose", h, tol, debug));
  ASSERT_TRUE(resblock->check_jacobian(1, "J_point", h, tol, debug));
  ASSERT_TRUE(resblock->check_jacobian(2, "J_target_extrinsic", h, tol, debug));
  ASSERT_TRUE(resblock->check_jacobian(3, "J_camera_extrinsic", h, tol, debug));
  ASSERT_TRUE(resblock->check_jacobian(4, "J_camera", h, tol, debug));
}

} // namespace cartesian
