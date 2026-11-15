#include <gtest/gtest.h>

#include "camera/BrownConrady4.hpp"
#include "ceres/ReprojError.hpp"
#include "core/Core.hpp"

namespace cartesian {

static Vec7 setup_pose() {
  const Mat3 C_WB = euler321(Vec3{-M_PI / 2.0, 0.0, -M_PI / 2.0});
  const Vec3 r_WB{0.0, 0.0, 0.0};
  const Mat4 T_WB = tf(C_WB, r_WB);
  return tf_vec(T_WB);
}

static Vec3 setup_point() { return Vec3{1.0, 0.1, 0.2}; }

static Vec7 setup_camera_extrinsic() {
  const Mat3 C_BCi = euler321(Vec3{0.0, 0.0, 0.0});
  const Vec3 r_BCi{0.0, 0.0, 0.0};
  const Mat4 T_BCi = tf(C_BCi, r_BCi);
  return tf_vec(T_BCi);
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

TEST(ReprojError, evaluate) {
  // Setup
  auto pose = setup_pose();
  auto point = setup_point();
  auto cam_ext = setup_camera_extrinsic();
  auto cam_geom = setup_camera_geometry(0, cam_ext);

  // Simulate camera measurement
  const auto res = cam_geom->resolution;
  const auto camera = cam_geom->camera_model;
  const auto params = cam_geom->intrinsic;
  const Mat4 T_WB = tf(pose);
  const Mat4 T_BCi = tf(cam_ext);
  const Mat4 T_CiW = T_BCi.inverse() * T_WB.inverse();
  const Vec3 p_C = tf_point(T_CiW, point);
  Vec2 z{0.0, 0.0};
  ASSERT_EQ(camera->project(res, params, p_C, z), 0);

  // Create residual block
  const Mat2 covar = eye(2);
  auto r = ReprojError::create(cam_geom, pose.data(), point.data(), z, covar);

  // Check Jacobians
  ASSERT_TRUE(r->check_jacobian(0, "J_pose"));
  ASSERT_TRUE(r->check_jacobian(1, "J_point"));
  ASSERT_TRUE(r->check_jacobian(2, "J_cam_ext"));
  ASSERT_TRUE(r->check_jacobian(3, "J_camera_intrinsic"));
}

} // namespace cartesian
