#include <gtest/gtest.h>

#include "camera/CameraGeometry.hpp"
#include "camera/UnifiedCameraModel.hpp"

namespace cartesian {

static VecX make_params(const double fx, const double xi) {
  VecX params(9);
  params << fx, fx * 1.01, 376.0, 240.0, xi, -0.1, 0.02, 0.001, -0.001;
  return params;
}

static Vec3 random_point(const double max_angle_deg) {
  // Direction within max_angle_deg of the optical axis, at a random distance
  const double theta = deg2rad(randf(0.0, max_angle_deg));
  const double phi = randf(-M_PI, M_PI);
  const Vec3 dir{sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
  return dir * randf(0.5, 5.0);
}

TEST(UnifiedCameraModel, type) {
  UnifiedCameraModel model;
  ASSERT_EQ(model.type(), "UnifiedCameraModel");
}

TEST(UnifiedCameraModel, pinhole_when_xi_is_zero) {
  // Without distortion and with xi = 0 the model is a pinhole camera
  VecX params = make_params(400.0, 0.0);
  params.tail<4>().setZero();

  UnifiedCameraModel model;
  const Vec3 p{0.3, -0.2, 2.0};
  Vec2 z;
  ASSERT_EQ(model.project(Vec2i{752, 480}, params, p, z), 0);
  ASSERT_NEAR(z.x(), params(0) * p.x() / p.z() + params(2), 1e-9);
  ASSERT_NEAR(z.y(), params(1) * p.y() / p.z() + params(3), 1e-9);
}

TEST(UnifiedCameraModel, project_and_back_project) {
  UnifiedCameraModel model;
  const Vec2i res{752, 480};

  // Rays up to 100 degrees off the optical axis, for different xi
  for (const double xi : {0.5, 0.9, 1.0, 1.3}) {
    const VecX params = make_params(250.0, xi);
    int num_valid = 0;
    for (int i = 0; i < 200; ++i) {
      const Vec3 p = random_point(100.0);
      Vec2 z;
      if (model.project(res, params, p, z) != 0) {
        continue;
      }

      Vec3 ray;
      model.back_project(params, z, ray);
      ASSERT_NEAR(ray.norm(), 1.0, 1e-9);
      ASSERT_LT((ray - p.normalized()).norm(), 1e-6) << "xi: " << xi;
      num_valid++;
    }
    ASSERT_GT(num_valid, 20) << "xi: " << xi;
  }
}

TEST(UnifiedCameraModel, rays_beyond_90_degrees) {
  // A stereographic projection (xi = 1) sees rays more than 90 degrees off the
  // optical axis
  UnifiedCameraModel model;
  VecX params = make_params(200.0, 1.0);
  params.tail<4>().setZero();

  const Vec3 p{sin(deg2rad(110.0)), 0.0, cos(deg2rad(110.0))};
  Vec2 z;
  ASSERT_EQ(model.project(Vec2i{752, 480}, params, p, z), 0);
  ASSERT_GT(z.x(), params(2));

  Vec3 ray;
  model.back_project(params, z, ray);
  ASSERT_LT(ray.z(), 0.0);
  ASSERT_LT((ray - p).norm(), 1e-9);
}

TEST(UnifiedCameraModel, point_behind_camera) {
  UnifiedCameraModel model;
  const VecX params = make_params(250.0, 0.5);
  Vec2 z;
  ASSERT_NE(model.project(Vec2i{752, 480}, params, Vec3{0.0, 0.0, -1.0}, z), 0);
}

TEST(UnifiedCameraModel, project_jacobian) {
  UnifiedCameraModel model;
  const Vec2i res{752, 480};
  const double h = 1e-6;

  for (const double xi : {0.0, 0.7, 1.0, 1.2}) {
    const VecX params = make_params(250.0, xi);
    for (int i = 0; i < 20; ++i) {
      const Vec3 p = random_point(70.0);
      const MatX J = model.project_jacobian(params, p);
      ASSERT_EQ(J.rows(), 2);
      ASSERT_EQ(J.cols(), 3);

      MatX J_fd = zeros(2, 3);
      for (int k = 0; k < 3; ++k) {
        Vec3 p_plus = p;
        Vec3 p_minus = p;
        p_plus(k) += h;
        p_minus(k) -= h;
        Vec2 z_plus, z_minus;
        model.project(res, params, p_plus, z_plus);
        model.project(res, params, p_minus, z_minus);
        J_fd.col(k) = (z_plus - z_minus) / (2.0 * h);
      }
      ASSERT_LT((J - J_fd).norm(), 1e-4 * std::max(1.0, J_fd.norm()));
    }
  }
}

TEST(UnifiedCameraModel, params_jacobian) {
  UnifiedCameraModel model;
  const Vec2i res{752, 480};
  const double h = 1e-6;

  for (const double xi : {0.0, 0.7, 1.0, 1.2}) {
    const VecX params = make_params(250.0, xi);
    for (int i = 0; i < 20; ++i) {
      const Vec3 p = random_point(70.0);
      const MatX J = model.params_jacobian(params, p);
      ASSERT_EQ(J.rows(), 2);
      ASSERT_EQ(J.cols(), 9);

      MatX J_fd = zeros(2, 9);
      for (int k = 0; k < 9; ++k) {
        VecX params_plus = params;
        VecX params_minus = params;
        params_plus(k) += h;
        params_minus(k) -= h;
        Vec2 z_plus, z_minus;
        model.project(res, params_plus, p, z_plus);
        model.project(res, params_minus, p, z_minus);
        J_fd.col(k) = (z_plus - z_minus) / (2.0 * h);
      }
      ASSERT_LT((J - J_fd).norm(), 1e-4 * std::max(1.0, J_fd.norm()));
    }
  }
}

TEST(UnifiedCameraModel, undistort) {
  // Undistorting gives the pixel of a pinhole camera with the same fx, fy, cx,
  // and cy, for the same ray
  UnifiedCameraModel model;
  const Vec2i res{752, 480};
  const VecX params = make_params(250.0, 0.9);

  for (int i = 0; i < 50; ++i) {
    const Vec3 p = random_point(60.0);
    Vec2 z;
    if (model.project(res, params, p, z) != 0) {
      continue;
    }
    const Vec2 z_undist = model.undistort(params, z);
    ASSERT_NEAR(z_undist.x(), params(0) * p.x() / p.z() + params(2), 1e-5);
    ASSERT_NEAR(z_undist.y(), params(1) * p.y() / p.z() + params(3), 1e-5);
  }
}

TEST(UnifiedCameraModel, camera_geometry) {
  CameraGeometry camera{0, "UnifiedCameraModel", Vec2i{752, 480}, VecX{}};
  ASSERT_EQ(camera.camera_model->type(), "UnifiedCameraModel");
  ASSERT_EQ(camera.intrinsic.size(), 9);
  ASSERT_NEAR(camera.intrinsic(2), 376.0, 1e-9);
  ASSERT_NEAR(camera.intrinsic(3), 240.0, 1e-9);
  ASSERT_NEAR(camera.intrinsic(4), 1.0, 1e-9);
  ASSERT_NEAR(camera.intrinsic.tail<4>().norm(), 0.0, 1e-12);

  // The default camera sees the same around the optical axis as a pinhole
  // camera with a 90 degree field of view
  const Vec3 p{0.01, 0.0, 1.0};
  Vec2 z;
  ASSERT_EQ(camera.project(p, z), 0);
  ASSERT_NEAR(z.x() - 376.0, 376.0 * 0.01, 0.05);
}

} // namespace cartesian
