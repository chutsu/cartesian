#include <gtest/gtest.h>

#include "camera/KannalaBrandt4.hpp"

namespace cartesian {

static void setup_test(Vec2i &cam_res, VecX &params) {
  cam_res = Vec2i{640, 480};

  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double k3 = 0.001;
  const double k4 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, k3, k4;
}

TEST(KannalaBrandt4, equi4_distort) {
  const int num_points = 100;
  const Vec4 dist_params{0.1, 0.01, 0.01, 0.01};
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  for (int i = 0; i < num_points; i++) {
    // Distort point
    const Vec3 point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 p{point(0) / point(2), point(1) / point(2)};
    const Vec2 p_d = equi4_distort(dist_params, p);

    // Use opencv to use equi distortion to distort point
    const std::vector<cv::Point2f> points{cv::Point2f(p(0), p(1))};
    const cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    const cv::Vec4f D(k1, k2, k3, k4);
    std::vector<cv::Point2f> image_points;
    cv::fisheye::distortPoints(points, image_points, K, D);
    const Vec2 expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p_d - expected).norm() < 1.0e-5);
  }
}

TEST(KannalaBrandt4, equi4_undistort) {
  const int num_points = 100;
  const Vec4 dist_params{0.1, 0.2, 0.3, 0.4};

  for (int i = 0; i < num_points; i++) {
    // Distort point
    const Vec3 point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 p{point.x() / point.z(), point.y() / point.z()};
    const Vec2 p_d = equi4_distort(dist_params, p);
    const Vec2 p_ud = equi4_undistort(dist_params, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_d.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p - p_ud).norm() < 1.0e-5);
  }
}

TEST(KannalaBrandt4, project) {
  // Setup
  KannalaBrandt4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Test pinhole equi4 project
  const Vec3 p_C{0.1, 0.2, 1.0};
  Vec2 z_hat;
  int retval = camera.project(cam_res, params, p_C, z_hat);
  ASSERT_TRUE(retval == 0);

  // Test OpenCV's version
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double k1 = params(4);
  const double k2 = params(5);
  const double k3 = params(6);
  const double k4 = params(7);
  const cv::Point3f obj_pt(p_C.x(), p_C.y(), p_C.z());
  const cv::Mat K = cv::Mat(cv::Matx33d(fx, 0, cx, 0, fy, cy, 0, 0, 1), true);
  const cv::Mat D = cv::Mat(cv::Matx41d(k1, k2, k3, k4), true);
  const cv::Mat rvec = cv::Mat(cv::Matx31d(0, 0, 0), true);
  const cv::Mat tvec = cv::Mat(cv::Matx31d(0, 0, 0), true);
  const std::vector<cv::Point3f> object_points = {obj_pt};
  std::vector<cv::Point2f> image_points;
  cv::fisheye::projectPoints(object_points, image_points, rvec, tvec, K, D);

  const auto z_hat_gnd = Vec2{image_points[0].x, image_points[0].y};
  ASSERT_TRUE(fltcmp(z_hat_gnd.x(), z_hat.x()) == 0);
  ASSERT_TRUE(fltcmp(z_hat_gnd.y(), z_hat.y()) == 0);
}

TEST(KannalaBrandt4, project_jacobian) {
  // Setup
  KannalaBrandt4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Analytical jacobian
  const Vec3 p_C{0.1, 0.2, 1.0};
  const MatX J = camera.project_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  MatX fdiff = zeros(2, 3);

  Vec2 z_hat;
  camera.project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 3; i++) {
    Vec3 p_C_diff = p_C;
    p_C_diff(i) += step;

    Vec2 z_hat_prime;
    camera.project(cam_res, params, p_C_diff, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

TEST(KannalaBrandt4, params_jacobian) {
  // Setup
  KannalaBrandt4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Analytical jacobian
  const Vec3 p_C{0.1, 0.2, 5.0};
  const MatX J = camera.params_jacobian(params, p_C);

  // Numerical diff
  const double step = 1e-8;
  MatX fdiff = zeros(2, 8);

  Vec2 z_hat;
  camera.project(cam_res, params, p_C, z_hat);

  for (int i = 0; i < 8; i++) {
    VecX params_diff = params;
    params_diff(i) += step;

    Vec2 z_hat_prime;
    camera.project(cam_res, params_diff, p_C, z_hat_prime);
    fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
  }

  ASSERT_TRUE((J - fdiff).norm() < 1e-4);
}

TEST(KannalaBrandt4, undistort) {
  // Setup
  KannalaBrandt4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Project without distortion
  const Vec3 p_C{0.1, 0.2, 1.0};
  Vec2 z_hat_gnd;
  pinhole_project(cam_res, params.head(4), p_C, z_hat_gnd);

  // Project with distortion and undistort
  Vec2 z_hat;
  camera.project(cam_res, params, p_C, z_hat);
  z_hat = camera.undistort(params, z_hat);

  ASSERT_FLOAT_EQ(z_hat_gnd.x(), z_hat.x());
  ASSERT_FLOAT_EQ(z_hat_gnd.y(), z_hat.y());
}

} // namespace cartesian
