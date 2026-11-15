#include <gtest/gtest.h>

#include "camera/BrownConrady4.hpp"

namespace cartesian {

static void setup_test(Vec2i &cam_res, VecX &params) {
  cam_res = Vec2i{640, 480};

  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double p1 = 0.001;
  const double p2 = 0.001;

  params.resize(8);
  params << fx, fy, cx, cy, k1, k2, p1, p2;
}

TEST(BrownConrady4, radtan4_distort) {
  const int nb_points = 100;
  const Vec4 dist_params{0.1, 0.01, 0.01, 0.01};
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double p1 = dist_params(2);
  const double p2 = dist_params(3);

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const Vec3 p{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 x{p(0) / p(2), p(1) / p(2)};
    const Vec2 pixel = radtan4_distort(dist_params, x);

    // Use opencv to use radtan distortion to distort point
    const std::vector<cv::Point3f> points{cv::Point3f(p(0), p(1), p(2))};
    const cv::Vec3f rvec;
    const cv::Vec3f tvec;
    const cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    const cv::Vec4f D(k1, k2, p1, p2);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(points, rvec, tvec, K, D, image_points);
    const Vec2 expected{image_points[0].x, image_points[0].y};

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << pixel.transpose() << std::endl;
    // std::cout << expected.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((pixel - expected).norm() < 1.0e-5);
  }
}

TEST(BrownConrady4, radtan4_undistort) {
  const int nb_points = 100;
  const Vec4 dist_params{0.1, 0.01, 0.01, 0.01};

  for (int i = 0; i < nb_points; i++) {
    // Distort point
    const Vec3 point{randf(-1.0, 1.0), randf(-1.0, 1.0), randf(5.0, 10.0)};
    const Vec2 p{point(0) / point(2), point(1) / point(2)};
    const Vec2 p_d = radtan4_distort(dist_params, p);
    const Vec2 p_ud = radtan4_undistort(dist_params, p_d);

    // // Debug
    // std::cout << p.transpose() << std::endl;
    // std::cout << p_ud.transpose() << std::endl;
    // std::cout << std::endl;

    ASSERT_TRUE((p - p_ud).norm() < 1.0e-5);
  }
}

TEST(BrownConrady4, project) {
  // Setup
  BrownConrady4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Test pinhole radtan4 project
  for (int i = 0; i < 1000; i++) {
    double px = randf(-0.5, 0.5);
    double py = randf(-0.5, 0.5);
    double pz = randf(0.1, 10.0);
    const Vec3 p_C{px, py, pz};
    Vec2 z_hat;
    int retval = camera.project(cam_res, params, p_C, z_hat);
    if (retval != 0) {
      continue;
    }

    // Test OpenCV's version
    const double fx = params(0);
    const double fy = params(1);
    const double cx = params(2);
    const double cy = params(3);
    const double k1 = params(4);
    const double k2 = params(5);
    const double p1 = params(6);
    const double p2 = params(7);
    const cv::Point3f obj_pt(p_C.x(), p_C.y(), p_C.z());
    const cv::Mat K = cv::Mat(cv::Matx33d(fx, 0, cx, 0, fy, cy, 0, 0, 1), true);
    const cv::Mat D = cv::Mat(cv::Matx41d(k1, k2, p1, p2), true);
    const cv::Mat rvec = cv::Mat(cv::Matx31d(0, 0, 0), true);
    const cv::Mat tvec = cv::Mat(cv::Matx31d(0, 0, 0), true);
    const std::vector<cv::Point3f> object_points = {obj_pt};
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rvec, tvec, K, D, image_points);

    const auto z_hat_gnd = Vec2{image_points[0].x, image_points[0].y};
    ASSERT_NEAR(z_hat_gnd.x(), z_hat.x(), 1e-4);
    ASSERT_NEAR(z_hat_gnd.y(), z_hat.y(), 1e-4);
  }
}

TEST(BrownConrady4, project_jacobian) {
  // Setup
  BrownConrady4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Analytical jacobian
  for (int i = 0; i < 100; i++) {
    double px = randf(-0.5, 0.5);
    double py = randf(-0.5, 0.5);
    double pz = randf(0.1, 10.0);
    const Vec3 p_C{px, py, pz};

    Vec2 z_hat;
    if (camera.project(cam_res, params, p_C, z_hat) != 0) {
      continue;
    }
    const MatX J = camera.project_jacobian(params, p_C);

    // Numerical diff
    const double step = 1e-8;
    MatX fdiff = zeros(2, 3);

    for (int i = 0; i < 3; i++) {
      Vec3 p_C_diff = p_C;
      p_C_diff(i) += step;

      Vec2 z_hat_prime;
      camera.project(cam_res, params, p_C_diff, z_hat_prime);
      fdiff.block(0, i, 2, 1) = (z_hat_prime - z_hat) / step;
    }
    ASSERT_TRUE((J - fdiff).norm() < 1e-4);
  }
}

TEST(BrownConrady4, params_jacobian) {
  // Setup
  BrownConrady4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  // Analytical jacobian
  const Vec3 p_C{0.1, 0.2, 1.0};
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

static Vec2 opencv_undistort_point(const VecX &cam_params, const Vec2 &kp) {
  std::vector<cv::Point2f> pts_in;
  pts_in.emplace_back(kp.x(), kp.y());

  std::vector<cv::Point2f> pts_out;

  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = cam_params(0);
  K.at<double>(1, 1) = cam_params(1);
  K.at<double>(0, 2) = cam_params(2);
  K.at<double>(1, 2) = cam_params(3);

  std::vector<double> D;
  D.push_back(cam_params(4));
  D.push_back(cam_params(5));
  D.push_back(cam_params(6));
  D.push_back(cam_params(7));

  cv::undistortPoints(pts_in, pts_out, K, D, cv::noArray(), K);
  const Vec2 kp_opencv{pts_out[0].x, pts_out[0].y};
  return kp_opencv;
}

TEST(BrownConrady4, undistort) {
  // Setup
  BrownConrady4 camera;
  Vec2i cam_res;
  VecX params;
  setup_test(cam_res, params);

  for (int i = 0; i < 100; i++) {
    // Project without distortion
    double px = randf(-0.5, 0.5);
    double py = randf(-0.5, 0.5);
    double pz = randf(0.1, 10.0);
    const Vec3 p_C{px, py, pz};
    Vec2 kp_gnd;
    if (pinhole_project(cam_res, params.head(4), p_C, kp_gnd) != 0) {
      continue;
    }

    // Project with distortion
    Vec2 kp_dist;
    camera.project(cam_res, params, p_C, kp_dist);

    // OpenCV undistort
    const Vec2 kp_opencv = opencv_undistort_point(params, kp_dist);

    // Project with distortion and undistort it
    const Vec2 kp_xyz = camera.undistort(params, kp_dist);

    ASSERT_FLOAT_EQ(kp_gnd.x(), kp_xyz.x());
    ASSERT_FLOAT_EQ(kp_gnd.y(), kp_xyz.y());
    ASSERT_NEAR(kp_opencv.x(), kp_xyz.x(), 1.0);
    ASSERT_NEAR(kp_opencv.y(), kp_xyz.y(), 1.0);
  }
}

} // namespace cartesian
