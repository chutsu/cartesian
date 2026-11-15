#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/Checkerboard.hpp"
#include "calib/CheckerboardDetector.hpp"

namespace cartesian {

static CheckerboardConfig test_config() {
  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = 8; // An odd x even number of squares has no ambiguity
  config.cols = 9;
  config.square_size = 0.05;
  return config;
}

/** Render the checkerboard with a white margin of one square */
static cv::Mat render_checkerboard(const CheckerboardConfig &config,
                                   const int square_px) {
  cv::Mat board(config.rows * square_px,
                config.cols * square_px,
                CV_8UC1,
                cv::Scalar{255});
  for (int r = 0; r < config.rows; ++r) {
    for (int c = 0; c < config.cols; ++c) {
      if ((r + c) % 2 == 0) {
        cv::rectangle(board,
                      cv::Rect{c * square_px, r * square_px, square_px, square_px},
                      cv::Scalar{0},
                      cv::FILLED);
      }
    }
  }

  cv::Mat image;
  cv::copyMakeBorder(board,
                     image,
                     square_px,
                     square_px,
                     square_px,
                     square_px,
                     cv::BORDER_CONSTANT,
                     cv::Scalar{255});
  return image;
}

/** Max distance between the keypoints and the homography of the object points */
static double homography_error(const CornerTarget &target) {
  std::vector<int> ids;
  Vec2s keypoints;
  Vec3s object_points;
  target.get_measurements(ids, keypoints, object_points);

  std::vector<cv::Point2f> src;
  std::vector<cv::Point2f> dst;
  for (size_t i = 0; i < ids.size(); ++i) {
    src.emplace_back(object_points[i].x(), object_points[i].y());
    dst.emplace_back(keypoints[i].x(), keypoints[i].y());
  }
  const cv::Mat H = cv::findHomography(src, dst, 0);
  std::vector<cv::Point2f> projected;
  cv::perspectiveTransform(src, projected, H);

  double max_error = 0.0;
  for (size_t i = 0; i < dst.size(); ++i) {
    max_error = std::max(max_error, cv::norm(projected[i] - dst[i]));
  }
  return max_error;
}

TEST(Checkerboard, object_points) {
  const auto config = test_config();
  const auto points = config.get_object_points();
  ASSERT_EQ(config.get_num_corners(), 7 * 8);
  ASSERT_EQ(points.size(), 7 * 8);

  // Row major, starting at the origin
  ASSERT_NEAR((points.at(0) - Vec3{0.0, 0.0, 0.0}).norm(), 0.0, 1e-12);
  ASSERT_NEAR((points.at(1) - Vec3{0.05, 0.0, 0.0}).norm(), 0.0, 1e-12);
  ASSERT_NEAR((points.at(8) - Vec3{0.0, 0.05, 0.0}).norm(), 0.0, 1e-12);
  ASSERT_NEAR((points.at(55) - Vec3{7 * 0.05, 6 * 0.05, 0.0}).norm(), 0.0, 1e-12);
}

TEST(Checkerboard, measurements) {
  Checkerboard target{0, 0, test_config()};
  ASSERT_FALSE(target.detected());
  ASSERT_EQ(target.get_num_detected(), 0);

  target.add(3, Vec2{10.0, 20.0});
  target.add(9, Vec2{30.0, 40.0});
  ASSERT_TRUE(target.detected());
  ASSERT_EQ(target.get_num_detected(), 2);
  ASSERT_TRUE(target.has(3));
  ASSERT_FALSE(target.has(4));

  std::vector<int> ids;
  Vec2s keypoints;
  Vec3s object_points;
  target.get_measurements(ids, keypoints, object_points);
  ASSERT_EQ(ids.size(), 2);
  ASSERT_EQ(ids[0], 3);
  ASSERT_NEAR((keypoints[0] - Vec2{10.0, 20.0}).norm(), 0.0, 1e-12);
  ASSERT_NEAR((object_points[0] - Vec3{3 * 0.05, 0.0, 0.0}).norm(), 0.0, 1e-12);

  target.remove(3);
  ASSERT_EQ(target.get_num_detected(), 1);
}

TEST(Checkerboard, save_load) {
  const auto path = fs::temp_directory_path() / "test_checkerboard_save.csv";
  Checkerboard target{123456789, 2, test_config()};
  target.add(3, Vec2{10.25, 20.5});
  target.add(9, Vec2{30.125, 40.0625});
  ASSERT_EQ(target.save(path), 0);

  const auto loaded = Checkerboard::load(path);
  ASSERT_EQ(loaded->ts, 123456789);
  ASSERT_EQ(loaded->camera_id, 2);
  ASSERT_EQ(loaded->target_type, "checkerboard");
  ASSERT_EQ(loaded->config.rows, 8);
  ASSERT_EQ(loaded->config.cols, 9);
  ASSERT_NEAR(loaded->config.square_size, 0.05, 1e-9);
  ASSERT_EQ(loaded->get_num_detected(), 2);
  ASSERT_NEAR((loaded->keypoints.at(3) - Vec2{10.25, 20.5}).norm(), 0.0, 1e-9);
  ASSERT_NEAR((loaded->keypoints.at(9) - Vec2{30.125, 40.0625}).norm(), 0.0, 1e-9);
  fs::remove(path);
}

TEST(CheckerboardDetector, detect) {
  const auto config = test_config();
  const auto image = render_checkerboard(config, 60);

  CheckerboardDetector detector{config};
  const auto target = detector.detect(1000, 0, image);
  ASSERT_EQ(target->ts, 1000);
  ASSERT_TRUE(target->detected());
  ASSERT_EQ(target->get_num_detected(), config.get_num_corners());

  // The detected corners must agree with the object points (up to the
  // projection of the plane), which tells ids and object points are consistent
  ASSERT_LT(homography_error(*target), 0.1);
}

TEST(CheckerboardDetector, detect_rotated) {
  // Rotating the image must not change which object point a corner is matched
  // with, up to the projection of the plane
  const auto config = test_config();
  const auto image = render_checkerboard(config, 60);
  cv::Mat rotated;
  cv::rotate(image, rotated, cv::ROTATE_90_CLOCKWISE);

  CheckerboardDetector detector{config};
  const auto target = detector.detect(0, 0, rotated);
  ASSERT_EQ(target->get_num_detected(), config.get_num_corners());
  ASSERT_LT(homography_error(*target), 0.1);
}

TEST(CheckerboardDetector, not_found) {
  const cv::Mat blank(480, 752, CV_8UC1, cv::Scalar{200});
  CheckerboardDetector detector{test_config()};
  const auto target = detector.detect(0, 0, blank);
  ASSERT_FALSE(target->detected());
}

} // namespace cartesian
