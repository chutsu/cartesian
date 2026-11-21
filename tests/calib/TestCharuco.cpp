#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/charuco/Charuco.hpp"
#include "calib/charuco/CharucoDetector.hpp"

namespace cartesian {

static CharucoConfig test_config() {
  CharucoConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.07;
  config.marker_size = 0.056;
  config.dictionary = "DICT_5X5_250";
  return config;
}

/** Render the ChArUco board with a white margin of one square */
static cv::Mat render_charuco(const CharucoConfig &config, const int square_px) {
  cv::Mat board;
  charuco_board(config).generateImage(
      cv::Size{config.cols * square_px, config.rows * square_px},
      board,
      0,
      1);

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
static double homography_error(const CalibTarget &target) {
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

TEST(Charuco, object_points) {
  const auto config = test_config();
  const auto points = config.get_object_points();
  ASSERT_EQ(config.get_num_corners(), 7 * 8);
  ASSERT_EQ(points.size(), 7 * 8);

  // Row major, starting at the top left inner corner
  const double s = config.square_size;
  ASSERT_NEAR((points.at(0) - Vec3{s, s, 0.0}).norm(), 0.0, 1e-6);
  ASSERT_NEAR((points.at(1) - Vec3{2 * s, s, 0.0}).norm(), 0.0, 1e-6);
  ASSERT_NEAR((points.at(8) - Vec3{s, 2 * s, 0.0}).norm(), 0.0, 1e-6);
}

TEST(Charuco, bad_dictionary) {
  auto config = test_config();
  config.dictionary = "DICT_DOES_NOT_EXIST";
  ASSERT_ANY_THROW(config.get_object_points());
}

TEST(Charuco, save_load) {
  const auto path = fs::temp_directory_path() / "test_charuco_save.csv";
  Charuco target{42, 1, test_config()};
  target.add(0, Vec2{10.25, 20.5});
  target.add(30, Vec2{300.125, 40.0625});
  ASSERT_EQ(target.save(path), 0);

  const auto loaded = Charuco::load(path);
  ASSERT_EQ(loaded->ts, 42);
  ASSERT_EQ(loaded->camera_id, 1);
  ASSERT_EQ(loaded->target_type, "charuco");
  ASSERT_EQ(loaded->config.rows, 8);
  ASSERT_EQ(loaded->config.cols, 9);
  ASSERT_NEAR(loaded->config.square_size, 0.07, 1e-6);
  ASSERT_NEAR(loaded->config.marker_size, 0.056, 1e-6);
  ASSERT_EQ(loaded->config.dictionary, "DICT_5X5_250");
  ASSERT_EQ(loaded->get_num_detected(), 2);
  ASSERT_NEAR((loaded->keypoints.at(30) - Vec2{300.125, 40.0625}).norm(), 0.0, 1e-9);
  fs::remove(path);
}

TEST(CharucoDetector, detect) {
  const auto config = test_config();
  const auto image = render_charuco(config, 80);

  CharucoDetector detector{config};
  const auto target = detector.detect_board(1000, 0, image);
  ASSERT_EQ(target->ts, 1000);
  ASSERT_TRUE(target->detected());
  ASSERT_EQ(target->get_num_detected(), config.get_num_corners());
  ASSERT_LT(homography_error(*target), 0.2);
}

TEST(CharucoDetector, detect_partial) {
  // The markers identify the corners, so a partial board is still detected
  // with the correct ids
  const auto config = test_config();
  const auto image = render_charuco(config, 80);
  cv::Mat partial = image.clone();
  partial(cv::Rect{0, 0, image.cols / 2, image.rows / 3}).setTo(cv::Scalar{255});

  CharucoDetector detector{config};
  const auto target = detector.detect_board(0, 0, partial);
  ASSERT_TRUE(target->detected());
  ASSERT_LT(target->get_num_detected(), config.get_num_corners());
  ASSERT_GT(target->get_num_detected(), 10);
  ASSERT_LT(homography_error(*target), 0.2);
}

TEST(CharucoDetector, not_found) {
  const cv::Mat blank(480, 752, CV_8UC1, cv::Scalar{200});
  CharucoDetector detector{test_config()};
  const auto target = detector.detect_board(0, 0, blank);
  ASSERT_FALSE(target->detected());
}

TEST(CharucoDetector, detect_through_the_interface) {
  const auto config = test_config();
  const auto image = render_charuco(config, 80);

  CharucoDetector charuco_detector{config};
  CalibTargetDetector &detector = charuco_detector;
  const auto targets = detector.detect(1000, 2, image);
  ASSERT_EQ(targets.size(), 1);
  ASSERT_TRUE(targets[0]->detected());
  ASSERT_EQ(targets[0]->ts, 1000);
  ASSERT_EQ(targets[0]->camera_id, 2);
  ASSERT_EQ(targets[0]->get_num_detected(), config.get_num_corners());
}

} // namespace cartesian
