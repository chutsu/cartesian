#include <gtest/gtest.h>

#include "calib/AprilGrid.hpp"
#include "calib/AprilGridConfig.hpp"
#include "calib/AprilGridDetector.hpp"

#define TEST_IMAGE TEST_DATA "/aprilgrid/aprilgrid.png"
#define TEST_IMAGE2 TEST_DATA "/aprilgrid/multiple_aprilgrids.png"

namespace cartesian {

TEST(AprilGridDetector, detect) {
  const int target_id = 0;
  const int tag_rows = 6;
  const int tag_cols = 6;
  const double tag_size = 0.088;
  const double tag_spacing = 0.3;
  AprilGridConfig config{target_id, tag_rows, tag_cols, tag_size, tag_spacing};
  AprilGridDetector detector(config);

  const timestamp_t ts = 0;
  const int camera_id = 0;
  const auto img = cv::imread(TEST_IMAGE, cv::IMREAD_GRAYSCALE);
  auto grids = detector.detect(ts, camera_id, img);
  ASSERT_EQ(grids.size(), 1);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  grids[0]->get_measurements(tag_ids,
                             corner_indicies,
                             keypoints,
                             object_points);
  ASSERT_EQ(tag_ids.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(corner_indicies.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(keypoints.size(), tag_rows * tag_cols * 4);
  ASSERT_EQ(object_points.size(), tag_rows * tag_cols * 4);
}

TEST(AprilGridDetector, detect_multiple) {
  // Setup target configs
  int num_targets = 4;
  std::map<int, AprilGridConfig> target_configs;
  for (int target_id = 0; target_id < num_targets; ++target_id) {
    AprilGridConfig config;
    config.target_id = target_id;
    config.tag_rows = 4;
    config.tag_cols = 4;
    config.tag_size = 1.0;
    config.tag_spacing = 0.1;
    config.tag_id_offset = target_id * (config.tag_rows * config.tag_cols);

    target_configs[target_id] = config;
  }

  // Setup detector
  AprilGridDetector detector(target_configs);
  detector.set_detector("olsen");

  // Detect multiple aprilgrid targets
  const timestamp_t ts = 0;
  const int camera_id = 0;
  const auto img = cv::imread(TEST_IMAGE2, cv::IMREAD_GRAYSCALE);
  auto grids = detector.detect(ts, camera_id, img);
  ASSERT_EQ(grids.size(), 4);

  // Check each aprilgrid has astleast tag_rows * tag_cols * 4 corners detected
  const int expected_measurements = 4 * 4 * 4; // tag_rows * tag_cols * 4
  for (int i = 0; i < num_targets; ++i) {
    ASSERT_TRUE(grids[i]->detected());
    ASSERT_EQ(grids[i]->target_id, i);
    ASSERT_EQ(grids[i]->get_tag_rows(), 4);
    ASSERT_EQ(grids[i]->get_tag_cols(), 4);
    ASSERT_EQ(grids[i]->get_tag_size(), 1.0);
    ASSERT_EQ(grids[i]->get_tag_spacing(), 0.1);
    ASSERT_EQ(grids[i]->get_tag_id_offset(), i * 16);

    std::vector<int> point_ids;
    Vec2s keypoints;
    Vec3s object_points;
    grids[i]->get_measurements(point_ids, keypoints, object_points);
    ASSERT_EQ(point_ids.size(), expected_measurements);
    ASSERT_EQ(keypoints.size(), expected_measurements);
    ASSERT_EQ(object_points.size(), expected_measurements);
  }

  // Debug image
  bool debug = false;
  if (debug) {
    // Load image
    auto viz = cv::imread(TEST_IMAGE2, cv::IMREAD_COLOR);

    // Draw calibration targete
    for (int i = 0; i < num_targets; ++i) {
      viz = grids[i]->draw(viz, 5.0);

      // Compute center of detected keypoints in image coordinates
      std::vector<int> point_ids;
      Vec2s keypoints;
      Vec3s object_points;
      grids[i]->get_measurements(point_ids, keypoints, object_points);
      Vec2 center(0, 0);
      for (const auto &kp : keypoints) {
        center += kp;
      }
      center /= keypoints.size();

      // Draw target ID at center
      const int font = cv::FONT_HERSHEY_PLAIN;
      const double font_scale = 5;
      const int thickness = 6;
      const cv::Scalar text_color(0, 0, 255);
      std::string text = std::to_string(grids[i]->target_id);
      cv::putText(viz,
                  text,
                  cv::Point2f(center.x(), center.y()),
                  font,
                  font_scale,
                  text_color,
                  thickness);
    }

    // Visualize
    cv::imshow("Detected", viz);
    cv::waitKey();
  }
}

} // namespace cartesian
