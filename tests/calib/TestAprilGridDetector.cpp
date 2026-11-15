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

TEST(AprilGridDetector, olsen_corners_have_no_half_pixel_offset) {
  // Render a tag of 80x80 pixels (8x8 modules of 10 pixels) with its outer
  // corners on pixel boundaries: in the convention where the center of pixel
  // (0, 0) is at (0, 0), they are at 59.5 and 139.5
  apriltag_family_t *family = tag36h11_create();
  image_u8_t *im = apriltag_to_image(family, 0);
  const cv::Mat tag_with_ring(im->height,
                              im->width,
                              CV_8UC1,
                              im->buf,
                              im->stride);
  const cv::Mat tag = tag_with_ring(cv::Rect{1, 1, 8, 8}).clone();
  image_u8_destroy(im);
  tag36h11_destroy(family);

  cv::Mat tag_scaled;
  cv::resize(tag, tag_scaled, cv::Size{80, 80}, 0, 0, cv::INTER_NEAREST);
  cv::Mat image(200, 200, CV_8UC1, cv::Scalar{255});
  tag_scaled.copyTo(image(cv::Rect{60, 60, 80, 80}));

  const AprilGridConfig config{0, 1, 1, 0.1, 0.0, 0};
  AprilGridDetector detector{config};
  detector.set_detector("olsen");
  const auto grids = detector.detect(0, 0, image);
  ASSERT_EQ(grids.size(), 1);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  grids[0]->get_measurements(tag_ids,
                             corner_indicies,
                             keypoints,
                             object_points);
  ASSERT_EQ(keypoints.size(), 4);

  const std::vector<Vec2> expected{{59.5, 59.5},
                                   {139.5, 59.5},
                                   {139.5, 139.5},
                                   {59.5, 139.5}};
  for (const auto &kp : keypoints) {
    double nearest = 1e9;
    for (const auto &p : expected) {
      nearest = std::min(nearest, (kp - p).norm());
    }
    ASSERT_LT(nearest, 0.2);
  }
}

} // namespace cartesian
