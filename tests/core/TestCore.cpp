#include <gtest/gtest.h>

#include "core/Core.hpp"

namespace cartesian {

TEST(Core, spread_keypoints) {
  // Load image
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.png";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  // Detect keypoints
  const cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  std::vector<cv::KeyPoint> kps;
  detector->detect(img, kps);

  // Spread keypoints
  const int min_dist = 5;
  const std::vector<cv::KeyPoint> kps_prev;
  const bool debug = false;
  const auto kps_new = spread_keypoints(img, kps, min_dist, kps_prev, debug);

  // Assert keypoints are a Euclidean distance away from each other
  for (const auto kp : kps_new) {
    for (const auto kp_ref : kps_new) {
      const int px = kp.pt.x;
      const int py = kp.pt.y;
      const int px_ref = kp_ref.pt.x;
      const int py_ref = kp_ref.pt.y;

      const int dx = std::abs(px_ref - px);
      const int dy = std::abs(py_ref - py);
      if (dx == 0 && dy == 0) {
        continue;
      }

      const int dist = sqrt(dx * dx + dy * dy);
      ASSERT_TRUE(dist >= min_dist);
    }
  }
}

// TEST(Core, optflow_track) {
//   const auto img_i_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
//   const auto img_j_path = "./test_data/frontend/cam1/1403715297312143104.jpg";
//   const auto img_i = cv::imread(img_i_path, cv::IMREAD_GRAYSCALE);
//   const auto img_j = cv::imread(img_j_path, cv::IMREAD_GRAYSCALE);
//
//   // Detect keypoints
//   // std::vector<cv::KeyPoint> kps_new;
//   // GridDetector det;
//   // det.detect(img_i, kps_new);
//   //
//   // // Optical-flow track
//   // std::vector<cv::KeyPoint> kps_i = kps_new;
//   // std::vector<cv::KeyPoint> kps_j = kps_new;
//   // std::vector<uchar> inliers;
//   // optflow_track(img_i, img_j, kps_i, kps_j, inliers);
// }

} // namespace cartesian
