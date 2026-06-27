#pragma once
#include <opencv2/opencv.hpp>

#include "core/Core.hpp"
#include "FeatureGrid.hpp"

namespace cartesian {

// Forward Declation
struct FeatureGrid;

/**
 * Grid detector
 */
struct GridDetector {
  int max_keypoints = 100;
  int grid_rows = 3;
  int grid_cols = 4;

  // cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
  // cv::Ptr<cv::Feature2D> detector = cv::FastFeatureDetector::create();
  cv::Ptr<cv::GFTTDetector> detector = cv::GFTTDetector::create();

  double quality_level = 0.01;
  double min_distance = 10;
  cv::Mat mask;
  int block_size = 3;
  bool use_harris = false;
  double k = 0.04;

  GridDetector() = default;
  virtual ~GridDetector() = default;

  void detect(
      const cv::Mat &image,
      std::vector<cv::KeyPoint> &kps_new,
      const std::vector<cv::KeyPoint> &kps_prev = std::vector<cv::KeyPoint>());

  cv::Mat debug(const cv::Mat &image,
                const std::vector<cv::KeyPoint> &kps_new,
                const std::vector<cv::KeyPoint> &kps_prev) const;
};

} // namespace cartesian
