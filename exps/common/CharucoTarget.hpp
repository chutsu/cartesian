#pragma once
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

#include "DetectorTarget.hpp"

namespace cartesian {

/** ChArUco board */
struct CharucoTarget : DetectorTarget {
  int cols = 9;
  int rows = 8;
  double square_size = 0.07;
  double marker_size = 0.056; // 5x5 markers + border = 7 modules
  cv::aruco::CharucoBoard board;
  cv::aruco::CharucoDetector detector;

  CharucoTarget();

  std::string name() const override;
  double width() const override;
  double height() const override;
  std::map<int, Vec3> corners() const override;
  std::map<int, Vec2> detect(const cv::Mat &image) override;
  cv::Mat render_board() const override;
};

} // namespace cartesian
