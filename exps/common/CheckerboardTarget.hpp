#pragma once
#include "DetectorTarget.hpp"

namespace cartesian {

/** Checkerboard */
struct CheckerboardTarget : DetectorTarget {
  // Squares (the detector looks for the inner corners, one less in each
  // direction). An odd x even number of squares avoids the 180 degree
  // orientation ambiguity.
  int cols = 9;
  int rows = 8;
  double square_size = 0.07;

  CheckerboardTarget() = default;

  std::string name() const override;
  double width() const override;
  double height() const override;
  std::map<int, Vec3> corners() const override;
  std::map<int, Vec2> detect(const cv::Mat &image) override;
  cv::Mat render_board() const override;
};

} // namespace cartesian
