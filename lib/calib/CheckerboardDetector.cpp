#include "CheckerboardDetector.hpp"

#include <algorithm>

#include <opencv2/calib3d.hpp>

#include "CornerRefinement.hpp"

namespace cartesian {

CheckerboardDetector::CheckerboardDetector(const CheckerboardConfig &config,
                                           const bool refine_corners)
    : config_{config}, refine_corners_{refine_corners} {}

std::shared_ptr<Checkerboard>
CheckerboardDetector::detect(const timestamp_t ts,
                             const int camera_id,
                             const cv::Mat &image) const {
  assert(image.channels() == 1);
  auto target = std::make_shared<Checkerboard>(ts, camera_id, config_);

  // Detect inner corners, row major
  const cv::Size pattern{config_.get_inner_cols(), config_.get_inner_rows()};
  const int flags = cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
  std::vector<cv::Point2f> corners;
  if (cv::findChessboardCornersSB(image, pattern, corners, flags) == false) {
    return target;
  }

  std::vector<Vec2> points;
  for (const auto &corner : corners) {
    points.emplace_back(corner.x, corner.y);
  }

  // Refine the corners, in a window of less than half the distance to the
  // nearest neighbouring corner (squares can be small, or foreshortened)
  if (refine_corners_ && points.size() > 1) {
    const int rows = config_.get_inner_rows();
    const int cols = config_.get_inner_cols();
    const double ratio = 0.4;
    std::vector<int> half_windows;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const int i = r * cols + c;
        double nearest = 1e9;
        if (c > 0)
          nearest = std::min(nearest, (points[i] - points[i - 1]).norm());
        if (c + 1 < cols)
          nearest = std::min(nearest, (points[i] - points[i + 1]).norm());
        if (r > 0)
          nearest = std::min(nearest, (points[i] - points[i - cols]).norm());
        if (r + 1 < rows)
          nearest = std::min(nearest, (points[i] - points[i + cols]).norm());
        half_windows.push_back(static_cast<int>(std::floor(ratio * nearest)));
      }
    }
    refine_corners_subpix(image, points, half_windows, 3.0);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    target->add(i, points[i]);
  }
  return target;
}

} // namespace cartesian
