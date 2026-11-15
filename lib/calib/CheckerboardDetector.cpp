#include "CheckerboardDetector.hpp"

#include <opencv2/calib3d.hpp>

namespace cartesian {

CheckerboardDetector::CheckerboardDetector(const CheckerboardConfig &config)
    : config_{config} {}

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

  for (size_t i = 0; i < corners.size(); ++i) {
    target->add(i, Vec2{corners[i].x, corners[i].y});
  }
  return target;
}

} // namespace cartesian
