#include "CharucoDetector.hpp"

namespace cartesian {

CharucoDetector::CharucoDetector(const CharucoConfig &config)
    : config_{config}, board_{charuco_board(config)},
      detector_{std::make_unique<cv::aruco::CharucoDetector>(board_)} {}

std::shared_ptr<Charuco> CharucoDetector::detect(const timestamp_t ts,
                                                 const int camera_id,
                                                 const cv::Mat &image) const {
  assert(image.channels() == 1);
  auto target = std::make_shared<Charuco>(ts, camera_id, config_);

  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  detector_->detectBoard(image, corners, ids);
  for (size_t i = 0; i < ids.size(); ++i) {
    target->add(ids[i], Vec2{corners[i].x, corners[i].y});
  }
  return target;
}

} // namespace cartesian
