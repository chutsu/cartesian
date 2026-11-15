#include "CharucoDetector.hpp"

#include <algorithm>

#include "CornerRefinement.hpp"

namespace cartesian {

static cv::aruco::DetectorParameters
make_detector_params(const bool refine_corners) {
  cv::aruco::DetectorParameters params;
  if (refine_corners) {
    params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  }
  return params;
}

CharucoDetector::CharucoDetector(const CharucoConfig &config,
                                 const bool refine_corners)
    : config_{config}, refine_corners_{refine_corners}, board_{charuco_board(
                                                            config)},
      detector_{std::make_unique<
          cv::aruco::CharucoDetector>(board_,
                                      cv::aruco::CharucoParameters{},
                                      make_detector_params(refine_corners))} {}

std::shared_ptr<Charuco> CharucoDetector::detect(const timestamp_t ts,
                                                 const int camera_id,
                                                 const cv::Mat &image) const {
  assert(image.channels() == 1);
  auto target = std::make_shared<Charuco>(ts, camera_id, config_);

  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  detector_->detectBoard(image, corners, ids);

  // OpenCV's ChArUco corners are half a pixel off in x and y, with respect to
  // the convention where the center of pixel (0, 0) is at (0, 0) (measured on
  // rendered boards, where the corners are known exactly)
  std::vector<Vec2> points;
  for (const auto &corner : corners) {
    points.emplace_back(corner.x - 0.5, corner.y - 0.5);
  }

  // Refine the corners, in a window of a fraction of the distance to the
  // nearest neighbouring corner. The markers inside the squares are close to
  // the corners, so the window has to be small: the error is lowest with 0.2
  // and grows quickly beyond 0.3.
  if (refine_corners_ && points.size() > 1) {
    std::map<int, size_t> index; // corner id -> index
    for (size_t i = 0; i < ids.size(); ++i) {
      index[ids[i]] = i;
    }
    const int row_corners = config_.cols - 1;
    const double ratio = 0.2;
    std::vector<int> half_windows(points.size(), 0);
    for (const auto &[id, i] : index) {
      double nearest = 1e9;
      for (const int other :
           {id - 1, id + 1, id - row_corners, id + row_corners}) {
        const bool same_row = (other == id - 1 || other == id + 1)
                                  ? (other / row_corners == id / row_corners)
                                  : true;
        if (same_row && index.count(other)) {
          nearest =
              std::min(nearest, (points[index.at(other)] - points[i]).norm());
        }
      }
      if (nearest < 1e8) {
        half_windows[i] = static_cast<int>(std::floor(ratio * nearest));
      }
    }
    refine_corners_subpix(image, points, half_windows, 3.0);
  }

  for (size_t i = 0; i < ids.size(); ++i) {
    target->add(ids[i], points[i]);
  }
  return target;
}

} // namespace cartesian
