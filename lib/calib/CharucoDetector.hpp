#pragma once
#include <memory>

#include <opencv2/objdetect/charuco_detector.hpp>

#include "../core/Core.hpp"
#include "Charuco.hpp"

namespace cartesian {

/** ChArUco Detector */
class CharucoDetector {
private:
  CharucoConfig config_;
  bool refine_corners_ = true;
  cv::aruco::CharucoBoard board_;
  std::unique_ptr<cv::aruco::CharucoDetector> detector_;

public:
  CharucoDetector() = delete;

  /**
   * `refine_corners` refines the marker corners, and then the board corners,
   * to sub-pixel accuracy. In simulation it reduces the corner error from
   * 0.38 px to 0.25 px.
   */
  CharucoDetector(const CharucoConfig &config,
                  const bool refine_corners = true);
  virtual ~CharucoDetector() = default;

  /**
   * Detect ChArUco board in a grayscale image. The board is returned even if
   * it was not found, check `detected()`.
   */
  std::shared_ptr<Charuco> detect(const timestamp_t ts,
                                  const int camera_id,
                                  const cv::Mat &image) const;
};

} // namespace cartesian
