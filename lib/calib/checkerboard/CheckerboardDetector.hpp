#pragma once
#include <memory>

#include "../../core/Core.hpp"
#include "../CalibTargetDetector.hpp"
#include "Checkerboard.hpp"

namespace cartesian {

/** Checkerboard Detector */
class CheckerboardDetector : public CalibTargetDetector {
private:
  CheckerboardConfig config_;

public:
  CheckerboardDetector() = delete;

  /**
   * `refine_corners` refines the corners with cv::cornerSubPix() after the
   * detection, in a window of 0.4 times the distance to the nearest
   * neighbouring corner. In simulation it halves the corner error (from 0.16 px
   * to 0.07 px).
   */
  CheckerboardDetector(const CheckerboardConfig &config,
                       const bool refine_corners = true);
  virtual ~CheckerboardDetector() = default;

  /** Detect the checkerboard, as a one element vector */
  std::vector<CalibTargetPtr> detect(const timestamp_t ts,
                                     const int camera_id,
                                     const cv::Mat &image) override;

  /**
   * Detect Checkerboard in a grayscale image. The checkerboard is returned
   * even if it was not found, check `detected()`.
   */
  std::shared_ptr<Checkerboard> detect_board(const timestamp_t ts,
                                             const int camera_id,
                                             const cv::Mat &image) const;
};

} // namespace cartesian
