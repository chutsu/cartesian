#pragma once
#include <vector>

#include "../core/Core.hpp"
#include "CalibTarget.hpp"

namespace cartesian {

/** Calibration target detector */
class CalibTargetDetector {
protected:
  bool refine_corners_ = true; // Refine the detected corners to sub-pixel

public:
  CalibTargetDetector(const bool refine_corners = true);
  virtual ~CalibTargetDetector() = default;

  /**
   * Detect the calibration targets in a grayscale image. Every target of the
   * detector is returned, also if it was not found, check `detected()`.
   */
  virtual std::vector<CalibTargetPtr> detect(const timestamp_t ts,
                                             const int camera_id,
                                             const cv::Mat &image) = 0;

  /** Refine the detected corners to sub-pixel accuracy, or not */
  void set_refine_corners(const bool refine_corners);

  /**
   * Refine corners to sub-pixel accuracy with cv::cornerSubPix().
   *
   * `half_window` is the half size of the search window [px] (the window is
   * 2 * half_window + 1 pixels wide), which should not reach other structures
   * in the image than the corner itself. A corner is kept where it was if its
   * window does not fit in the image, or if refinement fails or moves it by
   * more than `max_shift` [px].
   */
  static void refine_corners_subpix(const cv::Mat &image,
                                    std::vector<Vec2> &corners,
                                    const int half_window,
                                    const double max_shift);

  /**
   * Same as above, with a search window per corner. A corner with a
   * `half_windows` entry below 1 is not refined.
   */
  static void refine_corners_subpix(const cv::Mat &image,
                                    std::vector<Vec2> &corners,
                                    const std::vector<int> &half_windows,
                                    const double max_shift);
};

} // namespace cartesian
