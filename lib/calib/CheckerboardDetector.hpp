#pragma once
#include <memory>

#include "../core/Core.hpp"
#include "Checkerboard.hpp"

namespace cartesian {

/** Checkerboard Detector */
class CheckerboardDetector {
private:
  CheckerboardConfig config_;

public:
  CheckerboardDetector() = delete;
  CheckerboardDetector(const CheckerboardConfig &config);
  virtual ~CheckerboardDetector() = default;

  /**
   * Detect Checkerboard in a grayscale image. The checkerboard is returned
   * even if it was not found, check `detected()`.
   */
  std::shared_ptr<Checkerboard> detect(const timestamp_t ts,
                                       const int camera_id,
                                       const cv::Mat &image) const;
};

} // namespace cartesian
