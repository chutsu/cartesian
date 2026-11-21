#pragma once
#include "../../core/Core.hpp"
#include "../CalibTarget.hpp"
#include "CharucoConfig.hpp"

namespace cartesian {

/** ChArUco board */
struct Charuco : CalibTarget {
  CharucoConfig config;

  Charuco(const timestamp_t &ts,
          const int camera_id,
          const CharucoConfig &config);
  virtual ~Charuco() = default;

  /** Save Charuco */
  int save(const fs::path &save_path) const override;

  /** Load Charuco */
  static std::shared_ptr<Charuco> load(const fs::path &data_path);

  /** Draw detected corners */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;
};

} // namespace cartesian
