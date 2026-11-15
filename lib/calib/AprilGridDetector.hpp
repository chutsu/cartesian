#pragma once

#include "../core/Core.hpp"
#include "AprilGrid.hpp"
#include "AprilGridConfig.hpp"

// AprilTags3 by Ed Olsen
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/getopt.h"
}

// AprilTags by Michael Kaess
#include "ethz_apriltag/TagDetector.h"
#include "ethz_apriltag/Tag36h11.h"

namespace cartesian {

/** AprilGrid Detector **/
class AprilGridDetector {
private:
  // Settings
  int min_border_dist_ = 5;
  bool refine_corners_ = false;
  std::string detector_type_ = "kaess";
  std::map<int, AprilGridConfig> target_configs_;
  std::map<int, int> target_lut_; // tag_id, target_id

  // Form target lut
  void form_target_lut();

  // Ed Olsen's AprilTag detector
  apriltag_family_t *olsen_tf_ = nullptr;
  apriltag_detector_t *olsen_detector_ = nullptr;

  void olsen_setup();
  void olsen_cleanup();
  void olsen_detect(const cv::Mat &image,
                    std::vector<int> &tag_ids,
                    std::vector<int> &corner_indicies,
                    std::vector<Vec2> &keypoints);

  // Michale Kaess's AprilTag detector
  std::unique_ptr<ethz_apriltag::TagDetector> kaess_detector_;

  void kaess_setup();
  void kaess_detect(const cv::Mat &image,
                    std::vector<int> &tag_ids,
                    std::vector<int> &corner_indicies,
                    std::vector<Vec2> &keypoints);

public:
  /** Constructor / Destructor **/
  AprilGridDetector() = delete;
  AprilGridDetector(const AprilGridConfig &target_config);
  AprilGridDetector(const std::map<int, AprilGridConfig> &target_configs);
  virtual ~AprilGridDetector();

  /** Set detector */
  void set_detector(const std::string &detector_type);

  /**
   * Refine the tag corners with cv::cornerSubPix. Off by default, and not
   * recommended: the corners of both detectors come from lines fitted to whole
   * tag edges, which are more accurate than a local search around the corner.
   * In simulation it increased the corner error from 0.09 px to 0.3 px (kaess)
   * and from 0.18 px to 0.3 px (olsen).
   */
  void set_refine_corners(const bool refine_corners);

  /** Detect AprilGrid **/
  std::vector<std::shared_ptr<AprilGrid>> detect(const timestamp_t ts,
                                                 const int camera_id,
                                                 const cv::Mat &image);
};

} // namespace cartesian
