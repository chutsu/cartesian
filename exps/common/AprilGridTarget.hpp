#pragma once
#include "calib/AprilGridConfig.hpp"
#include "calib/AprilGridDetector.hpp"

#include "DetectorTarget.hpp"

namespace cartesian {

/** AprilGrid with (kalibr style) or without black squares between tags */
struct AprilGridTarget : DetectorTarget {
  std::string detector_type;
  bool kalibr_style;
  AprilGridConfig config;
  AprilGridDetector detector;

  /**
   * `detector_type` is "kaess" or "olsen". The two detectors use different
   * (incompatible) tag36h11 code tables and bit layouts, so the tag texture
   * is rendered to match the detector:
   *
   * - "kaess": ethz_apriltag (AprilTag 1), what kalibr printed grids use
   * - "olsen": apriltag (AprilTag 3)
   */
  AprilGridTarget(const bool kalibr_style,
                  const std::string &detector_type = "kaess",
                  const bool refine_corners = false);

  std::string name() const override;
  double width() const override;
  double height() const override;
  std::map<int, Vec3> corners() const override;
  std::map<int, Vec2> detect(const cv::Mat &image) override;

  /** Detect AprilGrid, with its tag ids and corner indices */
  std::shared_ptr<AprilGrid> detect_grid(const cv::Mat &image,
                                         const timestamp_t ts = 0);
  cv::Mat render_board() const override;

  /**
   * Make tag image with 1 pixel per module, 6x6 data bits and a black border:
   * 2 modules wide for kaess (10x10) and 1 module wide for olsen (8x8)
   */
  static cv::Mat make_tag(const std::string &detector_type, const int tag_id);

  /** Make the default AprilGrid config */
  static AprilGridConfig make_config();
};

} // namespace cartesian
