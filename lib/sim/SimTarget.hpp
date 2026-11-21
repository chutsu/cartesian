#pragma once
#include <functional>
#include <map>
#include <string>

#include "../calib/aprilgrid/AprilGridConfig.hpp"
#include "../calib/charuco/Charuco.hpp"
#include "../calib/checkerboard/Checkerboard.hpp"
#include "../core/Core.hpp"

namespace cartesian {

/**
 * Calibration board to simulate: its geometry, the texture to render, and the
 * library detector to detect it with.
 *
 * Target frame: origin at the bottom left corner of the board, x pointing
 * right, y pointing up and z pointing out of the board (towards the viewer).
 * Corner ids are target specific, but `corners` and `detect()` always use the
 * same ids.
 */
struct SimTarget {
  std::string name;
  double width = 0.0;  // [m]
  double height = 0.0; // [m]

  /** Ground truth corners (id -> p_T) in the target frame */
  std::map<int, Vec3> corners;

  /** Render board texture at `ppm` pixels per meter, without margin */
  std::function<cv::Mat(const double ppm)> render_board;

  /** Detect corners in a grayscale image (id -> pixel) */
  std::function<std::map<int, Vec2>(const cv::Mat &)> detect;

  /**
   * Render board texture (8-bit grayscale) at `ppm` pixels per meter, with a
   * white margin [m] around the board
   */
  cv::Mat render(const double margin, const double ppm) const;
};

/** Default configs of the simulated boards */
AprilGridConfig sim_aprilgrid_config();
CheckerboardConfig sim_checkerboard_config();
CharucoConfig sim_charuco_config();

/**
 * AprilGrid with (kalibr style) or without black squares between tags.
 * `detector_type` is "kaess" or "olsen", the two detectors use different tag textures.
 */
SimTarget make_sim_aprilgrid(const bool kalibr_style,
                             const std::string &detector_type = "kaess",
                             const bool refine_corners = false);

/** Checkerboard of 9x8 squares (an odd x even number avoids the 180 degree
 * orientation ambiguity) */
SimTarget make_sim_checkerboard(const bool refine_corners = false);

/** ChArUco board of 9x8 squares, with 5x5 markers (7 modules with border) */
SimTarget make_sim_charuco(const bool refine_corners = false);

} // namespace cartesian
