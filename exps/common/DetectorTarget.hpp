#pragma once
#include <map>
#include <string>

#include "core/Core.hpp"

namespace cartesian {

/**
 * Calibration target used in the detector experiment.
 *
 * Target frame: origin at the bottom left corner of the board, x pointing
 * right, y pointing up and z pointing out of the board (towards the viewer).
 * Corner ids are target specific, but a target's `corners()` and `detect()`
 * always use the same ids.
 */
struct DetectorTarget {
  /** Texture resolution [pixels per meter] */
  static constexpr double PPM = 1000.0;

  virtual ~DetectorTarget() = default;

  /** Return target name */
  virtual std::string name() const = 0;

  /** Return board width [m] */
  virtual double width() const = 0;

  /** Return board height [m] */
  virtual double height() const = 0;

  /** Return ground truth corners (id -> p_T) in the target frame */
  virtual std::map<int, Vec3> corners() const = 0;

  /** Detect corners in a grayscale image (id -> pixel) */
  virtual std::map<int, Vec2> detect(const cv::Mat &image) = 0;

  /** Render board texture at `PPM` without margin (8-bit grayscale) */
  virtual cv::Mat render_board() const = 0;

  /** Convert meters to whole number of texture pixels (fatal if not whole) */
  static int meters2px(const double meters);

  /** Render board texture with a white margin [m] around the board */
  cv::Mat render(const double margin) const;
};

} // namespace cartesian
