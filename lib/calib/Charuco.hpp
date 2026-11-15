#pragma once
#include <map>
#include <string>

#include <opencv2/objdetect/charuco_detector.hpp>

#include "../core/Core.hpp"
#include "CornerTarget.hpp"

namespace cartesian {

/**
 * ChArUco board config
 *
 * `rows` and `cols` are the number of squares, so the board has
 * (rows - 1) x (cols - 1) inner corners, and `marker_size` is the side length
 * of the ArUco markers, which sit in the white squares. `dictionary` is the
 * name of an OpenCV predefined ArUco dictionary, e.g. "DICT_5X5_250".
 */
struct CharucoConfig {
  int target_id = 0;
  int rows = 0;
  int cols = 0;
  double square_size = 0.0; // [m]
  double marker_size = 0.0; // [m]
  std::string dictionary = "DICT_5X5_250";

  /** Get number of inner corners */
  int get_num_corners() const;

  /**
   * Get object points (corner id -> p_T) of the inner corners
   *
   * Corner ids and the target frame are those of OpenCV's CharucoBoard: ids
   * are row major starting at the top left inner corner, x is right and y is
   * down along the board, and z = 0.
   */
  std::map<int, Vec3> get_object_points() const;
};

/** Get the OpenCV ChArUco board of a config */
cv::aruco::CharucoBoard charuco_board(const CharucoConfig &config);

/** ChArUco board */
struct Charuco : CornerTarget {
  CharucoConfig config;

  Charuco(const timestamp_t &ts,
          const int camera_id,
          const CharucoConfig &config);
  virtual ~Charuco() = default;

  /** Save Charuco */
  int save(const fs::path &save_path) const;

  /** Load Charuco */
  static std::shared_ptr<Charuco> load(const fs::path &data_path);

  /** Draw detected corners */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;
};

} // namespace cartesian
