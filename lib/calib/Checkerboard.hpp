#pragma once
#include <map>

#include "../core/Core.hpp"
#include "CornerTarget.hpp"

namespace cartesian {

/**
 * Checkerboard config
 *
 * `rows` and `cols` are the number of squares, so the board has
 * (rows - 1) x (cols - 1) inner corners. Use an odd number of squares in one
 * direction and an even number in the other, otherwise the orientation of the
 * board is ambiguous.
 */
struct CheckerboardConfig {
  int target_id = 0;
  int rows = 0;
  int cols = 0;
  double square_size = 0.0; // [m]

  /** Get number of inner corners in the horizontal and vertical direction */
  int get_inner_cols() const;
  int get_inner_rows() const;

  /** Get number of inner corners */
  int get_num_corners() const;

  /** Get board width and height [m] */
  double get_width() const;
  double get_height() const;

  /**
   * Get object points (corner id -> p_T) of the inner corners
   *
   * Corner ids are row major in the order OpenCV's findChessboardCornersSB()
   * returns them. The target frame has its origin at the first corner, with x
   * along the first row and y along the first column, and z = 0.
   */
  std::map<int, Vec3> get_object_points() const;

  /**
   * Get the inner corners (corner id -> p_B) in the board frame of
   * the board frame (origin at the bottom left corner of the board, x right, y
   * up), with the ids of `findChessboardCornersSB()`: row major
   * starting at the bottom right corner, going left and then up (determined by
   * the colour of the squares).
   */
  std::map<int, Vec3> get_board_points() const;

};

/** Checkerboard */
struct Checkerboard : CornerTarget {
  CheckerboardConfig config;

  Checkerboard(const timestamp_t &ts,
               const int camera_id,
               const CheckerboardConfig &config);
  virtual ~Checkerboard() = default;

  /** Save Checkerboard */
  int save(const fs::path &save_path) const;

  /** Load Checkerboard */
  static std::shared_ptr<Checkerboard> load(const fs::path &data_path);

  /** Draw detected corners */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;
};

} // namespace cartesian
