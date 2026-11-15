#include "CheckerboardTarget.hpp"

namespace cartesian {

static CheckerboardConfig make_config(const int rows,
                                      const int cols,
                                      const double square_size) {
  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = rows;
  config.cols = cols;
  config.square_size = square_size;
  return config;
}

CheckerboardTarget::CheckerboardTarget(const bool refine_corners)
    : config{make_config(rows, cols, square_size)}, detector{config,
                                                             refine_corners} {}

std::string CheckerboardTarget::name() const { return "checkerboard"; }

double CheckerboardTarget::width() const { return cols * square_size; }

double CheckerboardTarget::height() const { return rows * square_size; }

std::map<int, Vec3> CheckerboardTarget::corners() const {
  // OpenCV's findChessboardCornersSB() orders the inner corners row major
  // starting at the bottom right corner, going left and then up. The ordering
  // is determined by the colour of the squares.
  std::map<int, Vec3> corners;
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      const double x = width() - (c + 1) * square_size;
      const double y = (r + 1) * square_size;
      corners[r * (cols - 1) + c] = Vec3{x, y, 0.0};
    }
  }
  return corners;
}

std::map<int, Vec2> CheckerboardTarget::detect(const cv::Mat &image) {
  // The library detector returns the corners in the order of OpenCV's
  // findChessboardCornersSB(), the ids of `corners()`
  const auto target = detector.detect(0, 0, image);
  return target->keypoints;
}

cv::Mat CheckerboardTarget::render_board() const {
  const int sq = meters2px(square_size);
  cv::Mat board(rows * sq, cols * sq, CV_8UC1, cv::Scalar{255});
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if ((r + c) % 2 == 0) {
        cv::rectangle(board,
                      cv::Rect{c * sq, r * sq, sq, sq},
                      cv::Scalar{0},
                      cv::FILLED);
      }
    }
  }
  return board;
}

} // namespace cartesian
