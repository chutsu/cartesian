#include "CheckerboardTarget.hpp"

#include <opencv2/calib3d.hpp>

namespace cartesian {

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
  const cv::Size pattern{cols - 1, rows - 1};
  const int flags = cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY;
  std::vector<cv::Point2f> points;
  std::map<int, Vec2> detected;
  if (cv::findChessboardCornersSB(image, pattern, points, flags) == false) {
    return detected;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    detected[i] = Vec2{points[i].x, points[i].y};
  }
  return detected;
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
