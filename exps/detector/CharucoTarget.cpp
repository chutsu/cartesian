#include "CharucoTarget.hpp"

namespace cartesian {

CharucoTarget::CharucoTarget()
    : board{cv::Size{cols, rows},
            static_cast<float>(square_size),
            static_cast<float>(marker_size),
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250)},
      detector{board} {}

std::string CharucoTarget::name() const { return "charuco"; }

double CharucoTarget::width() const { return cols * square_size; }

double CharucoTarget::height() const { return rows * square_size; }

std::map<int, Vec3> CharucoTarget::corners() const {
  // Corner ids are row major, starting at the top left inner corner
  std::map<int, Vec3> corners;
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      const double x = (c + 1) * square_size;
      const double y = height() - (r + 1) * square_size;
      corners[r * (cols - 1) + c] = Vec3{x, y, 0.0};
    }
  }
  return corners;
}

std::map<int, Vec2> CharucoTarget::detect(const cv::Mat &image) {
  std::vector<cv::Point2f> points;
  std::vector<int> ids;
  detector.detectBoard(image, points, ids);

  std::map<int, Vec2> detected;
  for (size_t i = 0; i < ids.size(); ++i) {
    detected[ids[i]] = Vec2{points[i].x, points[i].y};
  }
  return detected;
}

cv::Mat CharucoTarget::render_board() const {
  cv::Mat image;
  board.generateImage(cv::Size{meters2px(width()), meters2px(height())},
                      image,
                      0, // Margin
                      1); // Marker border bits
  return image;
}

} // namespace cartesian
