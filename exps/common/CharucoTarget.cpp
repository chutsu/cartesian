#include "CharucoTarget.hpp"

namespace cartesian {

static CharucoConfig make_config(const int rows,
                                 const int cols,
                                 const double square_size,
                                 const double marker_size) {
  CharucoConfig config;
  config.target_id = 0;
  config.rows = rows;
  config.cols = cols;
  config.square_size = square_size;
  config.marker_size = marker_size;
  config.dictionary = "DICT_5X5_250";
  return config;
}

CharucoTarget::CharucoTarget(const bool refine_corners)
    : config{make_config(rows, cols, square_size, marker_size)},
      board{charuco_board(config)}, detector{config, refine_corners} {}

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
  const auto target = detector.detect(0, 0, image);
  return target->keypoints;
}

cv::Mat CharucoTarget::render_board() const {
  cv::Mat image;
  board.generateImage(cv::Size{meters2px(width()), meters2px(height())},
                      image,
                      0,  // Margin
                      1); // Marker border bits
  return image;
}

} // namespace cartesian
