#include "Checkerboard.hpp"

#include <opencv2/imgproc.hpp>

namespace cartesian {

/******************************************************************************
 * CONFIG
 *****************************************************************************/

int CheckerboardConfig::get_inner_cols() const { return cols - 1; }

int CheckerboardConfig::get_inner_rows() const { return rows - 1; }

int CheckerboardConfig::get_num_corners() const {
  return get_inner_cols() * get_inner_rows();
}

double CheckerboardConfig::get_width() const { return cols * square_size; }

double CheckerboardConfig::get_height() const { return rows * square_size; }

std::map<int, Vec3> CheckerboardConfig::get_board_points() const {
  std::map<int, Vec3> points;
  for (int r = 0; r < get_inner_rows(); ++r) {
    for (int c = 0; c < get_inner_cols(); ++c) {
      const double x = get_width() - (c + 1) * square_size;
      const double y = (r + 1) * square_size;
      points[r * get_inner_cols() + c] = Vec3{x, y, 0.0};
    }
  }
  return points;
}

std::map<int, Vec3> CheckerboardConfig::get_object_points() const {
  std::map<int, Vec3> points;
  for (int r = 0; r < get_inner_rows(); ++r) {
    for (int c = 0; c < get_inner_cols(); ++c) {
      points[r * get_inner_cols() + c] =
          Vec3{c * square_size, r * square_size, 0.0};
    }
  }
  return points;
}

/******************************************************************************
 * CHECKERBOARD
 *****************************************************************************/

Checkerboard::Checkerboard(const timestamp_t &ts,
                           const int camera_id,
                           const CheckerboardConfig &config_)
    : CalibTarget{"checkerboard",
                   ts,
                   camera_id,
                   config_.target_id,
                   config_.get_object_points()},
      config{config_} {}

int Checkerboard::save(const fs::path &save_path) const {
  return CalibTarget::save_corners(save_path,
                            {"rows " + std::to_string(config.rows),
                             "cols " + std::to_string(config.cols),
                             "square_size " + std::to_string(config.square_size)});
}

std::shared_ptr<Checkerboard> Checkerboard::load(const fs::path &data_path) {
  timestamp_t ts = 0;
  int camera_id = 0;
  std::string target_type;
  CheckerboardConfig config;
  std::map<std::string, std::string> fields;
  std::map<int, Vec2> corners;
  if (CalibTarget::load(data_path,
                         ts,
                         camera_id,
                         target_type,
                         config.target_id,
                         fields,
                         corners) != 0) {
    FATAL("Failed to load [%s]!", data_path.c_str());
  }
  if (target_type != "checkerboard") {
    FATAL("[%s] is not a checkerboard!", data_path.c_str());
  }
  config.rows = std::stoi(fields.at("rows"));
  config.cols = std::stoi(fields.at("cols"));
  config.square_size = std::stod(fields.at("square_size"));

  auto target = std::make_shared<Checkerboard>(ts, camera_id, config);
  for (const auto &[id, kp] : corners) {
    target->add(id, kp);
  }
  return target;
}

cv::Mat Checkerboard::draw(const cv::Mat &image,
                           const int marker_size,
                           const cv::Scalar &color) const {
  cv::Mat bgr;
  if (image.channels() == 1) {
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  } else {
    bgr = image.clone();
  }

  for (const auto &[id, kp] : keypoints) {
    cv::circle(bgr, cv::Point2f(kp.x(), kp.y()), marker_size, color, 1);
  }
  return bgr;
}

} // namespace cartesian
