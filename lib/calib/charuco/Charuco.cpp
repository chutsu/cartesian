#include "Charuco.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

namespace cartesian {

/******************************************************************************
 * CHARUCO
 *****************************************************************************/

Charuco::Charuco(const timestamp_t &ts,
                 const int camera_id,
                 const CharucoConfig &config_)
    : CalibTarget{"charuco",
                   ts,
                   camera_id,
                   config_.target_id,
                   config_.get_object_points()},
      config{config_} {}

int Charuco::save(const fs::path &save_path) const {
  return CalibTarget::save_corners(save_path,
                            {"rows " + std::to_string(config.rows),
                             "cols " + std::to_string(config.cols),
                             "square_size " + std::to_string(config.square_size),
                             "marker_size " + std::to_string(config.marker_size),
                             "dictionary " + config.dictionary});
}

std::shared_ptr<Charuco> Charuco::load(const fs::path &data_path) {
  timestamp_t ts = 0;
  int camera_id = 0;
  std::string target_type;
  CharucoConfig config;
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
  if (target_type != "charuco") {
    FATAL("[%s] is not a charuco board!", data_path.c_str());
  }
  config.rows = std::stoi(fields.at("rows"));
  config.cols = std::stoi(fields.at("cols"));
  config.square_size = std::stod(fields.at("square_size"));
  config.marker_size = std::stod(fields.at("marker_size"));
  config.dictionary = fields.at("dictionary");

  auto target = std::make_shared<Charuco>(ts, camera_id, config);
  for (const auto &[id, kp] : corners) {
    target->add(id, kp);
  }
  return target;
}

cv::Mat Charuco::draw(const cv::Mat &image,
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
