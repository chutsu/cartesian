#include "Charuco.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>

namespace cartesian {

/******************************************************************************
 * CONFIG
 *****************************************************************************/

cv::aruco::CharucoBoard charuco_board(const CharucoConfig &config) {
  static const std::map<std::string, cv::aruco::PredefinedDictionaryType>
      dictionaries{
          {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
          {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
          {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
          {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
          {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
          {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
          {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
          {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
          {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
          {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
          {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
          {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
          {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
          {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
          {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
          {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
          {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
          {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
          {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
          {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
          {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11},
      };
  if (dictionaries.count(config.dictionary) == 0) {
    FATAL("Unsupported ArUco dictionary [%s]!", config.dictionary.c_str());
  }

  return cv::aruco::CharucoBoard{
      cv::Size{config.cols, config.rows},
      static_cast<float>(config.square_size),
      static_cast<float>(config.marker_size),
      cv::aruco::getPredefinedDictionary(dictionaries.at(config.dictionary))};
}

int CharucoConfig::get_num_corners() const { return (rows - 1) * (cols - 1); }

double CharucoConfig::get_width() const { return cols * square_size; }

double CharucoConfig::get_height() const { return rows * square_size; }

std::map<int, Vec3> CharucoConfig::get_board_points() const {
  std::map<int, Vec3> points;
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      const double x = (c + 1) * square_size;
      const double y = get_height() - (r + 1) * square_size;
      points[r * (cols - 1) + c] = Vec3{x, y, 0.0};
    }
  }
  return points;
}

std::map<int, Vec3> CharucoConfig::get_object_points() const {
  const auto board = charuco_board(*this);
  const auto corners = board.getChessboardCorners();

  std::map<int, Vec3> points;
  for (size_t id = 0; id < corners.size(); ++id) {
    points[id] = Vec3{corners[id].x, corners[id].y, corners[id].z};
  }
  return points;
}

/******************************************************************************
 * CHARUCO
 *****************************************************************************/

Charuco::Charuco(const timestamp_t &ts,
                 const int camera_id,
                 const CharucoConfig &config_)
    : CornerTarget{"charuco",
                   ts,
                   camera_id,
                   config_.target_id,
                   config_.get_object_points()},
      config{config_} {}

int Charuco::save(const fs::path &save_path) const {
  return CornerTarget::save(save_path,
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
  if (CornerTarget::load(data_path,
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
