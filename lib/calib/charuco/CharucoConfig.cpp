#include "CharucoConfig.hpp"

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

} // namespace cartesian
