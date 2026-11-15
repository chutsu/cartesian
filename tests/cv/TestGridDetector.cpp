#include <gtest/gtest.h>

#include "cv/GridDetector.hpp"

namespace cartesian {

TEST(GridDetector, test) {
  const auto img_path = "./test_data/frontend/cam0/1403715297312143104.jpg";
  const auto img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

  std::vector<cv::KeyPoint> kps_prev;
  std::vector<cv::KeyPoint> kps_new;
  GridDetector detector;
  detector.detect(img, kps_new, kps_prev);

  // const auto image_debug = detector.debug(image, kps_new, kps_prev);
  // cv::waitKey(0);
}

} // namespace cartesian
