#include <gtest/gtest.h>

#include "Sfm.hpp"
#include "GridDetector.hpp"

namespace cartesian {

/** Load image paths **/
static std::vector<fs::path> load_image_paths(const fs::path &data_dir) {
  // Get image paths
  std::vector<fs::path> image_paths;
  for (const auto &entry : fs::directory_iterator(data_dir)) {
    image_paths.emplace_back(entry.path());
  }

  // Sort by filename
  std::sort(image_paths.begin(),
            image_paths.end(),
            [](const fs::path &a, const fs::path &b) {
              return a.filename() < b.filename();
            });

  return image_paths;
}

static cv::Mat viz_optflow(const cv::Mat &image_km1,
                           const cv::Mat &image_k,
                           const std::vector<cv::KeyPoint> &kps_km1,
                           const std::vector<cv::KeyPoint> &kps_k,
                           const std::vector<uchar> &inliers) {
  // Draw properties
  const int radius = 2;
  const cv::Scalar red{0, 0, 255};

  // Setup
  cv::Mat viz_track;
  const cv::Mat img_i = image_km1;
  const cv::Mat img_j = image_k;
  const auto kps_i = kps_km1;
  const auto kps_j = kps_k;
  cv::Mat viz_i;
  cv::Mat viz_j;
  cv::cvtColor(img_i, viz_i, cv::COLOR_GRAY2RGB);
  cv::cvtColor(img_j, viz_j, cv::COLOR_GRAY2RGB);

  // Draw KeyPoints
  for (size_t n = 0; n < kps_i.size(); n++) {
    if (inliers[n] == 0) {
      continue;
    }

    auto pt_i = kps_i[n].pt;
    auto pt_j = kps_j[n].pt;
    cv::circle(viz_i, pt_i, radius, red, -1);
    cv::circle(viz_j, pt_j, radius, red, -1);
  }

  // Stitch images horizontally
  cv::hconcat(viz_i, viz_j, viz_track);

  // Draw lines
  for (size_t n = 0; n < kps_i.size(); n++) {
    auto pt_i = kps_i[n].pt;
    auto pt_j = kps_j[n].pt;
    pt_j.x += viz_i.cols;
    const auto line = cv::LINE_AA;
    cv::line(viz_track, pt_i, pt_j, red, 1, line);
  }

  return viz_track;
}

TEST(Sfm, test) {
  // Load data
  const auto data_dir = fs::path("/data/euroc/imu_april/mav0/cam0/data");
  const auto image_paths = load_image_paths(data_dir);

  // Feature detector
  cv::Mat image_km1;
  std::vector<cv::KeyPoint> kps_km1;
  std::vector<cv::KeyPoint> kps_k;
  GridDetector detector;

  size_t frame_id = 0;
  int wait = 0;
  for (const auto &image_path : image_paths) {
    // Load image
    const auto image_k = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

    // Initialize keypoints
    if (frame_id == 0) {
      std::vector<cv::KeyPoint> kps_new;
      detector.detect(image_k, kps_new, kps_km1);

      kps_km1 = kps_new;
      image_km1 = image_k;
      frame_id++;
      continue;
    }

    // Track
    cv::Mat viz_track;
    if (frame_id > 0) {
      std::vector<uchar> inliers;
      optflow_track(image_km1, image_k, kps_km1, kps_k, inliers);
      filter_outliers(kps_km1, kps_k, inliers);
      viz_track = viz_optflow(image_km1, image_k, kps_km1, kps_k, inliers);
    }

    // Update
    kps_km1 = kps_k;
    image_km1 = image_k;
    frame_id++;

    // Visualize
    cv::Mat viz = image_k;
    // cv::hconcat(image_color, image_debug, viz);
    cv::imshow("Image", viz_track);
    auto ch = cv::waitKey(wait);
    if (ch == 'q') {
      break;
    } else if (ch == 'p') {
      wait ^= 1;
    }
  }

  // cv::waitKey(0);
}

} // namespace cartesian
