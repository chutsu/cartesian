#include "GridDetector.hpp"

namespace cartesian {

void GridDetector::detect(const cv::Mat &image,
                          std::vector<cv::KeyPoint> &kps_new,
                          const std::vector<cv::KeyPoint> &kps_prev) {
  // Asserts
  assert(image.channels() == 1);

  // Calculate number of grid cells and max corners per cell
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  const int dx = int(std::ceil(float(img_w) / float(grid_cols)));
  const int dy = int(std::ceil(float(img_h) / float(grid_rows)));
  const int num_cells = grid_rows * grid_cols;
  const int max_per_cell = floor(max_keypoints / num_cells);

  // Create feature grid of previous keypoints
  FeatureGrid grid{img_w, img_h};
  for (const auto &kp : kps_prev) {
    const int pixel_x = kp.pt.x;
    const int pixel_y = kp.pt.y;
    grid.add(pixel_x, pixel_y);
  }

  // Detect corners in each grid cell
  int cell_idx = 0;
  for (int y = 0; y < img_h; y += dy) {
    for (int x = 0; x < img_w; x += dx) {
      // Make sure roi width and height are not out of bounds
      const int w = (x + dx > img_w) ? img_w - x : dx;
      const int h = (y + dy > img_h) ? img_h - y : dy;

      // Offset keypoints
      const int vacancy = max_per_cell - grid.count(cell_idx);
      if (vacancy <= 0) {
        continue;
      }

      // Detect corners in grid cell
      cv::Rect roi(x, y, w, h);
      std::vector<cv::KeyPoint> kps_roi;

      detector->setMaxFeatures(vacancy);
      detector->setQualityLevel(quality_level);
      detector->setMinDistance(min_distance);
      detector->setBlockSize(block_size);
      detector->setHarrisDetector(use_harris);
      detector->setK(k);
      detector->detect(image(roi), kps_roi);

      std::vector<cv::Point2f> corners;
      for (const auto kp : kps_roi) {
        corners.emplace_back(kp.pt.x, kp.pt.y);
      }

      // cv::Size win_size{5, 5};
      // cv::Size mask{-1, -1};
      // cv::TermCriteria criteria{cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
      //                           40,
      //                           0.001};
      // cv::cornerSubPix(image(roi), corners, win_size, mask, criteria);

      for (size_t i = 0; i < corners.size(); i++) {
        const auto &corner = corners[i];
        kps_roi[i].pt.x = corner.x;
        kps_roi[i].pt.y = corner.y;
      }
      sort_keypoints(kps_roi);

      for (int i = 0; i < std::min((int) kps_roi.size(), vacancy); i++) {
        cv::KeyPoint kp = kps_roi[i];
        kp.pt.x += x;
        kp.pt.y += y;

        kp.pt.x = (kp.pt.x > 0) ? kp.pt.x : 0;
        kp.pt.x = (kp.pt.x < image.cols) ? kp.pt.x : image.cols - 1;
        kp.pt.y = (kp.pt.y > 0) ? kp.pt.y : 0;
        kp.pt.y = (kp.pt.y < image.rows) ? kp.pt.y : image.rows - 1;

        kps_new.push_back(kp);
        grid.add(kp.pt.x, kp.pt.y);
      }

      // Update cell_idx
      cell_idx += 1;
    }
  }
  kps_new = spread_keypoints(image, kps_new, min_distance, kps_prev);
}

cv::Mat GridDetector::debug(const cv::Mat &image,
                            const std::vector<cv::KeyPoint> &kps_new,
                            const std::vector<cv::KeyPoint> &kps_prev) const {
  // Form feature grid
  const int img_w = image.size().width;
  const int img_h = image.size().height;
  FeatureGrid grid{img_w, img_h};
  for (const auto &kp : kps_prev) {
    const int pixel_x = kp.pt.x;
    const int pixel_y = kp.pt.y;
    grid.add(pixel_x, pixel_y);
  }

  // Visualization properties
  const auto red = cv::Scalar{0, 0, 255};
  const auto yellow = cv::Scalar{0, 255, 255};
  const auto line = cv::LINE_AA;
  const auto font = cv::FONT_HERSHEY_SIMPLEX;

  // Setup
  const int dx = int(std::ceil(float(img_w) / float(grid_cols)));
  const int dy = int(std::ceil(float(img_h) / float(grid_rows)));
  cv::Mat viz;
  cv::cvtColor(image, viz, cv::COLOR_GRAY2RGB);

  // Draw horizontal lines
  for (int x = 0; x < img_w; x += dx) {
    cv::line(viz, cv::Point2f(x, 0), cv::Point2f(x, img_w), red, 1, line);
  }

  // Draw vertical lines
  for (int y = 0; y < img_h; y += dy) {
    cv::line(viz, cv::Point2f(0, y), cv::Point2f(img_w, y), red, 1, line);
  }

  // Draw bin numbers
  int cell_idx = 0;
  for (int y = 0; y < img_h; y += dy) {
    for (int x = 0; x < img_w; x += dx) {
      auto text = std::to_string(grid.count(cell_idx));
      auto origin = cv::Point2f{x + 10.0f, y + 20.0f};
      cv::putText(viz, text, origin, font, 0.5, red, 1, line);
      cell_idx += 1;
    }
  }

  // Draw keypoints
  cv::drawKeypoints(viz, kps_new, viz, red);
  cv::drawKeypoints(viz, kps_prev, viz, yellow);

  return viz;
}

} // namespace cartesian
