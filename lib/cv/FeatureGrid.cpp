#include "FeatureGrid.hpp"

#include <opencv2/opencv.hpp>

namespace cartesian {

FeatureGrid::FeatureGrid(const int image_width,
                         const int image_height,
                         const int grid_rows,
                         const int grid_cols)
    : image_width{image_width},
      image_height{image_height}, grid_rows{grid_rows}, grid_cols{grid_cols} {
  for (int i = 0; i < (grid_rows * grid_cols); i++) {
    cells.push_back(0);
  }
}

void FeatureGrid::add(const int pixel_x, const int pixel_y) {
  assert(pixel_x >= 0 && pixel_x <= image_width);
  assert(pixel_y >= 0 && pixel_y <= image_height);
  const int cell_idx = cellIndex(pixel_x, pixel_y);
  cells[cell_idx] += 1;
  keypoints.emplace_back(pixel_x, pixel_y);
}

int FeatureGrid::cellIndex(const int pixel_x, const int pixel_y) const {
  const float img_w = image_width;
  const float img_h = image_height;
  float grid_x = std::ceil((std::max(1, pixel_x) / img_w) * grid_cols) - 1.0;
  float grid_y = std::ceil((std::max(1, pixel_y) / img_h) * grid_rows) - 1.0;
  const int cell_id = int(grid_x + (grid_y * grid_cols));
  return cell_id;
}

int FeatureGrid::count(const int cell_idx) const { return cells[cell_idx]; }

void FeatureGrid::debug(const bool imshow) const {
  const int w = image_width;
  const int h = image_height;
  cv::Mat viz = cv::Mat::zeros(h, w, CV_32F);

  for (const auto &[x, y]: keypoints) {
    const cv::Point p(x, y);
    const int radius = 2;
    const cv::Scalar color{255, 255, 255};
    cv::circle(viz, p, radius, color, -1);
  }

  if (imshow) {
    cv::imshow("FeatureGrid", viz);
    cv::waitKey(0);
  }
}

} // namespace cartesian
