#include "CornerRefinement.hpp"

#include <opencv2/imgproc.hpp>

namespace cartesian {

void refine_corners_subpix(const cv::Mat &image,
                           std::vector<Vec2> &corners,
                           const int half_window,
                           const double max_shift) {
  refine_corners_subpix(image,
                        corners,
                        std::vector<int>(corners.size(), half_window),
                        max_shift);
}

void refine_corners_subpix(const cv::Mat &image,
                           std::vector<Vec2> &corners,
                           const std::vector<int> &half_windows,
                           const double max_shift) {
  assert(image.channels() == 1);
  assert(corners.size() == half_windows.size());

  const auto criteria =
      cv::TermCriteria{cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                       30,
                       0.01};
  for (size_t i = 0; i < corners.size(); ++i) {
    // Skip corners with their search window not inside the image
    const int w = half_windows[i];
    const double margin = w + 2;
    const Vec2 &c = corners[i];
    if (w < 1 || c.x() < margin || c.y() < margin ||
        c.x() > image.cols - 1 - margin || c.y() > image.rows - 1 - margin) {
      continue;
    }

    // Refine, and accept if it stayed close to where it started
    std::vector<cv::Point2f> point{cv::Point2f(c.x(), c.y())};
    cv::cornerSubPix(image, point, cv::Size{w, w}, cv::Size{-1, -1}, criteria);
    const Vec2 refined{point[0].x, point[0].y};
    if (refined.allFinite() && (refined - c).norm() <= max_shift) {
      corners[i] = refined;
    }
  }
}

} // namespace cartesian
