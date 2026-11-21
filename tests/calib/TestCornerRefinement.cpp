#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include "calib/CalibTargetDetector.hpp"

namespace cartesian {

/** Blurred checkerboard of 8x8 squares, the corner between squares at pixel
 * boundary 160 is at (159.5, 159.5), where the center of pixel (0, 0) is at
 * (0, 0) */
static cv::Mat make_checkerboard() {
  const int sq = 40;
  cv::Mat image(8 * sq, 8 * sq, CV_8UC1, cv::Scalar{255});
  for (int r = 0; r < 8; ++r) {
    for (int c = 0; c < 8; ++c) {
      if ((r + c) % 2 == 0) {
        cv::rectangle(image,
                      cv::Rect{c * sq, r * sq, sq, sq},
                      cv::Scalar{0},
                      cv::FILLED);
      }
    }
  }
  cv::GaussianBlur(image, image, cv::Size{}, 1.0);
  return image;
}

TEST(CornerRefinement, refine) {
  const auto image = make_checkerboard();
  const Vec2 truth{159.5, 159.5};

  std::vector<Vec2> corners{truth + Vec2{1.2, -0.9}};
  CalibTargetDetector::refine_corners_subpix(image, corners, 10, 3.0);
  ASSERT_LT((corners[0] - truth).norm(), 0.1);
}

TEST(CornerRefinement, refine_many_with_windows_per_corner) {
  const auto image = make_checkerboard();
  const std::vector<Vec2> truth{{119.5, 119.5}, {159.5, 199.5}, {239.5, 159.5}};

  std::vector<Vec2> corners;
  for (const auto &p : truth) {
    corners.push_back(p + Vec2{-0.7, 0.8});
  }
  CalibTargetDetector::refine_corners_subpix(image, corners, std::vector<int>{8, 10, 0}, 3.0);

  ASSERT_LT((corners[0] - truth[0]).norm(), 0.1);
  ASSERT_LT((corners[1] - truth[1]).norm(), 0.1);

  // A corner without a window is not refined
  ASSERT_NEAR((corners[2] - (truth[2] + Vec2{-0.7, 0.8})).norm(), 0.0, 1e-12);
}

TEST(CornerRefinement, keeps_corners_that_cannot_be_refined) {
  const auto image = make_checkerboard();

  // Search window outside of the image
  std::vector<Vec2> near_border{{3.0, 100.0}};
  CalibTargetDetector::refine_corners_subpix(image, near_border, 10, 3.0);
  ASSERT_NEAR((near_border[0] - Vec2{3.0, 100.0}).norm(), 0.0, 1e-12);

  // Refinement moves the corner by more than `max_shift`
  const Vec2 start = Vec2{159.5, 159.5} + Vec2{1.5, 1.5};
  std::vector<Vec2> too_far{start};
  CalibTargetDetector::refine_corners_subpix(image, too_far, 10, 0.5);
  ASSERT_NEAR((too_far[0] - start).norm(), 0.0, 1e-12);
}

} // namespace cartesian
