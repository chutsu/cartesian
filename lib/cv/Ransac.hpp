#pragma once
#include <vector>

#include "camera/CameraGeometry.hpp"

namespace cartesian {

/**
 * Perform RANSAC on keypoints `kps0` and `kps1` to reject outliers.
 */
void ransac(const CameraGeometry &cam0_geom,
            const CameraGeometry &cam1_geom,
            const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            std::vector<uchar> &inliers,
            const double reproj_threshold = 0.5,
            const double confidence = 0.99);

/**
 * Convenience overload for the monocular case (same undistortion for both
 * images).
 */
void ransac(const CameraGeometry &cam_geom,
            const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            std::vector<uchar> &inliers,
            const double reproj_threshold = 0.75,
            const double confidence = 0.99);

} // namespace cartesian
