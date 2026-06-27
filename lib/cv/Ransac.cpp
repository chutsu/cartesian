#include "Ransac.hpp"

namespace cartesian {

void ransac(const CameraGeometry &cam0_geom,
            const CameraGeometry &cam1_geom,
            const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            std::vector<uchar> &inliers,
            const double reproj_threshold,
            const double confidence) {
  assert(kps0.size() > 0 && kps1.size() > 0);
  assert(kps0.size() == kps1.size());

  std::vector<cv::Point2f> pts0 = kps2pts(kps0);
  std::vector<cv::Point2f> pts1 = kps2pts(kps1);
  for (size_t i = 0; i < pts0.size(); i++) {
    const Vec2 z0_in{pts0[i].x, pts0[i].y};
    const Vec2 z1_in{pts1[i].x, pts1[i].y};
    const Vec2 z0_out = cam0_geom.undistort(z0_in);
    const Vec2 z1_out = cam1_geom.undistort(z1_in);

    pts0[i].x = z0_out.x();
    pts0[i].y = z0_out.y();
    pts1[i].x = z1_out.x();
    pts1[i].y = z1_out.y();
  }

  const int method = cv::FM_RANSAC;
  cv::findFundamentalMat(pts0,
                         pts1,
                         method,
                         reproj_threshold,
                         confidence,
                         inliers);
}

void ransac(const CameraGeometry &cam_geom,
            const std::vector<cv::KeyPoint> &kps0,
            const std::vector<cv::KeyPoint> &kps1,
            std::vector<uchar> &inliers,
            const double reproj_threshold,
            const double confidence) {
  ransac(cam_geom, cam_geom, kps0, kps1, inliers, reproj_threshold, confidence);
}

} // namespace cartesian
