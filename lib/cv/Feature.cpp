#include "Feature.hpp"

namespace cartesian {

void Feature::update(const timestamp_t ts,
                     const int cam_id,
                     const cv::KeyPoint &kp) {
  keypoints[ts][cam_id] = kp;
}

} // namespace cartesian
