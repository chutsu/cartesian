#pragma once
#include <map>
#include <set>

#include "GridDetector.hpp"
#include "SfmTrack.hpp"

namespace cartesian {

/** Feature tracker of one camera */
struct SfmTracker {
  GridDetector detector;
  cv::Mat image_km1;
  std::map<size_t, SfmTrack> tracks;
  std::set<size_t> tracking; // Ids of the tracks that are being tracked
  size_t next_track_id = 0;

  /**
   * Track the features of the previous image into `image` taken at `ts`, with
   * optical flow and RANSAC to reject outliers, then detect new features to
   * start tracks when fewer than 70% of `max_features` are being tracked
   */
  void track_features(const timestamp_t ts,
                      const cv::Mat &image,
                      const CameraGeometry &camera_geom,
                      const int max_features);
};

} // namespace cartesian
