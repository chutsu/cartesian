#include "SfmTracker.hpp"
#include "Ransac.hpp"

namespace cartesian {

void SfmTracker::track_features(const timestamp_t ts,
                                const cv::Mat &image,
                                const CameraGeometry &camera_geom,
                                const int max_features) {
  detector.max_keypoints = max_features;

  // Detect new features and start tracks with them
  auto detect_new_features = [&]() {
    std::vector<cv::KeyPoint> kps_prev;
    for (const auto track_id : tracking) {
      const auto &track = tracks.at(track_id);
      kps_prev.push_back(track.keypoints.rbegin()->second);
    }

    std::vector<cv::KeyPoint> kps_new;
    detector.detect(image, kps_new, kps_prev);
    for (const auto &kp : kps_new) {
      SfmTrack track;
      track.camera_id = camera_geom.camera_id;
      track.track_id = next_track_id++;
      track.keypoints[ts] = kp;
      tracks[track.track_id] = track;
      tracking.insert(track.track_id);
    }
  };

  // First image
  if (image_km1.empty()) {
    detect_new_features();
    image_km1 = image.clone();
    return;
  }

  // Track the features from the previous image, with optical flow
  std::vector<size_t> track_ids;
  std::vector<cv::KeyPoint> kps_km1;
  for (const auto track_id : tracking) {
    track_ids.push_back(track_id);
    kps_km1.push_back(tracks.at(track_id).keypoints.rbegin()->second);
  }

  if (track_ids.empty() == false) {
    std::vector<uchar> inliers;
    std::vector<cv::KeyPoint> kps_k;
    optflow_track(image_km1, image, kps_km1, kps_k, inliers);

    // Reject outliers with RANSAC
    std::vector<size_t> tracked;
    std::vector<cv::KeyPoint> kps_i;
    std::vector<cv::KeyPoint> kps_j;
    for (size_t i = 0; i < inliers.size(); i++) {
      if (inliers[i]) {
        tracked.push_back(i);
        kps_i.push_back(kps_km1[i]);
        kps_j.push_back(kps_k[i]);
      }
    }
    std::vector<uchar> keep(tracked.size(), 1);
    if (tracked.size() >= 8) {
      std::vector<uchar> ransac_inliers;
      ransac(camera_geom, kps_i, kps_j, ransac_inliers);
      if (ransac_inliers.size() == keep.size()) {
        keep = ransac_inliers;
      }
    }
    std::vector<bool> ok(track_ids.size(), false);
    for (size_t j = 0; j < tracked.size(); j++) {
      ok[tracked[j]] = keep[j] != 0;
    }

    for (size_t i = 0; i < track_ids.size(); i++) {
      if (ok[i]) {
        tracks.at(track_ids[i]).keypoints[ts] = kps_k[i];
      } else {
        tracking.erase(track_ids[i]);
      }
    }
  }

  // Replace the lost features
  if (tracking.size() < (max_features * 0.7)) {
    detect_new_features();
  }
  image_km1 = image.clone();
}

} // namespace cartesian
