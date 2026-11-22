#pragma once
#include <map>

#include "camera/CameraGeometry.hpp"

namespace cartesian {

/** A feature tracked through the frames of one camera */
struct SfmTrack {
  int camera_id = 0;
  size_t track_id = 0;
  std::map<timestamp_t, cv::KeyPoint> keypoints; // Measurements (ts -> pixel)
  int point_id = -1;         // Id of the point, -1 if the track has none
  bool triangulated = false; // Point was triangulated, not just initialized

  /**
   * Triangulate the track from its first and last measurements, given the
   * camera poses T_WC at their times. Fails if there is not enough parallax
   * (`min_parallax_deg`), the point is behind a camera, or it reprojects more
   * than `max_reproj_error` [px] from the measurements.
   */
  bool triangulate(const CameraGeometry &camera_geom,
                   const Mat4 &T_WC_first,
                   const Mat4 &T_WC_last,
                   const double min_parallax_deg,
                   const double max_reproj_error,
                   Vec3 &p_W) const;
};

} // namespace cartesian
