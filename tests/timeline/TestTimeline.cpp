#include <gtest/gtest.h>

#include "timeline/Timeline.hpp"

namespace cartesian {

TEST(Timeline, timeline_event) {
  const std::string event_type = "event_t";
  const timestamp_t event_ts = 0;
  TimelineEvent event(event_type, event_ts);
}

TEST(Timeline, CameraEvent) {
  const timestamp_t ts = 0;
  const int camera_id = 0;
  const std::string image_path = "";
  CameraEvent event(ts, camera_id, image_path);
}

TEST(Timeline, ImuEvent) {
  const timestamp_t ts = 0;
  const Vec3 acc{0.0, 0.0, 0.0};
  const Vec3 gyr{0.0, 0.0, 0.0};
  ImuEvent event(ts, acc, gyr);
}

TEST(Timeline, CalibTargetEvent) {
  const timestamp_t ts = 0;
  const int camera_id = 0;
  const int target_id = 0;
  const std::shared_ptr<CalibTarget> calib_target;
  CalibTargetEvent event(ts, camera_id, target_id, calib_target);
}

} // namespace cartesian
