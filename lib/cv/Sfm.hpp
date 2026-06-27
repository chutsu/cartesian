#pragma once
#include <map>

#include "camera/CameraGeometry.hpp"

namespace cartesian {

struct Sfm {
  std::map<int, CameraGeometry> camera_geoms;
  std::map<size_t, Vec3> points;
  std::map<size_t, Vec7> poses;

  void add_camera();
  void add_point();
  void add_pose();
};

} // namespace cartesian
