#pragma once
#include "../../core/Core.hpp"

namespace cartesian {

/** AprilGridConfig **/
struct AprilGridConfig {
  int target_id = 0;
  int tag_rows = 0;
  int tag_cols = 0;
  double tag_size = 0.0;
  double tag_spacing = 0.0;
  int tag_id_offset = 0;

  /** Get width */
  double get_width() const;

  /** Get height */
  double get_height() const;

  /** Get width and height */
  Vec2 get_width_height() const;

  /** Get center */
  Vec2 get_center() const;

  /** Get center relative pose T_target_center */
  Mat4 get_center_relative_pose() const;

  /** Get number of tags */
  int get_num_tags() const;

  /** Get grid index */
  void get_grid_index(const int tag_id, int &i, int &j) const;

  /** Get object point */
  Vec3 get_object_point(const int tag_id, const int corner_index) const;

  /** Get object points */
  std::map<int, Vec3> get_object_points() const;
};

} // namespace cartesian
