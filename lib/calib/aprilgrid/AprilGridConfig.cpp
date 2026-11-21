#include "AprilGridConfig.hpp"

namespace cartesian {

double AprilGridConfig::get_width() const {
  auto w = (tag_cols * tag_size) + ((tag_cols - 1) * tag_size * tag_spacing);
  return w;
}

double AprilGridConfig::get_height() const {
  auto h = (tag_rows * tag_size) + ((tag_rows - 1) * tag_size * tag_spacing);
  return h;
}

Vec2 AprilGridConfig::get_width_height() const {
  return Vec2{get_width(), get_height()};
}

Vec2 AprilGridConfig::get_center() const { return get_width_height() / 2.0; }

Mat4 AprilGridConfig::get_center_relative_pose() const {
  const double calib_width = get_width();
  const double calib_height = get_height();
  const Mat3 C_TO = eye(3);
  const Vec3 r_TO{calib_width / 2.0, calib_height / 2.0, 1.0};
  const Mat4 T_TO = tf(C_TO, r_TO);
  return T_TO;
}

int AprilGridConfig::get_num_tags() const { return tag_rows * tag_cols; }

void AprilGridConfig::get_grid_index(const int tag_id, int &i, int &j) const {
  if ((tag_id - tag_id_offset) > (tag_rows * tag_cols)) {
    FATAL("tag_id > (tag_rows * tag_cols)!");
  } else if (tag_id < 0) {
    FATAL("tag_id < 0!");
  }

  i = int((tag_id - tag_id_offset) / tag_cols);
  j = int((tag_id - tag_id_offset) % tag_cols);
}

Vec3 AprilGridConfig::get_object_point(const int tag_id,
                                     const int corner_index) const {
  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  get_grid_index(tag_id, i, j);

  // Calculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const double x = j * (tag_size + tag_size * tag_spacing);
  const double y = i * (tag_size + tag_size * tag_spacing);

  // Calculate the x and y of each corner
  Vec3 object_point;
  switch (corner_index) {
    case 0: // Bottom left
      object_point = Vec3(x, y, 0);
      break;
    case 1: // Bottom right
      object_point = Vec3(x + tag_size, y, 0);
      break;
    case 2: // Top right
      object_point = Vec3(x + tag_size, y + tag_size, 0);
      break;
    case 3: // Top left
      object_point = Vec3(x, y + tag_size, 0);
      break;
    default:
      FATAL("Incorrect corner id [%d]!", corner_index);
      break;
  }

  return object_point;
}

std::map<int, Vec3> AprilGridConfig::get_object_points() const {
  std::map<int, Vec3> object_points;

  // Point ids start at 0, tag ids at the tag id offset
  for (int i = 0; i < (tag_rows * tag_cols); ++i) {
    for (int corner_index = 0; corner_index < 4; ++corner_index) {
      const int point_id = i * 4 + corner_index;
      object_points[point_id] =
          get_object_point(i + tag_id_offset, corner_index);
    }
  }

  return object_points;
}

} // namespace cartesian
