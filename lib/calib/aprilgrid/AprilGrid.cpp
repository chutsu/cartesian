#include "AprilGrid.hpp"

namespace cartesian {

/******************************************************************************
 * CONSTRUCTORS
 *****************************************************************************/

AprilGrid::AprilGrid(const timestamp_t &ts,
                     const int camera_id,
                     const AprilGridConfig &config)
    : CalibTarget{"aprilgrid",
                  ts,
                  camera_id,
                  config.target_id,
                  config.get_object_points()},
      config_{config} {}

/******************************************************************************
 * CONFIG
 *****************************************************************************/

AprilGridConfig AprilGrid::get_config() const { return config_; }

int AprilGrid::get_tag_rows() const { return config_.tag_rows; }

int AprilGrid::get_tag_cols() const { return config_.tag_cols; }

double AprilGrid::get_tag_size() const { return config_.tag_size; }

double AprilGrid::get_tag_spacing() const { return config_.tag_spacing; }

int AprilGrid::get_tag_id_offset() const { return config_.tag_id_offset; }

/******************************************************************************
 * GEOMETRY
 *****************************************************************************/

Vec2 AprilGrid::get_width_height() const {
  const auto rows = get_tag_rows();
  const auto cols = get_tag_cols();
  const auto size = get_tag_size();
  const auto spacing = get_tag_spacing();
  const auto w = (cols * size) + ((cols - 1) * size * spacing);
  const auto h = (rows * size) + ((rows - 1) * size * spacing);
  return Vec2{w, h};
}

Vec2 AprilGrid::get_center() const { return get_width_height() / 2.0; }

void AprilGrid::get_grid_index(const int tag_id, int &i, int &j) const {
  const auto tag_rows = get_tag_rows();
  const auto tag_cols = get_tag_cols();
  const auto tag_id_offset = get_tag_id_offset();

  if ((tag_id - tag_id_offset) > (tag_rows * tag_cols)) {
    FATAL("tag_id > (tag_rows * tag_cols)!");
  } else if (tag_id < 0) {
    FATAL("tag_id < 0!");
  }

  i = int((tag_id - tag_id_offset) / tag_cols);
  j = int((tag_id - tag_id_offset) % tag_cols);
}

Vec3 AprilGrid::get_object_point(const int tag_id,
                                 const int corner_index) const {
  // Calculate the AprilGrid index using tag id
  int i = 0;
  int j = 0;
  get_grid_index(tag_id, i, j);

  // Calculate the x and y of the tag origin (bottom left corner of tag)
  // relative to grid origin (bottom left corner of entire grid)
  const auto tag_size = get_tag_size();
  const auto tag_spacing = get_tag_spacing();
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

Vec3 AprilGrid::get_object_point(const int point_id) const {
  return get_object_point(get_tag_id(point_id), get_corner_index(point_id));
}

int AprilGrid::get_point_id(const int tag_id, const int corner_index) const {
  return (tag_id - get_tag_id_offset()) * 4 + corner_index;
}

int AprilGrid::get_tag_id(const int point_id) const {
  return point_id / 4 + get_tag_id_offset();
}

int AprilGrid::get_corner_index(const int point_id) const {
  return point_id % 4;
}

Vec2 AprilGrid::get_center_2d() const {
  const auto tag_rows = get_tag_rows();
  const auto tag_cols = get_tag_cols();
  const auto tag_size = get_tag_size();
  const auto tag_spacing = get_tag_spacing();

  double x = ((tag_cols / 2.0) * tag_size);
  x += (((tag_cols / 2.0) - 1) * tag_spacing * tag_size);
  x += (0.5 * tag_spacing * tag_size);

  double y = ((tag_rows / 2.0) * tag_size);
  y += (((tag_rows / 2.0) - 1) * tag_spacing * tag_size);
  y += (0.5 * tag_spacing * tag_size);

  return Vec2{x, y};
}

Vec3 AprilGrid::get_center_3d() const {
  const Vec2 center = get_center_2d();
  return Vec3{center.x(), center.y(), 0.0};
}

/******************************************************************************
 * MEASUREMENTS
 *****************************************************************************/

bool AprilGrid::has(const int tag_id, const int corner_index) const {
  return has(get_point_id(tag_id, corner_index));
}

void AprilGrid::add(const int tag_id, const int corner_index, const Vec2 &kp) {
  const auto id_offset = get_tag_id_offset();
  const auto num_tags = get_tag_rows() * get_tag_cols();
  if ((tag_id - id_offset) < 0 || (tag_id - id_offset) >= num_tags) {
    return;
  }
  if (corner_index < 0 || corner_index > 3) {
    return;
  }

  add(get_point_id(tag_id, corner_index), kp);
}

void AprilGrid::remove(const int tag_id, const int corner_index) {
  remove(get_point_id(tag_id, corner_index));
}

void AprilGrid::remove_tag(const int tag_id) {
  for (int corner_index = 0; corner_index < 4; ++corner_index) {
    remove(tag_id, corner_index);
  }
}

/******************************************************************************
 * FILE I/O
 *****************************************************************************/

int AprilGrid::save(const fs::path &save_path) const {
  // Check save dir
  const auto dir_path = save_path.parent_path();
  if (!fs::exists(dir_path) && fs::create_directories(dir_path)) {
    LOG_ERROR("Could not create dir [%s]!", dir_path.c_str());
    return -1;
  }

  // Open file for saving
  const auto fp = fopen(save_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", save_path.c_str());
    return -1;
  }

  // Get measurement data
  std::vector<int> point_ids;
  Vec2s keypoints;
  Vec3s object_points;
  get_measurements(point_ids, keypoints, object_points);

  // Output header
  fprintf(fp, "timestamp %ld\n", ts);
  fprintf(fp, "camera_id %d\n", camera_id);
  fprintf(fp, "target_type %s\n", target_type.c_str());
  fprintf(fp, "target_id %d\n", target_id);
  fprintf(fp, "tag_rows %d\n", get_tag_rows());
  fprintf(fp, "tag_cols %d\n", get_tag_cols());
  fprintf(fp, "tag_size %f\n", get_tag_size());
  fprintf(fp, "tag_spacing %f\n", get_tag_spacing());
  fprintf(fp, "tag_id_offset %d\n", get_tag_id_offset());
  fprintf(fp, "corners_detected %ld\n", point_ids.size());
  fprintf(fp, "\n");
  fprintf(fp, "# tag_id corner_index kp_x kp_y pt_x pt_y pt_z\n");
  for (size_t i = 0; i < point_ids.size(); i++) {
    const int tag_id = get_tag_id(point_ids[i]);
    const int corner_index = get_corner_index(point_ids[i]);
    const Vec2 kp = keypoints[i];
    const Vec3 pt = object_points[i];

    fprintf(fp, "%d ", tag_id);
    fprintf(fp, "%d ", corner_index);
    fprintf(fp, "%lf ", kp.x());
    fprintf(fp, "%lf ", kp.y());
    fprintf(fp, "%lf ", pt.x());
    fprintf(fp, "%lf ", pt.y());
    fprintf(fp, "%lf\n", pt.z());
  }

  // Close up
  fclose(fp);
  return 0;
}

static void aprilgrid_parse_line(FILE *fp,
                                 const char *key,
                                 const char *value_type,
                                 void *value) {
  assert(fp != NULL);
  assert(key != NULL);
  assert(value_type != NULL);
  assert(value != NULL);

  // Parse line
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  if (fgets(buf, buf_len, fp) == NULL) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Split key-value
  char delim[2] = " ";
  char *key_str = strtok(buf, delim);
  char *value_str = strtok(NULL, delim);

  // Check key matches
  if (strcmp(key_str, key) != 0) {
    FATAL("Failed to parse [%s]\n", key);
  }

  // Typecase value
  if (value_type == NULL) {
    FATAL("Value type not set!\n");
  }

  if (strcmp(value_type, "uint64_t") == 0) {
    *(uint64_t *) value = atol(value_str);
  } else if (strcmp(value_type, "int") == 0) {
    *(int *) value = atoi(value_str);
  } else if (strcmp(value_type, "double") == 0) {
    *(double *) value = atof(value_str);
  } else if (strcmp(value_type, "string") == 0) {
    *(std::string *) value = std::string(value_str);
  } else {
    FATAL("Invalid value type [%s]\n", value_type);
  }
}

static void aprilgrid_parse_skip_line(FILE *fp) {
  assert(fp != NULL);
  const size_t buf_len = 1024;
  char buf[1024] = {0};
  char *retval = fgets(buf, buf_len, fp);
  UNUSED(retval);
}

std::shared_ptr<AprilGrid> AprilGrid::load(const fs::path &data_path) {
  // Open file for loading
  FILE *fp = fopen(data_path.c_str(), "r");
  if (fp == NULL) {
    FATAL("Failed to open [%s]!\n", data_path.c_str());
  }

  // Parse configuration
  timestamp_t ts = 0;
  int camera_id = 0;
  std::string target_type;
  AprilGridConfig config;
  int corners_detected = 0;
  aprilgrid_parse_line(fp, "timestamp", "uint64_t", &ts);
  aprilgrid_parse_line(fp, "camera_id", "int", &camera_id);
  aprilgrid_parse_line(fp, "target_type", "string", &target_type);
  aprilgrid_parse_line(fp, "target_id", "int", &config.target_id);
  aprilgrid_parse_line(fp, "tag_rows", "int", &config.tag_rows);
  aprilgrid_parse_line(fp, "tag_cols", "int", &config.tag_cols);
  aprilgrid_parse_line(fp, "tag_size", "double", &config.tag_size);
  aprilgrid_parse_line(fp, "tag_spacing", "double", &config.tag_spacing);
  aprilgrid_parse_line(fp, "tag_id_offset", "int", &config.tag_id_offset);
  aprilgrid_parse_line(fp, "corners_detected", "int", &corners_detected);
  aprilgrid_parse_skip_line(fp);
  aprilgrid_parse_skip_line(fp);
  auto grid = std::make_shared<AprilGrid>(ts, camera_id, config);

  // Parse data
  const char *scan_format = "%d %d %lf %lf %lf %lf %lf";
  for (int i = 0; i < corners_detected; i++) {
    // Parse data line
    int tag_id = 0;
    int corner_index = 0;
    double kp[2] = {0};
    double p[3] = {0};
    const int retval = fscanf(fp,
                              scan_format,
                              &tag_id,
                              &corner_index,
                              &kp[0],
                              &kp[1],
                              &p[0],
                              &p[1],
                              &p[2]);
    if (retval != 7) {
      FATAL("Failed to parse data line %d in [%s]\n", i, data_path.c_str());
    }
    grid->add(tag_id, corner_index, {kp[0], kp[1]});
  }

  // Clean up
  fclose(fp);

  return grid;
}

std::vector<std::shared_ptr<CalibTarget>>
AprilGrid::load_directory(const fs::path &dir_path) {
  std::vector<fs::path> csv_files;
  for (const auto &csv_file : fs::directory_iterator(dir_path)) {
    csv_files.push_back(csv_file);
  }
  sort(csv_files.begin(), csv_files.end());

  std::vector<std::shared_ptr<CalibTarget>> grids;
  for (const auto &grid_csv : csv_files) {
    const auto csv_path = dir_path / grid_csv;
    grids.push_back(AprilGrid::load(grid_csv));
  }

  return grids;
}

/******************************************************************************
 * VISUALIZATION
 *****************************************************************************/

cv::Mat AprilGrid::draw(const cv::Mat &image,
                        const int marker_size,
                        const cv::Scalar &color) const {
  const cv::Scalar text_color(0, 255, 0);
  const int font = cv::FONT_HERSHEY_PLAIN;
  const double font_scale = 1.0;
  const int thickness = 2;
  cv::Mat image_rgb = gray2rgb(image);

  std::vector<int> point_ids;
  Vec2s keypoints;
  Vec3s object_points;
  get_measurements(point_ids, keypoints, object_points);

  for (size_t i = 0; i < point_ids.size(); i++) {
    // Setup
    const auto point_id = point_ids[i];
    const auto kp = keypoints[i];

    // Draw corners
    cv::Point2f p(kp.x(), kp.y());
    cv::circle(image_rgb, p, marker_size, color, -1);

    // Label corner
    cv::Point2f cxy(kp.x(), kp.y());
    std::string text = std::to_string(point_id);
    cv::putText(image_rgb, text, cxy, font, font_scale, text_color, thickness);
  }

  return image_rgb;
}

void AprilGrid::imshow(const std::string &title, const cv::Mat &image) const {
  cv::imshow(title, draw(image));
  cv::waitKey(1);
}

} // namespace cartesian
