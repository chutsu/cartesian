#include "CalibTarget.hpp"

#include <fstream>
#include <sstream>

namespace cartesian {

CalibTarget::CalibTarget(const std::string &target_type,
                         const timestamp_t &ts,
                         const int camera_id,
                         const int target_id,
                         const std::map<int, Vec3> &points)
    : target_type{target_type}, ts{ts}, camera_id{camera_id},
      target_id{target_id}, points{points} {}

bool CalibTarget::detected() const { return keypoints.size() > 0; }

int CalibTarget::get_num_detected() const { return keypoints.size(); }

void CalibTarget::get_measurements(std::vector<int> &point_ids,
                                    Vec2s &kps,
                                    Vec3s &object_points) const {
  for (const auto &[id, kp] : keypoints) {
    point_ids.push_back(id);
    kps.push_back(kp);
    object_points.push_back(points.at(id));
  }
}

bool CalibTarget::has(const int id) const { return keypoints.count(id) > 0; }

void CalibTarget::add(const int id, const Vec2 &kp) {
  if (points.count(id) == 0) {
    FATAL("Corner id [%d] is not on target [%d]!", id, target_id);
  }
  keypoints[id] = kp;
}

void CalibTarget::remove(const int id) { keypoints.erase(id); }

int CalibTarget::save_corners(const fs::path &save_path,
                               const std::vector<std::string> &config) const {
  // Check save dir
  const auto dir_path = save_path.parent_path();
  if (!fs::exists(dir_path) && fs::create_directories(dir_path)) {
    LOG_ERROR("Could not create dir [%s]!", dir_path.c_str());
    return -1;
  }

  // Open file for saving
  std::ofstream file{save_path};
  if (file.is_open() == false) {
    LOG_ERROR("Failed to open [%s] for saving!", save_path.c_str());
    return -1;
  }
  file.precision(17);

  // Output header
  file << "timestamp " << ts << "\n";
  file << "camera_id " << camera_id << "\n";
  file << "target_type " << target_type << "\n";
  file << "target_id " << target_id << "\n";
  for (const auto &line : config) {
    file << line << "\n";
  }
  file << "corners_detected " << keypoints.size() << "\n";
  file << "\n";
  file << "# point_id kp_x kp_y pt_x pt_y pt_z\n";
  for (const auto &[id, kp] : keypoints) {
    const Vec3 &pt = points.at(id);
    file << id << " " << kp.x() << " " << kp.y() << " " << pt.x() << " "
         << pt.y() << " " << pt.z() << "\n";
  }

  return 0;
}

int CalibTarget::load(const fs::path &data_path,
                       timestamp_t &ts,
                       int &camera_id,
                       std::string &target_type,
                       int &target_id,
                       std::map<std::string, std::string> &config,
                       std::map<int, Vec2> &corners) {
  std::ifstream file{data_path};
  if (file.is_open() == false) {
    LOG_ERROR("Failed to open [%s]!", data_path.c_str());
    return -1;
  }

  // Parse "key value" lines until the blank line, then the corner lines
  std::string line;
  bool header = true;
  while (std::getline(file, line)) {
    if (line.empty()) {
      header = false;
      continue;
    }
    if (line[0] == '#') {
      continue;
    }

    std::istringstream ss{line};
    if (header) {
      std::string key;
      std::string value;
      ss >> key >> value;
      config[key] = value;

    } else {
      int id = 0;
      double kp_x = 0.0;
      double kp_y = 0.0;
      if (!(ss >> id >> kp_x >> kp_y)) {
        LOG_ERROR("Failed to parse line [%s] in [%s]!",
                  line.c_str(),
                  data_path.c_str());
        return -1;
      }
      corners[id] = Vec2{kp_x, kp_y};
    }
  }

  // Pop the generic fields, what remains is the target configuration
  auto pop = [&](const std::string &key, std::string &value) {
    if (config.count(key) == 0) {
      return false;
    }
    value = config.at(key);
    config.erase(key);
    return true;
  };
  std::string ts_str, camera_id_str, target_id_str, num_str;
  if (!pop("timestamp", ts_str) || !pop("camera_id", camera_id_str) ||
      !pop("target_type", target_type) || !pop("target_id", target_id_str) ||
      !pop("corners_detected", num_str)) {
    LOG_ERROR("Incomplete header in [%s]!", data_path.c_str());
    return -1;
  }
  ts = std::stoll(ts_str);
  camera_id = std::stoi(camera_id_str);
  target_id = std::stoi(target_id_str);
  if (static_cast<size_t>(std::stoi(num_str)) != corners.size()) {
    LOG_ERROR("Expected %s corners but found %zu in [%s]!",
              num_str.c_str(),
              corners.size(),
              data_path.c_str());
    return -1;
  }

  return 0;
}

} // namespace cartesian
