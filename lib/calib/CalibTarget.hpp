#pragma once
#include <map>

#include "../core/Core.hpp"

namespace cartesian {

// Forward declaration
struct CalibTarget;
using CalibTargetPtr = std::shared_ptr<CalibTarget>;
using CalibTargetMap = std::map<int, CalibTargetPtr>;
using CalibTargetData = std::map<timestamp_t, CalibTargetMap>;

/**
 * Calibration Target
 *
 * A set of points, e.g. the corners of a checkerboard, a ChArUco board or the
 * tags of an AprilGrid. Every point has an id and a known position in the
 * target frame. The measurements are the points that were detected in an
 * image.
 */
struct CalibTarget {
  std::string target_type;
  timestamp_t ts = 0;
  int camera_id = 0;
  int target_id = 0;
  std::map<int, Vec3> points;    // All target points (id -> p_T)
  std::map<int, Vec2> keypoints; // Detected points (id -> pixel)

  CalibTarget(const std::string &target_type,
              const timestamp_t &timestamp,
              const int camera_id,
              const int target_id,
              const std::map<int, Vec3> &points);
  virtual ~CalibTarget() = default;

  /** Check if detected */
  bool detected() const;

  /** Get number detected */
  int get_num_detected() const;

  /** Get measurements: ids, pixels and target frame positions of the points */
  void get_measurements(std::vector<int> &point_ids,
                        Vec2s &keypoints,
                        Vec3s &object_points) const;

  /** Check if point `id` was detected */
  bool has(const int id) const;

  /** Add measurement of point `id` */
  void add(const int id, const Vec2 &kp);

  /** Remove measurement of point `id` */
  void remove(const int id);

  /** Save target */
  virtual int save(const fs::path &save_path) const = 0;

protected:
  /**
   * Save target to file, `config` are the lines (without the trailing newline)
   * describing the target configuration, e.g. "rows 8"
   */
  int save_corners(const fs::path &save_path,
                   const std::vector<std::string> &config) const;

  /**
   * Load target file. The target configuration lines are returned as
   * key -> value strings in `config`, and the detected corners in `corners`.
   */
  static int load(const fs::path &data_path,
                  timestamp_t &ts,
                  int &camera_id,
                  std::string &target_type,
                  int &target_id,
                  std::map<std::string, std::string> &config,
                  std::map<int, Vec2> &corners);
};

} // namespace cartesian
