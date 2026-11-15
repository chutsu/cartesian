#pragma once
#include <map>

#include "../core/Core.hpp"
#include "CalibTarget.hpp"

namespace cartesian {

/**
 * Calibration target whose measurements are individually identified corners,
 * e.g. a checkerboard or a ChArUco board.
 *
 * Every corner has an id and a known position in the target frame. The
 * measurements are the corners that were detected in an image.
 */
struct CornerTarget : CalibTarget {
  std::map<int, Vec3> points;    // All target corners (id -> p_T)
  std::map<int, Vec2> keypoints; // Detected corners (id -> pixel)

  CornerTarget(const std::string &target_type,
               const timestamp_t &ts,
               const int camera_id,
               const int target_id,
               const std::map<int, Vec3> &points);
  virtual ~CornerTarget() = default;

  /** Check if detected */
  bool detected() const override;

  /** Get number detected */
  int get_num_detected() const override;

  /** Get measurements */
  void get_measurements(std::vector<int> &point_ids,
                        Vec2s &keypoints,
                        Vec3s &object_points) const override;

  /** Check if corner `id` was detected */
  bool has(const int id) const;

  /** Add measurement of corner `id` */
  void add(const int id, const Vec2 &kp);

  /** Remove measurement of corner `id` */
  void remove(const int id);

protected:
  /**
   * Save target to file, `config` are the lines (without the trailing newline)
   * describing the target configuration, e.g. "rows 8"
   */
  int save(const fs::path &save_path,
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
