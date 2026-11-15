#pragma once

#include <string>
#include <algorithm>
#include <set>
#include <unordered_map>

#include "../core/Core.hpp"
#include "../camera/CameraModel.hpp"
#include "CalibTarget.hpp"
#include "AprilGridConfig.hpp"

// AprilTags3 by Ed Olsen
extern "C" {
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/getopt.h"
}

// AprilTags by Michael Kaess
#include "ethz_apriltag/TagDetector.h"
#include "ethz_apriltag/Tag36h11.h"

namespace cartesian {

/** AprilGrid */
class AprilGrid : public CalibTarget {
private:
  AprilGridConfig config_;

  // Data
  struct TagDetection {
    std::set<int> corner_indicies;
    std::unordered_map<int, Vec2> keypoints;
  };
  std::unordered_map<int, TagDetection> data_;

public:
  /*****************************************************************************
   * CONSTRUCTORS
   ***************************************************************************/

  AprilGrid(const timestamp_t &timestamp,
            const int camera_id,
            const AprilGridConfig &config);
  AprilGrid(const AprilGrid &src);
  ~AprilGrid() = default;

  /*****************************************************************************
   * STATE
   ***************************************************************************/

  /** Check if detected */
  bool detected() const override;

  /** Return number detected */
  int get_num_detected() const override;

  /*****************************************************************************
   * CONFIG
   ***************************************************************************/

  /** Return AprilGrid config */
  AprilGridConfig get_config() const;

  /** Return number of tag rows */
  int get_tag_rows() const;

  /** Return number of tag cols */
  int get_tag_cols() const;

  /** Return number of tag size */
  double get_tag_size() const;

  /** Return number of tag spacing */
  double get_tag_spacing() const;

  /** Return tag id offset */
  int get_tag_id_offset() const;

  /*****************************************************************************
   * GEOMETRY
   ***************************************************************************/

  /** Calculate width and height */
  Vec2 get_width_height() const;

  /** Calculate center */
  Vec2 get_center() const;

  /** Calculate AprilGrid tag index based on tag id */
  void get_grid_index(const int tag_id, int &i, int &j) const;

  /** Calculate object point based on tag id and corner index */
  Vec3 get_object_point(const int tag_id, const int corner_index) const;

  /** Calculate object point based on point id */
  Vec3 get_object_point(const int point_id) const;

  /** Get the 2D center of the AprilGrid */
  Vec2 get_center_2d() const;

  /** Get the 3D center of the AprilGrid */
  Vec3 get_center_3d() const;

  /*****************************************************************************
   * MEASUREMENTS
   ***************************************************************************/

  /** Get measurements */
  void get_measurements(std::vector<int> &tag_ids,
                        std::vector<int> &corner_indicies,
                        Vec2s &keypoints,
                        Vec3s &object_points) const;

  /** Get measurements */
  void get_measurements(std::vector<int> &point_ids,
                        Vec2s &keypoints,
                        Vec3s &object_points) const override;

  /** Check to see if AprilGrid has specific tag id and corner index */
  bool has(const int tag_id, const int corner_index) const;

  /** Add measurmeent */
  void add(const int tag_id, const int corner_index, const Vec2 &kp);

  /** Remove measurmeent */
  void remove(const int tag_id, const int corner_index);

  /** Remove measurmeent */
  void remove(const int tag_id);

  /*****************************************************************************
   * FILE I/O
   ***************************************************************************/

  /** Save AprilGrid **/
  int save(const fs::path &save_path) const;

  /** Load AprilGrid **/
  static std::shared_ptr<AprilGrid> load(const fs::path &data_path);

  /** Load CalibTargets **/
  static std::vector<std::shared_ptr<CalibTarget>>
  load_directory(const fs::path &dir_path);

  /*****************************************************************************
   * VISUALIZATION
   ***************************************************************************/

  /** Draw AprilGrid */
  cv::Mat draw(const cv::Mat &image,
               const int marker_size = 2,
               const cv::Scalar &color = cv::Scalar{0, 0, 255}) const;

  /** Imshow AprilGrid */
  void imshow(const std::string &title, const cv::Mat &image) const;
};

} // namespace cartesian
