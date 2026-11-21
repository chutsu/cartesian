#pragma once

#include <string>
#include <algorithm>

#include "../../core/Core.hpp"
#include "../../camera/CameraModel.hpp"
#include "../CalibTarget.hpp"
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

public:
  /*****************************************************************************
   * CONSTRUCTORS
   ***************************************************************************/

  AprilGrid(const timestamp_t &timestamp,
            const int camera_id,
            const AprilGridConfig &config);
  ~AprilGrid() = default;

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

  /** Point id of corner `corner_index` of tag `tag_id`, ids start at 0 */
  int get_point_id(const int tag_id, const int corner_index) const;

  /** Tag id of point `point_id` */
  int get_tag_id(const int point_id) const;

  /** Corner index (0 - 3) of point `point_id` */
  int get_corner_index(const int point_id) const;

  /** Get the 2D center of the AprilGrid */
  Vec2 get_center_2d() const;

  /** Get the 3D center of the AprilGrid */
  Vec3 get_center_3d() const;

  /*****************************************************************************
   * MEASUREMENTS
   ***************************************************************************/

  // The measurements are those of CalibTarget, by point id
  using CalibTarget::add;
  using CalibTarget::has;
  using CalibTarget::remove;

  /** Check to see if AprilGrid has specific tag id and corner index */
  bool has(const int tag_id, const int corner_index) const;

  /** Add measurement of a corner of a tag, ignored if the tag is not on the grid */
  void add(const int tag_id, const int corner_index, const Vec2 &kp);

  /** Remove measurement of a corner of a tag */
  void remove(const int tag_id, const int corner_index);

  /** Remove the measurements of all the corners of a tag */
  void remove_tag(const int tag_id);

  /*****************************************************************************
   * FILE I/O
   ***************************************************************************/

  /** Save AprilGrid **/
  int save(const fs::path &save_path) const override;

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
