#include "AprilGridDetector.hpp"

#include <algorithm>

namespace cartesian {

AprilGridDetector::AprilGridDetector(const AprilGridConfig &target_config)
    : CalibTargetDetector{false} {
  target_configs_[target_config.target_id] = target_config;
  form_target_lut();
  olsen_setup();
  kaess_setup();
}

AprilGridDetector::AprilGridDetector(
    const std::map<int, AprilGridConfig> &target_configs)
    : CalibTargetDetector{false}, target_configs_{target_configs} {
  form_target_lut();
  olsen_setup();
  kaess_setup();
}

AprilGridDetector::~AprilGridDetector() {
  olsen_cleanup();
}

void AprilGridDetector::form_target_lut() {
  for (const auto [target_id, target_config] : target_configs_) {
    const int tag_id_offset = target_config.tag_id_offset;
    const int num_tags = target_config.tag_rows * target_config.tag_cols;
    const int tag_id_end = tag_id_offset + num_tags;

    for (int tag_id = tag_id_offset; tag_id < tag_id_end; ++tag_id) {
      if (target_lut_.count(tag_id)) {
        FATAL("tag_id: %d already exists in target_id: %d!",
              tag_id,
              target_lut_[tag_id]);
      }

      target_lut_[tag_id] = target_id;
    }
  }
}

void AprilGridDetector::olsen_setup() {
  olsen_tf_ = tag36h11_create();

  olsen_detector_ = apriltag_detector_create();
  olsen_detector_->quad_decimate = 1.0;
  olsen_detector_->quad_sigma = 0.0; // Blur
  olsen_detector_->nthreads = 2;
  olsen_detector_->debug = 0;
  olsen_detector_->refine_edges = 1;

  apriltag_detector_add_family(olsen_detector_, olsen_tf_);
}

void AprilGridDetector::olsen_cleanup() {
  apriltag_detector_destroy(olsen_detector_);
  tag36h11_destroy(olsen_tf_);
}

void AprilGridDetector::olsen_detect(const cv::Mat &image,
                                     std::vector<int> &tag_ids,
                                     std::vector<int> &corner_indicies,
                                     std::vector<Vec2> &keypoints) {
  assert(image.channels() == 1);
  const int image_width = image.cols;
  const int image_height = image.rows;

  // Make an image_u8_t header for the Mat data
  image_u8_t im = {.width = image.cols,
                   .height = image.rows,
                   .stride = (int32_t) image.step[0],
                   .buf = image.data};

  // Detector tags
  zarray_t *detections = apriltag_detector_detect(olsen_detector_, &im);
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);

    // Check tag id
    // if (det->id < 0 || det->id >= (tag_rows_ * tag_cols_)) {
    //   continue;
    // }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= det->p[i][0] < min_border_dist_;
      bad_tag |= det->p[i][0] > image_width - min_border_dist_;
      bad_tag |= det->p[i][1] < min_border_dist_;
      bad_tag |= det->p[i][1] > image_height - min_border_dist_;
      if (bad_tag) {
        break;
      }
    }

    tag_ids.push_back(det->id); // Bottom left
    tag_ids.push_back(det->id); // Bottom right
    tag_ids.push_back(det->id); // Top right
    tag_ids.push_back(det->id); // Top left

    corner_indicies.push_back(0); // Bottom left
    corner_indicies.push_back(1); // Bottom right
    corner_indicies.push_back(2); // Top right
    corner_indicies.push_back(3); // Top left

    // apriltag has the center of pixel (0, 0) at (0.5, 0.5), shift the corners
    // to our convention (and OpenCV's), where it is at (0, 0)
    const Vec2 half_pixel{0.5, 0.5};
    keypoints.push_back(Vec2{det->p[0][0], det->p[0][1]} - half_pixel); // BL
    keypoints.push_back(Vec2{det->p[1][0], det->p[1][1]} - half_pixel); // BR
    keypoints.push_back(Vec2{det->p[2][0], det->p[2][1]} - half_pixel); // TR
    keypoints.push_back(Vec2{det->p[3][0], det->p[3][1]} - half_pixel); // TL
  }
  apriltag_detections_destroy(detections);
}

void AprilGridDetector::kaess_setup() {
  kaess_detector_ = std::make_unique<ethz_apriltag::TagDetector>(
      ethz_apriltag::tagCodes36h11);
}

void AprilGridDetector::kaess_detect(const cv::Mat &image,
                                     std::vector<int> &tag_ids,
                                     std::vector<int> &corner_indicies,
                                     std::vector<Vec2> &keypoints) {
  assert(image.channels() == 1);
  const int image_width = image.cols;
  const int image_height = image.rows;

  const auto detections = kaess_detector_->extractTags(image);
  for (const auto &tag : detections) {
    // Check detection
    if (tag.good == false) {
      continue;
    }

    // Check tag id
    // if (tag.id < 0 || tag.id >= (tag_rows_ * tag_cols_)) {
    //   continue;
    // }

    // Check if too close to image bounds
    bool bad_tag = false;
    for (int i = 0; i < 4; i++) {
      bad_tag |= tag.p[i].first < min_border_dist_;
      bad_tag |= tag.p[i].first > image_width - min_border_dist_;
      bad_tag |= tag.p[i].second < min_border_dist_;
      bad_tag |= tag.p[i].second > image_height - min_border_dist_;
      if (bad_tag) {
        break;
      }
    }

    tag_ids.push_back(tag.id); // Bottom left
    tag_ids.push_back(tag.id); // Bottom right
    tag_ids.push_back(tag.id); // Top right
    tag_ids.push_back(tag.id); // Top left

    corner_indicies.push_back(0); // Bottom left
    corner_indicies.push_back(1); // Bottom right
    corner_indicies.push_back(2); // Top right
    corner_indicies.push_back(3); // Top left

    keypoints.emplace_back(tag.p[0].first, tag.p[0].second); // Bottom left
    keypoints.emplace_back(tag.p[1].first, tag.p[1].second); // Bottom right
    keypoints.emplace_back(tag.p[2].first, tag.p[2].second); // Top right
    keypoints.emplace_back(tag.p[3].first, tag.p[3].second); // Top left
  }
}

void AprilGridDetector::set_detector(const std::string &detector_type) {
  detector_type_ = detector_type;
}

std::vector<CalibTargetPtr>
AprilGridDetector::detect(const timestamp_t ts,
                          const int camera_id,
                          const cv::Mat &image) {
  // Detect AprilTags
  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  std::vector<Vec2> keypoints;
  if (detector_type_ == "kaess") {
    kaess_detect(image, tag_ids, corner_indicies, keypoints);
  } else if (detector_type_ == "olsen") {
    olsen_detect(image, tag_ids, corner_indicies, keypoints);
  } else {
    throw std::runtime_error("Invalid detector type! " + detector_type_);
  }

  // Refine the corners of every tag, in a window of about half a module (a tag
  // is 10 modules wide for kaess, with a 2 module black border, and 8 for
  // olsen)
  if (refine_corners_) {
    const double modules = (detector_type_ == "kaess") ? 10.0 : 8.0;
    for (size_t i = 0; i + 3 < keypoints.size(); i += 4) {
      const double side = (keypoints[i + 1] - keypoints[i]).norm();
      const int half_window = std::clamp(
          static_cast<int>(std::round(0.6 * side / modules)), 2, 8);
      std::vector<Vec2> corners(keypoints.begin() + i, keypoints.begin() + i + 4);
      refine_corners_subpix(image, corners, half_window, half_window);
      std::copy(corners.begin(), corners.end(), keypoints.begin() + i);
    }
  }

  // Form AprilGrids
  std::map<int, std::shared_ptr<AprilGrid>> aprilgrids;
  for (const auto &[target_id, target_config] : target_configs_) {
    const auto config = target_configs_[target_id];
    aprilgrids[target_id] = std::make_shared<AprilGrid>(ts, camera_id, config);
  }

  // Add measurement
  for (size_t i = 0; i < tag_ids.size(); ++i) {
    const auto tag_id = tag_ids[i];
    if (target_lut_.count(tag_id) == 0) {
      continue;
    }

    const auto target_id = target_lut_[tag_id];
    const auto corner_index = corner_indicies[i];
    aprilgrids[target_id]->add(tag_id, corner_index, keypoints[i]);
  }

  // Form result
  std::vector<CalibTargetPtr> result;
  for (const auto &[target_id, target] : aprilgrids) {
    result.push_back(target);
  }

  return result;
}

} // namespace cartesian
