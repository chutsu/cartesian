#include "AprilGridTarget.hpp"

#include "ethz_apriltag/Tag36h11.h"

namespace cartesian {

AprilGridConfig AprilGridTarget::make_config() {
  AprilGridConfig config;
  config.target_id = 0;
  config.tag_rows = 6;
  config.tag_cols = 6;
  config.tag_size = 0.08;
  config.tag_spacing = 0.25; // Ratio of tag_size
  config.tag_id_offset = 0;
  return config;
}

AprilGridTarget::AprilGridTarget(const bool kalibr_style_,
                                 const std::string &detector_type_)
    : detector_type{detector_type_}, kalibr_style{kalibr_style_},
      config{make_config()}, detector{config} {
  detector.set_detector(detector_type);
}

std::string AprilGridTarget::name() const {
  const std::string style = kalibr_style ? "kalibr" : "plain";
  return "aprilgrid_" + style + "_" + detector_type;
}

double AprilGridTarget::width() const { return config.get_width(); }

double AprilGridTarget::height() const { return config.get_height(); }

std::map<int, Vec3> AprilGridTarget::corners() const {
  return config.get_object_points();
}

std::map<int, Vec2> AprilGridTarget::detect(const cv::Mat &image) {
  const auto grids = detector.detect(0, 0, image);

  std::vector<int> point_ids;
  Vec2s keypoints;
  Vec3s object_points;
  grids[0]->get_measurements(point_ids, keypoints, object_points);

  std::map<int, Vec2> detected;
  for (size_t i = 0; i < point_ids.size(); ++i) {
    detected[point_ids[i]] = keypoints[i];
  }
  return detected;
}

cv::Mat AprilGridTarget::make_tag(const std::string &detector_type,
                                  const int tag_id) {
  if (detector_type == "kaess") {
    // ethz_apriltag expects a black border of 2 modules (TagDetector's
    // `blackBorder` defaults to 2), so a tag is 2 + 6 + 2 = 10 modules wide.
    // Code MSB is the top left data bit, going row major. 1 is white.
    const int border = 2;
    const int dim = 6;
    const int num_modules = 2 * border + dim;
    const auto code = ethz_apriltag::tagCodes36h11.codes.at(tag_id);
    cv::Mat tag(num_modules, num_modules, CV_8UC1, cv::Scalar{0});
    for (int r = 0; r < dim; ++r) {
      for (int c = 0; c < dim; ++c) {
        const int bit = dim * dim - 1 - (r * dim + c);
        tag.at<uchar>(r + border, c + border) = ((code >> bit) & 1ULL) ? 255 : 0;
      }
    }
    return tag;
  }

  // apriltag has a black border of 1 module, so a tag is 1 + 6 + 1 = 8 modules
  // wide. Its image has a 1 pixel white ring, crop it off.
  const int num_modules = 8;
  apriltag_family_t *family = tag36h11_create();
  image_u8_t *im = apriltag_to_image(family, tag_id);
  cv::Mat tag{im->height, im->width, CV_8UC1, im->buf, (size_t) im->stride};
  cv::Mat tag_cropped = tag(cv::Rect{1, 1, num_modules, num_modules}).clone();
  image_u8_destroy(im);
  tag36h11_destroy(family);
  return tag_cropped;
}

cv::Mat AprilGridTarget::render_board() const {
  const int w = meters2px(width());
  const int h = meters2px(height());
  const int tag_px = meters2px(config.tag_size);
  const int gap_px = meters2px(config.tag_size * config.tag_spacing);
  const int num_modules = make_tag(detector_type, 0).rows;
  if (tag_px % num_modules != 0) {
    FATAL("tag size must be a multiple of %d pixels!", num_modules);
  }

  // Tags
  cv::Mat board(h, w, CV_8UC1, cv::Scalar{255});
  for (int tag_id = 0; tag_id < config.get_num_tags(); ++tag_id) {
    int i = 0;
    int j = 0;
    config.get_grid_index(tag_id, i, j);

    cv::Mat tag_scaled;
    cv::resize(make_tag(detector_type, tag_id),
               tag_scaled,
               cv::Size{tag_px, tag_px},
               0,
               0,
               cv::INTER_NEAREST);

    // Paste tag, with the grid origin (i, j) = (0, 0) at the bottom left
    const int x = j * (tag_px + gap_px);
    const int y = h - (i * (tag_px + gap_px)) - tag_px;
    tag_scaled.copyTo(board(cv::Rect{x, y, tag_px, tag_px}));
  }

  // Black squares at the gap intersections between tags (kalibr style)
  if (kalibr_style) {
    for (int i = 1; i < config.tag_rows; ++i) {
      for (int j = 1; j < config.tag_cols; ++j) {
        const int x = j * (tag_px + gap_px) - gap_px;
        const int y = h - (i * (tag_px + gap_px));
        cv::rectangle(board,
                      cv::Rect{x, y, gap_px, gap_px},
                      cv::Scalar{0},
                      cv::FILLED);
      }
    }
  }

  return board;
}

} // namespace cartesian
