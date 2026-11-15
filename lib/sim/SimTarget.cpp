#include "SimTarget.hpp"

#include <opencv2/imgproc.hpp>


#include "../calib/AprilGridDetector.hpp"
#include "../calib/CharucoDetector.hpp"
#include "../calib/CheckerboardDetector.hpp"

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "ethz_apriltag/Tag36h11.h"

namespace cartesian {

/** Convert meters to a whole number of pixels (fatal if not whole) */
static int meters2px(const double meters, const double ppm) {
  const double px = meters * ppm;
  if (std::abs(px - std::round(px)) > 1e-6) {
    FATAL("%f [m] is not a whole number of pixels at %f ppm!", meters, ppm);
  }
  return static_cast<int>(std::round(px));
}

/******************************************************************************
 * BOARD TEXTURES
 *
 * 8-bit grayscale images of the boards, with the bottom left corner of the
 * board at the bottom left of the image, x pointing right and y pointing up.
 *****************************************************************************/

/**
 * Make AprilGrid tag image with 1 pixel per module, 6x6 data bits and a black
 * border: 2 modules wide for the "kaess" detector (10x10) and 1 module wide for
 * the "olsen" detector (8x8). The two detectors use different (incompatible)
 * tag36h11 code tables and bit layouts, so the texture has to match the
 * detector:
 *
 * - "kaess": ethz_apriltag (AprilTag 1), what kalibr printed grids use
 * - "olsen": apriltag (AprilTag 3)
 */
static cv::Mat aprilgrid_tag(const std::string &detector_type, const int tag_id) {
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
        tag.at<uchar>(r + border, c + border) =
            ((code >> bit) & 1ULL) ? 255 : 0;
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

static cv::Mat aprilgrid_board(const AprilGridConfig &config,
                               const std::string &detector_type,
                               const bool kalibr_style,
                               const double ppm) {
  const int w = meters2px(config.get_width(), ppm);
  const int h = meters2px(config.get_height(), ppm);
  const int tag_px = meters2px(config.tag_size, ppm);
  const int gap_px = meters2px(config.tag_size * config.tag_spacing, ppm);
  const int num_modules = aprilgrid_tag(detector_type, 0).rows;
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
    cv::resize(aprilgrid_tag(detector_type, tag_id),
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

static cv::Mat checkerboard_board(const CheckerboardConfig &config,
                                  const double ppm) {
  const int sq = meters2px(config.square_size, ppm);
  cv::Mat board(config.rows * sq, config.cols * sq, CV_8UC1, cv::Scalar{255});
  for (int r = 0; r < config.rows; ++r) {
    for (int c = 0; c < config.cols; ++c) {
      if ((r + c) % 2 == 0) {
        cv::rectangle(board,
                      cv::Rect{c * sq, r * sq, sq, sq},
                      cv::Scalar{0},
                      cv::FILLED);
      }
    }
  }
  return board;
}

static cv::Mat charuco_board_image(const CharucoConfig &config,
                                   const double ppm) {
  cv::Mat image;
  charuco_board(config).generateImage(
      cv::Size{meters2px(config.get_width(), ppm),
               meters2px(config.get_height(), ppm)},
      image,
      0,  // Margin
      1); // Marker border bits
  return image;
}

/******************************************************************************
 * SIM TARGET
 *****************************************************************************/

cv::Mat SimTarget::render(const double margin, const double ppm) const {
  const int m = meters2px(margin, ppm);
  cv::Mat texture;
  cv::copyMakeBorder(render_board(ppm),
                     texture,
                     m,
                     m,
                     m,
                     m,
                     cv::BORDER_CONSTANT,
                     cv::Scalar{255});
  return texture;
}

AprilGridConfig sim_aprilgrid_config() {
  AprilGridConfig config;
  config.target_id = 0;
  config.tag_rows = 6;
  config.tag_cols = 6;
  config.tag_size = 0.08;
  config.tag_spacing = 0.25; // Ratio of tag_size
  config.tag_id_offset = 0;
  return config;
}

CheckerboardConfig sim_checkerboard_config() {
  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.07;
  return config;
}

CharucoConfig sim_charuco_config() {
  CharucoConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.07;
  config.marker_size = 0.056;
  config.dictionary = "DICT_5X5_250";
  return config;
}

SimTarget make_sim_aprilgrid(const bool kalibr_style,
                             const std::string &detector_type,
                             const bool refine_corners) {
  const auto config = sim_aprilgrid_config();
  auto detector = std::make_shared<AprilGridDetector>(config);
  detector->set_detector(detector_type);
  detector->set_refine_corners(refine_corners);

  SimTarget t;
  t.name = std::string("aprilgrid_") + (kalibr_style ? "kalibr" : "plain") +
           "_" + detector_type;
  t.width = config.get_width();
  t.height = config.get_height();
  t.corners = config.get_object_points();
  t.render_board = [config, detector_type, kalibr_style](const double ppm) {
    return aprilgrid_board(config, detector_type, kalibr_style, ppm);
  };
  t.detect = [detector](const cv::Mat &image) {
    std::vector<int> point_ids;
    Vec2s keypoints;
    Vec3s object_points;
    detector->detect(0, 0, image)[0]->get_measurements(point_ids,
                                                       keypoints,
                                                       object_points);
    std::map<int, Vec2> detected;
    for (size_t i = 0; i < point_ids.size(); ++i) {
      detected[point_ids[i]] = keypoints[i];
    }
    return detected;
  };
  return t;
}

SimTarget make_sim_checkerboard(const bool refine_corners) {
  const auto config = sim_checkerboard_config();
  auto detector = std::make_shared<CheckerboardDetector>(config, refine_corners);

  SimTarget t;
  t.name = "checkerboard";
  t.width = config.get_width();
  t.height = config.get_height();
  t.corners = config.get_board_points();
  t.render_board = [config](const double ppm) {
    return checkerboard_board(config, ppm);
  };
  t.detect = [detector](const cv::Mat &image) {
    // The detector returns the corners in the order of OpenCV's
    // findChessboardCornersSB(), the ids of `get_board_points()`
    return detector->detect(0, 0, image)->keypoints;
  };
  return t;
}

SimTarget make_sim_charuco(const bool refine_corners) {
  const auto config = sim_charuco_config();
  auto detector = std::make_shared<CharucoDetector>(config, refine_corners);

  SimTarget t;
  t.name = "charuco";
  t.width = config.get_width();
  t.height = config.get_height();
  t.corners = config.get_board_points();
  t.render_board = [config](const double ppm) {
    return charuco_board_image(config, ppm);
  };
  t.detect = [detector](const cv::Mat &image) {
    return detector->detect(0, 0, image)->keypoints;
  };
  return t;
}

} // namespace cartesian
