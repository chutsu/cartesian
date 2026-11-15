#include "Logger.hpp"

#include <array>

namespace cartesian {

Logger::Logger(const std::string session_name) {
  rec_ = std::make_unique<rerun::RecordingStream>(session_name);
  rec_->spawn().exit_on_failure();
  rec_->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
}

std::chrono::time_point<std::chrono::system_clock>
Logger::convert_timestamp(const timestamp_t ts) {
  return std::chrono::time_point<std::chrono::system_clock>{
      std::chrono::nanoseconds{ts}};
}

rerun::Image Logger::convert_image(const cv::Mat &image) const {
  rerun::ColorModel color = rerun::ColorModel::BGR;
  if (image.channels() == 1) {
    color = rerun::ColorModel::L;
  } else if (image.channels() == 3) {
    color = rerun::ColorModel::BGR;
  }

  const auto data = rerun::borrow(image.data, image.total() * image.elemSize());
  const rerun::WidthHeight image_size(image.cols, image.rows);
  const auto rr_image = rerun::Image(data, image_size, color);
  return rr_image;
}

rerun::Transform3D Logger::convert_pose(const Mat4 &pose,
                                        const float axis_length) const {
  const Eigen::Vector3f p = pose.block<3, 1>(0, 3).cast<float>();
  const Eigen::Matrix3f R = pose.block<3, 3>(0, 0).cast<float>();
  const rerun::Vec3D rr_position(p.data());
  const rerun::Mat3x3 rr_rotation(R.data());
  const rerun::components::Scale3D rr_scale{1.0, 1.0, 1.0};
  const rerun::components::AxisLength rr_length{axis_length};

  return rerun::Transform3D{rr_position, rr_rotation}
      .with_scale(rr_scale)
      .with_axis_length(rr_length);
}

rerun::Points3D Logger::convert_points(const std::vector<Vec3> &points,
                                       const std::vector<Vec3i> &colors,
                                       const std::vector<double> &radii) const {

  std::vector<rerun::Position3D> rerun_positions;
  std::vector<rerun::Color> rerun_colors;
  std::vector<rerun::Radius> rerun_radii;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto p = points[i];
    const auto c = colors[i];
    rerun_positions.emplace_back(p.x(), p.y(), p.z());
    rerun_colors.emplace_back(c.x(), c.y(), c.z());
    rerun_radii.push_back(radii[i]);
  }

  return rerun::Points3D(rerun_positions)
      .with_colors(rerun_colors)
      .with_radii(rerun_radii);
}

rerun::LineStrips3D Logger::convert_line(const std::vector<Vec3> &points,
                                         const Vec3i &color,
                                         const float &radii) {
  std::vector<std::vector<std::array<float, 3>>> rerun_points;
  rerun_points.resize(1);
  for (const auto &pt : points) {
    const float x = pt.x();
    const float y = pt.y();
    const float z = pt.z();
    rerun_points[0].push_back({x, y, z});
  }

  return rerun::LineStrips3D(rerun_points)
      .with_colors({rerun::Color(color.x(), color.y(), color.z())})
      .with_radii({radii});
}

/******************************************************************************
 * IMAGE
 *****************************************************************************/

void Logger::log_image(const std::string &topic,
                       const timestamp_t ts,
                       const cv::Mat &image) {
  rec_->set_time_timestamp("time", convert_timestamp(ts));
  rec_->log(topic, convert_image(image));
}

/******************************************************************************
 * SCALAR
 *****************************************************************************/

void Logger::init_series_line(const std::string &topic,
                              const Vec3i &color,
                              const float line_width) {
  rerun::Rgba32 rr_color(color.x(), color.y(), color.z());
  rec_->log_static(topic,
                   rerun::SeriesLines()
                       .with_colors(rr_color)
                       .with_names({topic})
                       .with_widths({line_width}));
}

void Logger::log_scalar(const std::string &topic,
                        const timestamp_t ts,
                        const double value) {
  rec_->set_time_timestamp("time", convert_timestamp(ts));
  rec_->log(topic, rerun::Scalars{value});
}

void Logger::log_scalar(const std::string &topic,
                        const std::map<timestamp_t, double> &values) {
  for (const auto &[ts, value] : values) {
    log_scalar(topic, ts, value);
  }
}

/******************************************************************************
 * LINE
 *****************************************************************************/

void Logger::log_line(const std::string &topic,
                      const std::vector<Vec3> &points,
                      const Vec3i &color,
                      const float radii) {
  rec_->log_static(topic, convert_line(points, color, radii));
}

/******************************************************************************
 * POINTS
 *****************************************************************************/

void Logger::log_points(const std::string &topic,
                        const std::vector<Vec3> &points,
                        const std::vector<Vec3i> &colors,
                        const std::vector<double> &radii) {
  assert(points.size() == colors.size());
  rec_->log_static(topic, convert_points(points, colors, radii));
}

void Logger::log_points(const std::string &topic,
                        const timestamp_t ts,
                        const std::vector<Vec3> &points,
                        const std::vector<Vec3i> &colors,
                        const std::vector<double> &radii) {
  assert(points.size() == colors.size());
  rec_->set_time_timestamp("time", convert_timestamp(ts));
  rec_->log(topic, convert_points(points, colors, radii));
}

/******************************************************************************
 * POSE
 *****************************************************************************/

void Logger::log_pose(const std::string &topic,
                      const Mat4 &pose,
                      const float axis_length) {
  rec_->log_static(topic, convert_pose(pose, axis_length));
}

void Logger::log_pose(const std::string &topic,
                      const timestamp_t ts,
                      const Mat4 &pose,
                      const float axis_length) {
  rec_->set_time_timestamp("time", convert_timestamp(ts));
  rec_->log(topic, convert_pose(pose, axis_length));
}

void Logger::log_poses(const std::string &topic,
                       const std::map<timestamp_t, Mat4> &poses,
                       const float axis_length) {
  for (const auto &[ts, pose] : poses) {
    log_pose(topic, ts, pose, axis_length);
  }
}

/******************************************************************************
 * TRAJECTORY
 *****************************************************************************/

void Logger::log_trajectory(const std::string &topic,
                            const std::map<timestamp_t, Mat4> &poses,
                            const Vec3i &color,
                            const float radii) {
  std::vector<Vec3> positions;
  for (const auto &[ts, pose] : poses) {
    positions.push_back(tf_trans(pose));
  }
  log_line(topic, positions, color, radii);
}

/******************************************************************************
 * CALIBRATION TARGET
 *****************************************************************************/

void Logger::log_target(const std::string &topic,
                        const AprilGridConfig &config,
                        const Mat4 &T_WT) {
  std::vector<Vec3> points_data;
  std::vector<Vec3i> points_colors;
  std::vector<double> points_radii;

  const auto num_tags = config.tag_rows * config.tag_cols;
  for (int tag_id = 0; tag_id < num_tags; ++tag_id) {
    for (int corner_index = 0; corner_index < 4; ++corner_index) {
      const Vec3 p_T = config.get_object_point(tag_id, corner_index);
      const Vec3 p_W = tf_point(T_WT, p_T);
      points_data.push_back(p_W);
      points_colors.emplace_back(255, 0, 0);
      points_radii.emplace_back(0.01);
    }
  }

  log_pose(topic + "-pose", T_WT, config.tag_size);
  log_points(topic + "-points", points_data, points_colors, points_radii);
}

/******************************************************************************
 * TEXT
 *****************************************************************************/

void Logger::log_text(const std::string &topic, const std::string &text) {
  rec_->log_static(topic, rerun::TextLog(text));
}

void Logger::log_text(const std::string &topic,
                      const timestamp_t ts,
                      const std::string &text) {
  rec_->set_time_timestamp("time", convert_timestamp(ts));
  rec_->log(topic, rerun::TextLog(text));
}

} // namespace cartesian
