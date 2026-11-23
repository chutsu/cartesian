#pragma once
#include <cassert>

#include "Core.hpp"
#include "calib/aprilgrid/AprilGridConfig.hpp"

#include <rerun.hpp>
#include <eigen3/Eigen/Core>
#include <opencv2/core.hpp>

namespace cartesian {

class Logger {
public:
  /** How the logger reaches the rerun viewer */
  enum class Mode {
    CONNECT, // Connect to a viewer that is already running
    SPAWN,   // Start a new viewer
    OFF,     // Log nothing, do not start or connect to a viewer
  };

private:
  std::unique_ptr<rerun::RecordingStream> rec_;
  bool enabled_ = true;

  /** Convert timestamp to std::chrono::time_point **/
  std::chrono::time_point<std::chrono::system_clock>
  convert_timestamp(const timestamp_t ts);

  /** Convert cv::Mat to rerun::Image **/
  rerun::Image convert_image(const cv::Mat &image) const;

  /** Convert Mat4 to rerun::Transform3D **/
  rerun::Transform3D convert_pose(const Mat4 &pose,
                                  const float axis_length = 1.0) const;

  /** Convert std::vector<Vec3> to rerun::Points3D **/
  rerun::Points3D convert_points(const std::vector<Vec3> &points,
                                 const std::vector<Vec3i> &colors,
                                 const std::vector<double> &radii) const;

  /** Convert std::vector<Vec3> to rerun::LineStrips3D **/
  rerun::LineStrips3D convert_line(const std::vector<Vec3> &points,
                                   const Vec3i &color,
                                   const float &radii);

public:
  /**
   * How the logger reaches the rerun viewer, from the environment variable
   * `CARTESIAN_VIZ`:
   *
   * - unset, or anything else: `Mode::CONNECT`, connect to a viewer that is
   *   already running, at `rerun+http://127.0.0.1:9876/proxy` or
   *   `CARTESIAN_VIZ_URL`
   * - `spawn`: `Mode::SPAWN`, start a new viewer
   * - `0`, `false`, `off` or `no`: `Mode::OFF`, log nothing and do not start
   *   or connect to a viewer, e.g. in docker or in CI
   *
   *   rerun &                        # start the viewer once
   *   ./build/unittests              # the tests log to it
   *   CARTESIAN_VIZ=0 ./build/unittests
   */
  static Mode mode();

  /** Check if the logger sends data to a viewer, i.e. the mode is not `Mode::OFF` */
  static bool enabled();

  /**
   * Constructor, `session_name` is the name of the application logging, which
   * groups its recordings, `recording_name` is the name shown for this
   * recording in the viewer, if empty the viewer picks one
   */
  Logger(const std::string session_name = "cartesian",
         const std::string recording_name = "");
  virtual ~Logger() = default;

  /** Set the name of the recording, shown in the viewer */
  void set_recording_name(const std::string &recording_name);

  /*****************************************************************************
   * IMAGE
   ***************************************************************************/

  /** Log Image */
  void log_image(const std::string &topic,
                 const timestamp_t ts,
                 const cv::Mat &image);

  /*****************************************************************************
   * SCALAR
   ***************************************************************************/

  /** Log scalar */
  void init_series_line(const std::string &topic,
                        const Vec3i &color,
                        const float line_width);
  void log_scalar(const std::string &topic,
                  const timestamp_t ts,
                  const double value);
  void log_scalar(const std::string &topic,
                  const std::map<timestamp_t, double> &values);

  /*****************************************************************************
   * LINE
   ***************************************************************************/

  /** Log line */
  void log_line(const std::string &topic,
                const std::vector<Vec3> &points,
                const Vec3i &color = Vec3i{255.0, 0.0, 0.0},
                const float radii = 0.01);

  /*****************************************************************************
   * POINTS
   ***************************************************************************/

  /** Log Points */
  void log_points(const std::string &topic,
                  const std::vector<Vec3> &points,
                  const std::vector<Vec3i> &colors,
                  const std::vector<double> &radii);
  void log_points(const std::string &topic,
                  const timestamp_t ts,
                  const std::vector<Vec3> &points,
                  const std::vector<Vec3i> &colors,
                  const std::vector<double> &radii);

  /*****************************************************************************
   * POSE
   ***************************************************************************/

  /** Log pose */
  void log_pose(const std::string &topic,
                const Mat4 &pose,
                const float axis_length = 1.0);
  void log_pose(const std::string &topic,
                const timestamp_t ts,
                const Mat4 &pose,
                const float axis_length = 1.0);
  void log_poses(const std::string &topic,
                 const std::map<timestamp_t, Mat4> &poses,
                 const float axis_length = 1.0);

  /*****************************************************************************
   * TRAJECTORY
   ***************************************************************************/

  /** Log trajectory */
  void log_trajectory(const std::string &topic,
                      const std::map<timestamp_t, Mat4> &poses,
                      const Vec3i &color = Vec3i{255.0, 0.0, 0.0},
                      const float radii = 0.005);

  /*****************************************************************************
   * CALIBRATION TARGET
   ***************************************************************************/

  /** Log Calibration Target */
  void log_target(const std::string &topic,
                  const AprilGridConfig &config,
                  const Mat4 &T_WT);

  /*****************************************************************************
   * TEXT
   ***************************************************************************/

  /** Log text */
  void log_text(const std::string &topic, const std::string &text);
  void log_text(const std::string &topic,
                const timestamp_t ts,
                const std::string &text);
};

} // namespace cartesian
