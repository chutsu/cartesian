#include <gtest/gtest.h>
#include "core/Logger.hpp"

namespace cartesian {

TEST(Logger, viz_disabled_by_env) {
  // Restore the environment of the test run when done
  const char *prev = std::getenv("CARTESIAN_VIZ");
  const std::string prev_value = prev ? prev : "";

  for (const char *off : {"0", "false", "off", "no"}) {
    setenv("CARTESIAN_VIZ", off, 1);
    ASSERT_FALSE(Logger::enabled());
  }
  for (const char *on : {"1", "true", "on", ""}) {
    setenv("CARTESIAN_VIZ", on, 1);
    ASSERT_TRUE(Logger::enabled());
  }
  unsetenv("CARTESIAN_VIZ");
  ASSERT_TRUE(Logger::enabled());

  // By default connect to a viewer, only spawn one if asked to
  ASSERT_EQ(Logger::mode(), Logger::Mode::CONNECT);
  setenv("CARTESIAN_VIZ", "1", 1);
  ASSERT_EQ(Logger::mode(), Logger::Mode::CONNECT);
  setenv("CARTESIAN_VIZ", "spawn", 1);
  ASSERT_EQ(Logger::mode(), Logger::Mode::SPAWN);
  ASSERT_TRUE(Logger::enabled());
  setenv("CARTESIAN_VIZ", "0", 1);
  ASSERT_EQ(Logger::mode(), Logger::Mode::OFF);

  // A disabled logger does not start a viewer, and logging is a no-op
  setenv("CARTESIAN_VIZ", "0", 1);
  Logger log;
  ASSERT_FALSE(log.enabled());
  log.set_recording_name("name");
  log.log_text("text", "hello");
  log.log_text("text", 0, "hello");
  log.log_scalar("scalar", 0, 1.0);
  log.log_pose("pose", 0, eye(4));
  log.log_image("image", 0, cv::Mat(10, 10, CV_8UC3, cv::Scalar(0, 0, 0)));

  if (prev) {
    setenv("CARTESIAN_VIZ", prev_value.c_str(), 1);
  } else {
    unsetenv("CARTESIAN_VIZ");
  }
}

TEST(DISABLED_Logger, log_image) {
  Logger log;

  // Create test image
  cv::Mat img(400, 400, CV_8UC3, cv::Scalar(255, 255, 255));
  // -- Draw circle
  const cv::Point circle_center(200, 200);
  const int circle_radius = 80;
  const cv::Scalar circle_color(255, 0, 0);
  const int circle_thickness = 3;
  cv::circle(img, circle_center, circle_radius, circle_color, circle_thickness);
  // -- Draw rectangle
  const cv::Point rect_vert1(50, 50);
  const cv::Point rect_vert2(150, 150);
  const cv::Scalar rect_color(0, 255, 0);
  const int rect_thickness = -1;
  cv::rectangle(img, rect_vert1, rect_vert2, rect_color, rect_thickness);
  // -- Draw text
  const std::string text_str = "Hello World!";
  const cv::Point text_pos{50, 350};
  const int text_font = cv::FONT_HERSHEY_SIMPLEX;
  const int text_scale = 1;
  const cv::Scalar text_color(0, 0, 0);
  cv::putText(img, text_str, text_pos, text_font, text_scale, text_color, 2);

  // Log image
  log.log_image("image", 0, img);
}

TEST(DISABLED_Logger, log_points) {
  Logger log;

  std::vector<Vec3> points;
  std::vector<Vec3i> colors;
  std::vector<double> radii;
  for (int i = 0; i < 10; ++i) {
    const auto x = randf(-1.0, 1.0);
    const auto y = randf(-1.0, 1.0);
    const auto z = randf(-1.0, 1.0);

    const int r = randf(0.0, 1.0) * 255;
    const int g = randf(0.0, 1.0) * 255;
    const int b = randf(0.0, 1.0) * 255;

    points.emplace_back(x, y, z);
    colors.emplace_back(r, g, b);
    radii.emplace_back(0.1);
  }

  log.log_points("points", 0, points, colors, radii);
}

TEST(DISABLED_Logger, log_pose) {
  Logger log;

  std::vector<Mat4> poses;
  for (int i = 0; i < 10; ++i) {
    const auto x = randf(-1.0, 1.0);
    const auto y = randf(-1.0, 1.0);
    const auto z = randf(-1.0, 1.0);

    const auto roll = randf(-1.0, 1.0);
    const auto pitch = randf(-1.0, 1.0);
    const auto yaw = randf(-1.0, 1.0);

    const Mat3 R = euler321(Vec3{roll, pitch, yaw});
    const Vec3 t{x, y, z};
    const Mat4 pose = tf(R, t);

    log.log_pose("poses", i, pose);
  }
}

} // namespace cartesian
