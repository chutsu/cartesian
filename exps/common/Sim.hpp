#pragma once
#include <random>
#include <string>
#include <vector>

#include "camera/CameraGeometry.hpp"
#include "core/Core.hpp"

#include "DetectorTarget.hpp"

namespace cartesian {

/******************************************************************************
 * CAMERA
 *****************************************************************************/

/** Camera model specification, used to build a ground truth camera */
struct CameraSpec {
  std::string model = "BrownConrady4"; // or "KannalaBrandt4"
  Vec2i resolution{752, 480};
  double fov = 90.0;      // Horizontal field of view [deg]
  Vec4 dist{0, 0, 0, 0};  // BrownConrady4: k1, k2, p1, p2. KB4: k1..k4
  double cx_offset = 0.0; // Principal point offset from image center [px]
  double cy_offset = 0.0;
  double fy_ratio = 1.0; // fy / fx
};

/**
 * Make a camera such that the horizontal image edge, through the principal
 * point, sees exactly `fov` degrees (distortion included).
 */
CameraGeometry make_camera(const CameraSpec &spec);

/**
 * Unit ray (camera frame) through image point `z`. Unlike the camera models'
 * `back_project()` this also works for rays 90 degrees or more off the optical
 * axis of fisheye lenses.
 */
Vec3 camera_ray(const CameraGeometry &camera, const Vec2 &z);

/**
 * Check the camera's projection is monotonic from the principal point to the
 * image corners, i.e. the distortion does not fold the image over.
 */
bool camera_is_monotonic(const CameraGeometry &camera);

/******************************************************************************
 * CAMERA POSES
 *****************************************************************************/

struct CameraPose {
  Mat4 T_TC; // Camera pose in the target frame
  double dist = 0.0;
  double tilt = 0.0; // [deg]
  double roll = 0.0; // [deg]
};

struct PoseSampling {
  unsigned int seed = 0;
  int num_poses = 0;
  double min_dist = 0.0; // Distance to board center [m]
  double max_dist = 0.0;
  double max_tilt = 0.0; // Angle between board normal and camera position [deg]
};

/** Sample camera poses T_TC looking at the board center from the front */
std::vector<CameraPose> sample_poses(const PoseSampling &sampling,
                                     const DetectorTarget &target);

/**
 * Sample camera poses T_TC with the board center placed at a random image
 * location, so that the board is observed across the entire image. Image
 * locations further than `max_theta` [deg] off the optical axis are avoided.
 */
std::vector<CameraPose> sample_poses_spread(const PoseSampling &sampling,
                                            const DetectorTarget &target,
                                            const CameraGeometry &camera,
                                            const double max_theta);

/******************************************************************************
 * RENDERER
 *****************************************************************************/

struct RenderParams {
  int supersample = 2;      // Anti-aliasing
  double margin = 0.1;      // White margin around board [m]
  double blur_sigma = 0.8;  // Gaussian blur [px]
  double noise_sigma = 2.0; // Gaussian noise [intensity levels]
};

/** Renders a planar target texture as seen by a camera */
class Renderer {
private:
  RenderParams params_;
  Vec2i resolution_;
  std::vector<Vec3> rays_; // Unit ray of every (super sampled) pixel
  int rays_w_ = 0;
  int rays_h_ = 0;

public:
  Renderer(const RenderParams &params, const CameraGeometry &camera);

  cv::Mat render(const cv::Mat &texture,
                 const Mat4 &T_TC,
                 std::mt19937 &gen) const;
};

} // namespace cartesian
