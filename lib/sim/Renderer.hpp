#pragma once
#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "../camera/CameraGeometry.hpp"
#include "../core/Core.hpp"

namespace cartesian {

/******************************************************************************
 * RENDERER
 *****************************************************************************/

struct RenderParams {
  int supersample = 2;      // Anti-aliasing
  double margin = 0.1;      // White margin around board [m]
  double blur_sigma = 0.8;  // Gaussian blur [px]
  double noise_sigma = 2.0; // Gaussian noise [intensity levels]
  double ppm = 1000.0;      // Texture resolution [pixels per meter]
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
