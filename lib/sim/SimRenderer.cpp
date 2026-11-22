#include "SimRenderer.hpp"

#include <opencv2/imgproc.hpp>



namespace cartesian {

/******************************************************************************
 * RENDERER
 *****************************************************************************/

SimRenderer::SimRenderer(const SimRenderer::RenderParams &params, const CameraGeometry &camera)
    : params_{params}, resolution_{camera.resolution} {
  // Pre-compute rays of every pixel
  const int k = params_.supersample;
  rays_w_ = resolution_.x() * k;
  rays_h_ = resolution_.y() * k;
  rays_.resize(rays_w_ * rays_h_);

  cv::parallel_for_(cv::Range{0, rays_h_}, [&](const cv::Range &rows) {
    for (int v = rows.start; v < rows.end; ++v) {
      for (int u = 0; u < rays_w_; ++u) {
        // Pixel center of the super sampled pixel in the image (the center of
        // pixel (0, 0) is at (0, 0))
        const Vec2 z{(u + 0.5) / k - 0.5, (v + 0.5) / k - 0.5};
        rays_[v * rays_w_ + u] = camera.unit_ray(z);
      }
    }
  });
}

cv::Mat SimRenderer::render(const cv::Mat &texture,
                         const Mat4 &T_TC,
                         std::mt19937 &gen) const {
  const Mat3 C_TC = T_TC.block<3, 3>(0, 0);
  const Vec3 r_TC = T_TC.block<3, 1>(0, 3);
  const double ppm = params_.ppm;
  const double texture_h = texture.rows;
  const double margin = params_.margin;

  // Ray cast pixels onto the target plane (z = 0) to find texture lookup
  cv::Mat map_x(rays_h_, rays_w_, CV_32FC1);
  cv::Mat map_y(rays_h_, rays_w_, CV_32FC1);
  cv::parallel_for_(cv::Range{0, rays_h_}, [&](const cv::Range &rows) {
    for (int v = rows.start; v < rows.end; ++v) {
      for (int u = 0; u < rays_w_; ++u) {
        const Vec3 ray = C_TC * rays_[v * rays_w_ + u];
        const double s = -r_TC.z() / ray.z();
        float mx = -1.0f;
        float my = -1.0f;
        if (s > 0.0) {
          const double x = r_TC.x() + s * ray.x();
          const double y = r_TC.y() + s * ray.y();
          // Meters to texture pixels (board frame has y up, image y down)
          mx = (x + margin) * ppm - 0.5;
          my = texture_h - (y + margin) * ppm - 0.5;
        }
        map_x.at<float>(v, u) = mx;
        map_y.at<float>(v, u) = my;
      }
    }
  });

  // Sample texture (gray outside of the texture)
  cv::Mat image;
  cv::remap(texture,
            image,
            map_x,
            map_y,
            cv::INTER_LINEAR,
            cv::BORDER_CONSTANT,
            cv::Scalar{128});

  // Anti-alias, blur and add noise
  cv::resize(image,
             image,
             cv::Size{resolution_.x(), resolution_.y()},
             0,
             0,
             cv::INTER_AREA);
  if (params_.blur_sigma > 0.0) {
    cv::GaussianBlur(image, image, cv::Size{}, params_.blur_sigma);
  }
  if (params_.noise_sigma > 0.0) {
    std::normal_distribution<double> noise{0.0, params_.noise_sigma};
    for (int v = 0; v < image.rows; ++v) {
      for (int u = 0; u < image.cols; ++u) {
        const double p = image.at<uchar>(v, u) + noise(gen);
        image.at<uchar>(v, u) = cv::saturate_cast<uchar>(p);
      }
    }
  }

  return image;
}

} // namespace cartesian
