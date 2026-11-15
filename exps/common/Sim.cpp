#include "Sim.hpp"

#include <opencv2/imgproc.hpp>

namespace cartesian {

/******************************************************************************
 * CAMERA
 *****************************************************************************/

static bool is_fisheye(const CameraGeometry &camera) {
  return camera.camera_model->type() == "KannalaBrandt4";
}

/** Radial distortion `thd(th)` of the Kannala-Brandt model */
static double equi_thd(const Vec4 &k, const double th) {
  const double th2 = th * th;
  return th * (1 + k(0) * th2 + k(1) * th2 * th2 + k(2) * th2 * th2 * th2 +
               k(3) * th2 * th2 * th2 * th2);
}

/** Radial distortion of the radial-tangential model along the x-axis */
static double radtan_rd(const Vec4 &d, const double r) {
  const double r2 = r * r;
  return r * (1 + d(0) * r2 + d(1) * r2 * r2);
}

CameraGeometry make_camera(const CameraSpec &spec) {
  const double w = spec.resolution.x();
  const double h = spec.resolution.y();
  const double half_fov = deg2rad(spec.fov / 2.0);

  // Normalized radius of the horizontal image edge, including distortion
  double edge = 0.0;
  if (spec.model == "KannalaBrandt4") {
    edge = equi_thd(spec.dist, half_fov);
  } else {
    edge = radtan_rd(spec.dist, tan(half_fov));
  }

  // The edge of the image (w / 2 px) sees half the field of view
  const double fx = (w / 2.0) / edge;
  const double fy = fx * spec.fy_ratio;
  const double cx = w / 2.0 + spec.cx_offset;
  const double cy = h / 2.0 + spec.cy_offset;

  VecX intrinsic = zeros(8, 1);
  intrinsic << fx, fy, cx, cy, spec.dist;
  return CameraGeometry{0, spec.model, spec.resolution, intrinsic};
}

Vec3 camera_ray(const CameraGeometry &camera, const Vec2 &z) {
  if (is_fisheye(camera) == false) {
    Vec3 ray;
    camera.back_project(z, ray);
    return ray.normalized();
  }

  // Invert th_d = th * (1 + k1 th^2 + ... + k4 th^8) with Newton's method
  const VecX &K = camera.intrinsic;
  const Vec4 k = K.tail(4);
  const Vec2 p{(z.x() - K(2)) / K(0), (z.y() - K(3)) / K(1)};
  const double thd = p.norm();
  if (thd < 1e-12) {
    return Vec3{0.0, 0.0, 1.0};
  }

  double th = thd;
  for (int i = 0; i < 30; ++i) {
    const double th2 = th * th;
    const double f = equi_thd(k, th) - thd;
    const double df = 1 + 3 * k(0) * th2 + 5 * k(1) * th2 * th2 +
                      7 * k(2) * th2 * th2 * th2 +
                      9 * k(3) * th2 * th2 * th2 * th2;
    th -= f / df;
  }
  const Vec2 dir = p / thd;
  return Vec3{sin(th) * dir.x(), sin(th) * dir.y(), cos(th)};
}

bool camera_is_monotonic(const CameraGeometry &camera) {
  // Walk along the image diagonal, away from the principal point, and make
  // sure the angle off the optical axis keeps increasing
  const VecX &K = camera.intrinsic;
  const Vec2 c{K(2), K(3)};
  const Vec2 corner{camera.resolution.x() * 1.0, camera.resolution.y() * 1.0};
  const Vec2 dir = (corner - c).normalized();
  const double max_r = (corner - c).norm();

  double prev_theta = -1.0;
  for (double r = 0.0; r <= max_r; r += 1.0) {
    const Vec3 ray = camera_ray(camera, c + r * dir);
    const double theta = acos(std::min(1.0, std::max(-1.0, ray.z())));
    if (theta < prev_theta - 1e-9) {
      return false;
    }
    prev_theta = theta;
  }
  return true;
}

/******************************************************************************
 * CAMERA POSES
 *****************************************************************************/

static Mat3 look_at(const Vec3 &z) {
  const Vec3 x = Vec3{0.0, -1.0, 0.0}.cross(z).normalized();
  const Vec3 y = z.cross(x);
  Mat3 C;
  C << x, y, z;
  return C;
}

std::vector<CameraPose> sample_poses(const PoseSampling &s,
                                     const DetectorTarget &target) {
  std::mt19937 gen{s.seed};
  std::uniform_real_distribution<double> uni{0.0, 1.0};
  auto uniform = [&](const double a, const double b) {
    return a + (b - a) * uni(gen);
  };

  const Vec3 center{target.width() / 2.0, target.height() / 2.0, 0.0};
  std::vector<CameraPose> poses;
  for (int i = 0; i < s.num_poses; ++i) {
    // Position on a sphere around the board center
    const double dist = uniform(s.min_dist, s.max_dist);
    const double tilt = deg2rad(uniform(0.0, s.max_tilt));
    const double azimuth = uniform(0.0, 2.0 * M_PI);
    const Vec3 dir{sin(tilt) * cos(azimuth),
                   sin(tilt) * sin(azimuth),
                   cos(tilt)};
    const Vec3 r_TC = center + dist * dir;

    // Look at a point near the board center
    const Vec3 aim = center + Vec3{uniform(-0.1, 0.1) * target.width(),
                                   uniform(-0.1, 0.1) * target.height(),
                                   0.0};
    Mat3 C_TC = look_at((aim - r_TC).normalized());

    // Roll about the optical axis
    const double roll = deg2rad(uniform(-180.0, 180.0));
    C_TC = C_TC * rotz(roll);

    poses.push_back({tf(C_TC, r_TC), dist, rad2deg(tilt), rad2deg(roll)});
  }

  return poses;
}

std::vector<CameraPose> sample_poses_spread(const PoseSampling &s,
                                            const DetectorTarget &target,
                                            const CameraGeometry &camera,
                                            const double max_theta) {
  std::mt19937 gen{s.seed};
  std::uniform_real_distribution<double> uni{0.0, 1.0};
  auto uniform = [&](const double a, const double b) {
    return a + (b - a) * uni(gen);
  };

  const Vec3 center{target.width() / 2.0, target.height() / 2.0, 0.0};
  const Vec3 e_z{0.0, 0.0, 1.0};
  std::vector<CameraPose> poses;
  for (int i = 0; i < s.num_poses; ++i) {
    // Position on a sphere around the board center
    const double dist = uniform(s.min_dist, s.max_dist);
    const double tilt = deg2rad(uniform(0.0, s.max_tilt));
    const double azimuth = uniform(0.0, 2.0 * M_PI);
    const Vec3 dir{sin(tilt) * cos(azimuth),
                   sin(tilt) * sin(azimuth),
                   cos(tilt)};
    const Vec3 r_TC = center + dist * dir;

    // Pick the image location of the board center
    Vec3 ray;
    do {
      const Vec2 z{uniform(0.0, camera.resolution.x() - 1.0),
                   uniform(0.0, camera.resolution.y() - 1.0)};
      ray = camera_ray(camera, z);
    } while (rad2deg(acos(ray.z())) > max_theta);

    // Orient the camera such that the board center lies along `ray`:
    // C_TC * ray = direction to the board center (in the target frame)
    const Mat3 C_look = look_at((center - r_TC).normalized());
    const Mat3 Q =
        Eigen::Quaterniond::FromTwoVectors(ray, e_z).toRotationMatrix();
    const double roll = deg2rad(uniform(-180.0, 180.0));
    const Mat3 C_TC = C_look * rotz(roll) * Q;

    poses.push_back({tf(C_TC, r_TC), dist, rad2deg(tilt), rad2deg(roll)});
  }

  return poses;
}

/******************************************************************************
 * RENDERER
 *****************************************************************************/

Renderer::Renderer(const RenderParams &params, const CameraGeometry &camera)
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
        rays_[v * rays_w_ + u] = camera_ray(camera, z);
      }
    }
  });
}

cv::Mat Renderer::render(const cv::Mat &texture,
                         const Mat4 &T_TC,
                         std::mt19937 &gen) const {
  const Mat3 C_TC = T_TC.block<3, 3>(0, 0);
  const Vec3 r_TC = T_TC.block<3, 1>(0, 3);
  const double ppm = DetectorTarget::PPM;
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
