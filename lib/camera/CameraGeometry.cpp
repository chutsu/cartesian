#include "CameraGeometry.hpp"
#include "Pinhole.hpp"
#include "BrownConrady4.hpp"
#include "KannalaBrandt4.hpp"
#include "UnifiedCameraModel.hpp"

namespace cartesian {

CameraGeometry::CameraGeometry(const int camera_id_,
                               const std::string &camera_model_str_,
                               const Vec2i &resolution_,
                               const VecX &intrinsic_,
                               const VecX &extrinsic_)
    : camera_id{camera_id_},
      resolution{resolution_}, intrinsic{intrinsic_}, extrinsic{extrinsic_} {
  // Initialize camera model
  int intrinsic_size = 0;
  if (camera_model_str_ == "BrownConrady4") {
    camera_model = std::make_shared<BrownConrady4>();
    intrinsic_size = 8;
  } else if (camera_model_str_ == "KannalaBrandt4") {
    camera_model = std::make_shared<KannalaBrandt4>();
    intrinsic_size = 8;
  } else if (camera_model_str_ == "UnifiedCameraModel") {
    camera_model = std::make_shared<UnifiedCameraModel>();
    intrinsic_size = 9;
  } else {
    FATAL("Unsupported camera model [%s]", camera_model_str_.c_str());
  }

  // Initialize camera intrinsic
  if (intrinsic.size() != 0) {
    return;
  }
  const double fx = pinhole_focal(resolution.x(), 90.0);
  const double fy = pinhole_focal(resolution.x(), 90.0);
  const double cx = resolution.x() / 2.0;
  const double cy = resolution.y() / 2.0;

  intrinsic.resize(intrinsic_size);
  intrinsic.setZero();
  intrinsic[0] = fx;
  intrinsic[1] = fy;
  intrinsic[2] = cx;
  intrinsic[3] = cy;
  if (camera_model_str_ == "UnifiedCameraModel") {
    // xi = 1 is a stereographic projection, which sees a pinhole camera with a
    // focal length of fx / (1 + xi) around the optical axis
    const double xi = 1.0;
    intrinsic[0] = (1.0 + xi) * fx;
    intrinsic[1] = (1.0 + xi) * fy;
    intrinsic[4] = xi;
  }
}

void CameraGeometry::set_extrinsic(const Mat4 &transform) {
  const Vec7 data = tf_vec(transform);
  for (int i = 0; i < 7; ++i) {
    extrinsic(i) = data(i);
  }
}

int CameraGeometry::project(const Vec3 &p_C, Vec2 &z_hat) const {
  return camera_model->project(resolution, intrinsic, p_C, z_hat);
}

MatX CameraGeometry::project_jacobian(const Vec3 &p_C) const {
  return camera_model->project_jacobian(intrinsic, p_C);
}

MatX CameraGeometry::params_jacobian(const Vec3 &p_C) const {
  return camera_model->params_jacobian(intrinsic, p_C);
}

void CameraGeometry::back_project(const Vec2 &x, Vec3 &ray) const {
  camera_model->back_project(intrinsic, x, ray);
}

Vec3 CameraGeometry::unit_ray(const Vec2 &x) const {
  Vec3 ray;
  back_project(x, ray);
  return ray.normalized();
}

Vec2 CameraGeometry::undistort(const Vec2 &z) const {
  return camera_model->undistort(intrinsic, z);
}

/** Difference between the estimated and ground truth projections [px] */
void model_error(const CameraGeometry &gt,
                 const CameraGeometry &est,
                 const double max_theta,
                 double &rmse,
                 double &max_err) {
  double sse = 0.0;
  int n = 0;
  max_err = 0.0;
  for (double v = 0.0; v < gt.resolution.y(); v += 8.0) {
    for (double u = 0.0; u < gt.resolution.x(); u += 8.0) {
      const Vec2 z{u, v};
      const Vec3 ray = gt.unit_ray(z);
      if (rad2deg(acos(ray.z())) > max_theta) {
        continue;
      }

      Vec2 z_est;
      est.project(ray, z_est);
      const double err = (z_est - z).norm();
      if (std::isfinite(err) == false) {
        rmse = NAN;
        max_err = NAN;
        return;
      }
      sse += err * err;
      max_err = std::max(max_err, err);
      n++;
    }
  }
  rmse = (n > 0) ? sqrt(sse / n) : NAN;
}

} // namespace cartesian
