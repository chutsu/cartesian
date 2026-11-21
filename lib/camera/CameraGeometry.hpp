#pragma once
#include "CameraModel.hpp"

namespace cartesian {

// Forward declaration
struct CameraGeometry;
using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;

/** Camera Geometry */
struct CameraGeometry {
  int camera_id;                             // Camera id
  std::shared_ptr<CameraModel> camera_model; // Projection and distortion
  Vec2i resolution;                          // Image size [px]
  VecX intrinsic;                            // Parameters of the model
  VecX extrinsic;                            // Pose T_C0Ci (x y z qx qy qz qw)

  CameraGeometry() = delete;
  CameraGeometry(const int camera_id_,
                 const std::string &camera_model_,
                 const Vec2i &resolution_,
                 const VecX &intrinsic_,
                 const VecX &extrinsic_ = tf_vec());
  virtual ~CameraGeometry() = default;

  /** Set Extrinsic */
  void set_extrinsic(const Mat4 &transform);

  /** Project */
  int project(const Vec3 &p_C, Vec2 &z_hat) const;

  /** Project Jacobian */
  MatX project_jacobian(const Vec3 &p_C) const;

  /** Parameters Jacobian */
  MatX params_jacobian(const Vec3 &p_C) const;

  /** Back project */
  void back_project(const Vec2 &x, Vec3 &ray) const;

  /** Unit ray through pixel `x`, also for rays 90 deg or more off axis */
  Vec3 unit_ray(const Vec2 &x) const;

  /** Undistort */
  Vec2 undistort(const Vec2 &z) const;
};

/**
 * Difference between the estimated and ground truth projections [px], over the
 * image pixels within `max_theta` [deg] of the optical axis
 */
void model_error(const CameraGeometry &gt,
                 const CameraGeometry &est,
                 const double max_theta,
                 double &rmse,
                 double &max_err);

} // namespace cartesian
