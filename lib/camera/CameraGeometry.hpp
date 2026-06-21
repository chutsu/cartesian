#pragma once
#include "CameraModel.hpp"

namespace cartesian {

// Forward declaration
struct CameraGeometry;
using CameraGeometryPtr = std::shared_ptr<CameraGeometry>;

/** Camera Geometry */
struct CameraGeometry {
  int camera_id;
  std::shared_ptr<CameraModel> camera_model;
  Vec2i resolution;
  VecX intrinsic;
  VecX extrinsic;

  CameraGeometry() = delete;
  CameraGeometry(const int camera_id_,
                 const std::string &camera_model_,
                 const Vec2i &resolution_,
                 const VecX &intrinsic_,
                 const VecX &extrinsic_);
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

  /** Undistort */
  Vec2 undistort(const Vec2 &z) const;
};

} // namespace cartesian
