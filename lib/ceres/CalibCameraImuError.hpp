#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuGeometry.hpp"
#include "camera/CameraGeometry.hpp"
#include "calib/CalibTargetGeometry.hpp"

namespace cartesian {

class CalibCameraImuError;
using CalibCameraImuErrorPtr = std::shared_ptr<CalibCameraImuError>;

class CalibCameraImuError : public ResidualBlock {
public:
  enum Mode { NOT_SET, STANDARD, POSE_INTERP };

private:
  Mode mode_;
  timestamp_t ts_km1_;
  timestamp_t ts_k_;
  std::shared_ptr<CameraGeometry> camera_geometry_;
  Vec2 z_k_;
  Mat2 covar_;
  Mat2 info_;
  Mat2 sqrt_info_;
  mutable bool valid_ = true;
  mutable Vec2 residuals_;

public:
  CalibCameraImuError(const Mode mode,
                      const timestamp_t ts_km1,
                      const timestamp_t ts_k,
                      const std::shared_ptr<CameraGeometry> &camera_geometry,
                      const std::vector<double *> &param_ptrs,
                      const std::vector<ParamBlock::Type> &param_types,
                      const Vec2 &z_k,
                      const Mat2 &covar);

  static CalibCameraImuErrorPtr
  create(CalibCameraImuError::Mode mode,
         const timestamp_t ts_km1,
         const timestamp_t ts_k,
         const std::shared_ptr<CameraGeometry> &camera,
         const std::shared_ptr<ImuGeometry> &imu,
         const std::shared_ptr<CalibTargetGeometry> &target,
         double *sensor_pose_km1,
         double *sensor_pose_k,
         double *target_pose,
         double *time_delay,
         const int point_id,
         const Vec2 &z_k,
         const Mat2 &covar = eye(2));

  bool valid() const;
  Mat2 get_covariance_matrix() const;
  bool get_residuals(Vec2 &r) const;
  bool get_reproj_error(double *error) const;

  bool eval_std(double const *const *params, double *res, double **jacs) const;
  bool eval_pi(double const *const *params, double *res, double **jacs) const;

  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

} // namespace cartesian
