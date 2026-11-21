#pragma once
#include "ResidualBlock.hpp"
#include "imu/ImuGeometry.hpp"
#include "camera/CameraGeometry.hpp"
#include "calib/CalibTargetGeometry.hpp"

namespace cartesian {

class CalibCameraImuError;
using CalibCameraImuErrorPtr = std::shared_ptr<CalibCameraImuError>;

/** Reprojection error of a target point in a camera-imu calibration */
class CalibCameraImuError : public ResidualBlock {
public:
  /** STANDARD ignores the time delay, POSE_INTERP extrapolates the pose */
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
  /** Constructor, `z_k` is the measured pixel and `covar` its covariance */
  CalibCameraImuError(const Mode mode,
                      const timestamp_t ts_km1,
                      const timestamp_t ts_k,
                      const std::shared_ptr<CameraGeometry> &camera_geometry,
                      const std::vector<double *> &param_ptrs,
                      const std::vector<ParamBlock::Type> &param_types,
                      const Vec2 &z_k,
                      const Mat2 &covar);

  /** Create the error of target point `point_id`, measured at `z_k` */
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

  /** True if the point projected into the camera in the last evaluation */
  bool valid() const;

  /** Get the covariance of the pixel measurement */
  Mat2 get_covariance_matrix() const;

  /** Get the last reprojection error [px], false if the point is not valid */
  bool get_residuals(Vec2 &r) const;

  /** Get the norm of the reprojection error [px] of the last evaluation */
  bool get_reproj_error(double *error) const;

  /** Evaluate the error with the pose at the time of the view */
  bool eval_std(double const *const *params, double *res, double **jacs) const;

  /** Evaluate the error with the pose extrapolated by the time delay */
  bool eval_pi(double const *const *params, double *res, double **jacs) const;

  /** Evaluate the error of the mode, with minimal Jacobians */
  bool eval(double const *const *params,
            double *res,
            double **jacs) const override;
};

} // namespace cartesian
