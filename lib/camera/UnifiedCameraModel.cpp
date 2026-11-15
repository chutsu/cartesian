#include "UnifiedCameraModel.hpp"
#include "BrownConrady4.hpp"

namespace cartesian {

/** Parameter indices */
enum { FX = 0, FY, CX, CY, XI, K1, K2, P1, P2, NUM_PARAMS };

/** Largest angle behind the image plane that can be projected */
static double ucm_fov_parameter(const double xi) {
  return (xi <= 1.0) ? xi : 1.0 / xi;
}

std::string UnifiedCameraModel::type() const { return "UnifiedCameraModel"; }

int UnifiedCameraModel::project(const Vec2i &res,
                                const VecX &params,
                                const Vec3 &p_C,
                                Vec2 &z_hat) const {
  assert(params.size() == NUM_PARAMS);
  const double xi = params(XI);
  const Vec4 dist_params = params.tail<4>();

  // Project to the unit sphere, and then to the normalized image plane
  const double d = p_C.norm();
  const double denom = p_C.z() + xi * d;
  if (p_C.z() <= -(ucm_fov_parameter(xi) * d) || denom < 1e-12) {
    z_hat = Vec2{0.0, 0.0};
    return -1;
  }
  const Vec2 p{p_C.x() / denom, p_C.y() / denom};

  // Distort, scale and center
  const Vec2 p_d = radtan4_distort(dist_params, p);
  z_hat.x() = params(FX) * p_d.x() + params(CX);
  z_hat.y() = params(FY) * p_d.y() + params(CY);

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());
  return (x_ok && y_ok) ? 0 : -1;
}

MatX UnifiedCameraModel::project_jacobian(const VecX &params,
                                          const Vec3 &p_C) const {
  assert(params.size() == NUM_PARAMS);
  const double xi = params(XI);
  const Vec4 dist_params = params.tail<4>();

  // Normalized point p = (x, y) / denom, denom = z + xi * |p_C|
  const double d = p_C.norm();
  const double denom = p_C.z() + xi * d;
  const Vec2 p{p_C.x() / denom, p_C.y() / denom};
  const Vec3 denom_jac{xi * p_C.x() / d,
                       xi * p_C.y() / d,
                       1.0 + xi * p_C.z() / d};

  Mat<2, 3> J_p;
  J_p.row(0) = (Vec3{1.0, 0.0, 0.0} - p.x() * denom_jac).transpose() / denom;
  J_p.row(1) = (Vec3{0.0, 1.0, 0.0} - p.y() * denom_jac).transpose() / denom;

  const Mat2 J_dist = radtan4_point_jacobian(dist_params, p);
  const Vec2 focal{params(FX), params(FY)};

  MatX J = focal.asDiagonal() * J_dist * J_p;
  return J;
}

MatX UnifiedCameraModel::params_jacobian(const VecX &params,
                                         const Vec3 &p_C) const {
  assert(params.size() == NUM_PARAMS);
  const double xi = params(XI);
  const Vec4 dist_params = params.tail<4>();

  const double d = p_C.norm();
  const double denom = p_C.z() + xi * d;
  const Vec2 p{p_C.x() / denom, p_C.y() / denom};
  const Vec2 p_d = radtan4_distort(dist_params, p);
  const Mat2 J_dist = radtan4_point_jacobian(dist_params, p);
  const Vec2 focal{params(FX), params(FY)};

  MatX J = zeros(2, NUM_PARAMS);
  J(0, FX) = p_d.x();
  J(1, FY) = p_d.y();
  J(0, CX) = 1.0;
  J(1, CY) = 1.0;

  // d p / d xi = -p * d / denom
  const Vec2 p_xi = -p * d / denom;
  J.col(XI) = focal.asDiagonal() * J_dist * p_xi;

  // Distortion parameters
  const MatX J_dist_params = radtan4_params_jacobian(dist_params, p);
  J.block(0, K1, 2, 4) = focal.asDiagonal() * J_dist_params;

  return J;
}

void UnifiedCameraModel::back_project(const VecX &params,
                                      const Vec2 &x,
                                      Vec3 &ray) const {
  assert(params.size() == NUM_PARAMS);
  const double xi = params(XI);
  const Vec4 dist_params = params.tail<4>();

  // Undistort to the normalized image plane
  const Vec2 p_d{(x.x() - params(CX)) / params(FX),
                 (x.y() - params(CY)) / params(FY)};
  const Vec2 p = radtan4_undistort(dist_params, p_d);

  // Lift to the unit sphere
  const double r2 = p.squaredNorm();
  const double s = sqrt(std::max(0.0, 1.0 + (1.0 - xi * xi) * r2));
  const double lambda = (xi + s) / (r2 + 1.0);
  ray = Vec3{lambda * p.x(), lambda * p.y(), lambda - xi};
}

Vec2 UnifiedCameraModel::undistort(const VecX &params, const Vec2 &z) const {
  Vec3 ray;
  back_project(params, z, ray);

  // Project the ray with a pinhole camera
  const double depth = std::max(ray.z(), 1e-9);
  return Vec2{params(FX) * ray.x() / depth + params(CX),
              params(FY) * ray.y() / depth + params(CY)};
}

} // namespace cartesian
