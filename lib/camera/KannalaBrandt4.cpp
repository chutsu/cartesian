#include "KannalaBrandt4.hpp"

namespace cartesian {

///////////////////////////////////////////////////////////////////////////////
// EQUIDISTANT DISTORTION                                                    //
///////////////////////////////////////////////////////////////////////////////

Vec2 equi4_distort(const Vec4 &dist_params, const Vec2 &p) {
  const double r = p.norm();
  if (r < 1e-8) {
    return p;
  }

  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  // Apply equi distortion
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double x_dash = (thd / r) * p(0);
  const double y_dash = (thd / r) * p(1);

  return Vec2{x_dash, y_dash};
}

Vec2 equi4_undistort(const Vec4 &dist_params, const Vec2 &p) {
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  const double thd = sqrt(p(0) * p(0) + p(1) * p(1));
  double th = thd; // Initial guess
  for (int i = 20; i > 0; i--) {
    const double th2 = th * th;
    const double th4 = th2 * th2;
    const double th6 = th4 * th2;
    const double th8 = th4 * th4;
    th = thd / (1 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  }

  const double scaling = tan(th) / thd;
  return Vec2{p(0) * scaling, p(1) * scaling};
}

Mat2 equi4_point_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  const double k1 = dist_params(0);
  const double k2 = dist_params(1);
  const double k3 = dist_params(2);
  const double k4 = dist_params(3);

  const double x = p(0);
  const double y = p(1);
  const double r = p.norm();
  const double th = atan(r);
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  const double thd = th * (1.0 + k1 * th2 + k2 * th4 + k3 * th6 + k4 * th8);
  const double s = thd / r;

  // Form jacobian
  const double th_r = 1.0 / (r * r + 1.0);
  double thd_th = 1.0 + 3.0 * k1 * th2;
  thd_th += 5.0 * k2 * th4;
  thd_th += 7.0 * k3 * th6;
  thd_th += 9.0 * k4 * th8;
  const double s_r = thd_th * th_r / r - thd / (r * r);
  const double r_x = 1.0 / r * x;
  const double r_y = 1.0 / r * y;

  Mat2 J_point = eye(2);
  J_point(0, 0) = s + x * s_r * r_x;
  J_point(0, 1) = x * s_r * r_y;
  J_point(1, 0) = y * s_r * r_x;
  J_point(1, 1) = s + y * s_r * r_y;

  return J_point;
}

MatX equi4_params_jacobian(const Vec4 &dist_params, const Vec2 &p) {
  UNUSED(dist_params);

  const double x = p(0);
  const double y = p(1);
  const double r = p.norm();
  const double th = atan(r);

  const double th3 = th * th * th;
  const double th5 = th3 * th * th;
  const double th7 = th5 * th * th;
  const double th9 = th7 * th * th;

  MatX J_dist = zeros(2, 4);
  J_dist(0, 0) = x * th3 / r;
  J_dist(0, 1) = x * th5 / r;
  J_dist(0, 2) = x * th7 / r;
  J_dist(0, 3) = x * th9 / r;

  J_dist(1, 0) = y * th3 / r;
  J_dist(1, 1) = y * th5 / r;
  J_dist(1, 2) = y * th7 / r;
  J_dist(1, 3) = y * th9 / r;

  return J_dist;
}

///////////////////////////////////////////////////////////////////////////////
// KANNALA-BRANDT4 CAMERA MODEL                                              //
///////////////////////////////////////////////////////////////////////////////

std::string KannalaBrandt4::type() const {
  std::string typestr = "KannalaBrandt4";
  return typestr;
}

/** Evaluate th_d = th * (1 + k1 th^2 + ... + k4 th^8) and its derivative */
static void equi4_theta(const Vec4 &k,
                        const double th,
                        double &thd,
                        double &thd_th) {
  const double th2 = th * th;
  const double th4 = th2 * th2;
  const double th6 = th4 * th2;
  const double th8 = th4 * th4;
  thd = th * (1.0 + k(0) * th2 + k(1) * th4 + k(2) * th6 + k(3) * th8);
  thd_th = 1.0 + 3.0 * k(0) * th2 + 5.0 * k(1) * th4 + 7.0 * k(2) * th6 +
           9.0 * k(3) * th8;
}

int KannalaBrandt4::project(const Vec2i &res,
                            const VecX &params,
                            const Vec3 &p_C,
                            Vec2 &z_hat) const {
  const Vec4 k = params.tail(4);
  const double x = p_C.x();
  const double y = p_C.y();
  const double z = p_C.z();
  const double r = sqrt(x * x + y * y);

  // Scale s such that the distorted point is s * (x, y), computed from the
  // angle off the optical axis so that rays 90 degrees or more are supported
  double s = 0.0;
  if (r < 1e-8) {
    if (z <= 0.0) {
      z_hat = Vec2{0.0, 0.0};
      return -1; // Ray along the negative optical axis
    }
    s = 1.0 / z;
  } else {
    const double th = atan2(r, z);
    double thd, thd_th;
    equi4_theta(k, th, thd, thd_th);
    s = thd / r;
  }

  z_hat.x() = params(0) * s * x + params(2);
  z_hat.y() = params(1) * s * y + params(3);

  // Check projection is within image frame
  const bool x_ok = (z_hat.x() >= 0 && z_hat.x() < res.x());
  const bool y_ok = (z_hat.y() >= 0 && z_hat.y() < res.y());
  return (x_ok && y_ok) ? 0 : -1;
}

MatX KannalaBrandt4::project_jacobian(const VecX &params,
                                      const Vec3 &p_C) const {
  const Vec4 k = params.tail(4);
  const double x = p_C.x();
  const double y = p_C.y();
  const double z = p_C.z();
  const double r = sqrt(x * x + y * y);

  // Jacobian of the distorted point p_d = s * (x, y) w.r.t. p_C
  Mat<2, 3> J_pd = zeros(2, 3);
  if (r < 1e-8) {
    J_pd(0, 0) = 1.0 / z;
    J_pd(1, 1) = 1.0 / z;
    J_pd(0, 2) = -x / (z * z);
    J_pd(1, 2) = -y / (z * z);
  } else {
    const double th = atan2(r, z);
    double thd, thd_th;
    equi4_theta(k, th, thd, thd_th);
    const double s = thd / r;

    // th = atan2(r, z), r = |(x, y)|
    const double q = r * r + z * z;
    const Vec3 th_p{z * x / (r * q), z * y / (r * q), -r / q};
    const Vec3 r_p{x / r, y / r, 0.0};
    const Vec3 s_p = thd_th * th_p / r - thd / (r * r) * r_p;

    J_pd.block<1, 3>(0, 0) = x * s_p.transpose();
    J_pd.block<1, 3>(1, 0) = y * s_p.transpose();
    J_pd(0, 0) += s;
    J_pd(1, 1) += s;
  }

  const Vec2 focal{params(0), params(1)};
  MatX J = focal.asDiagonal() * J_pd;
  return J;
}

MatX KannalaBrandt4::params_jacobian(const VecX &params,
                                     const Vec3 &p_C) const {
  const Vec4 k = params.tail(4);
  const double x = p_C.x();
  const double y = p_C.y();
  const double z = p_C.z();
  const double r = sqrt(x * x + y * y);

  double s = 0.0;
  Vec4 th_pow = Vec4::Zero(); // th^3, th^5, th^7, th^9
  if (r < 1e-8) {
    s = 1.0 / z;
  } else {
    const double th = atan2(r, z);
    double thd, thd_th;
    equi4_theta(k, th, thd, thd_th);
    s = thd / r;
    th_pow << pow(th, 3), pow(th, 5), pow(th, 7), pow(th, 9);
  }

  MatX J = zeros(2, 8);
  J(0, 0) = s * x;
  J(1, 1) = s * y;
  J(0, 2) = 1.0;
  J(1, 3) = 1.0;
  if (r >= 1e-8) {
    for (int i = 0; i < 4; i++) {
      J(0, 4 + i) = params(0) * x * th_pow(i) / r;
      J(1, 4 + i) = params(1) * y * th_pow(i) / r;
    }
  }

  return J;
}

void KannalaBrandt4::back_project(const VecX &params,
                                  const Vec2 &x,
                                  Vec3 &ray) const {
  const Vec4 k = params.tail(4);
  const Vec2 p_d{(x.x() - params(2)) / params(0),
                 (x.y() - params(3)) / params(1)};
  const double thd = p_d.norm();
  if (thd < 1e-12) {
    ray = Vec3{0.0, 0.0, 1.0};
    return;
  }

  // Invert th_d = th * (1 + k1 th^2 + ... + k4 th^8) with Newton's method
  double th = thd;
  for (int i = 0; i < 30; i++) {
    double f, df;
    equi4_theta(k, th, f, df);
    if (fabs(df) < 1e-12) {
      break;
    }
    const double step = (f - thd) / df;
    th -= step;
    if (fabs(step) < 1e-14) {
      break;
    }
  }

  // Unit ray, which can point 90 degrees or more off the optical axis
  const double scale = sin(th) / thd;
  ray = Vec3{scale * p_d.x(), scale * p_d.y(), cos(th)};
}

Vec2 KannalaBrandt4::undistort(const VecX &params, const Vec2 &z) const {
  // Back-project and undistort
  const double fx = params(0);
  const double fy = params(1);
  const double cx = params(2);
  const double cy = params(3);
  const double px = (z.x() - cx) / fx;
  const double py = (z.y() - cy) / fy;
  const Vec2 p{px, py};

  const VecX dist_params = params.tail(params.size() - 4);
  const Vec2 p_undist = equi4_undistort(dist_params, p);

  // Project undistorted point to image plane
  const double x = p_undist.x() * fx + cx;
  const double y = p_undist.y() * fy + cy;
  const Vec2 z_undist = {x, y};

  return z_undist;
}
} // namespace cartesian
