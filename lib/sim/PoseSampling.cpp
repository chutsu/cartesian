#include "PoseSampling.hpp"

#include <random>

#include "../camera/CameraSpec.hpp"
#include "SimTarget.hpp"

namespace cartesian {

static Mat3 look_at(const Vec3 &z) {
  const Vec3 x = Vec3{0.0, -1.0, 0.0}.cross(z).normalized();
  const Vec3 y = z.cross(x);
  Mat3 C;
  C << x, y, z;
  return C;
}

std::vector<CameraPose> sample_poses(const PoseSampling &s,
                                     const double width,
                                     const double height) {
  std::mt19937 gen{s.seed};
  std::uniform_real_distribution<double> uni{0.0, 1.0};
  auto uniform = [&](const double a, const double b) {
    return a + (b - a) * uni(gen);
  };

  const Vec3 center{width / 2.0, height / 2.0, 0.0};
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
    const Vec3 aim =
        center +
        Vec3{uniform(-0.1, 0.1) * width, uniform(-0.1, 0.1) * height, 0.0};
    Mat3 C_TC = look_at((aim - r_TC).normalized());

    // Roll about the optical axis
    const double roll = deg2rad(uniform(-180.0, 180.0));
    C_TC = C_TC * rotz(roll);

    poses.push_back({tf(C_TC, r_TC), dist, rad2deg(tilt), rad2deg(roll)});
  }

  return poses;
}

std::vector<CameraPose> sample_poses_spread(const PoseSampling &s,
                                            const double width,
                                            const double height,
                                            const CameraGeometry &camera,
                                            const double max_theta) {
  std::mt19937 gen{s.seed};
  std::uniform_real_distribution<double> uni{0.0, 1.0};
  auto uniform = [&](const double a, const double b) {
    return a + (b - a) * uni(gen);
  };

  const Vec3 center{width / 2.0, height / 2.0, 0.0};
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

/** Check the whole board is seen by the camera, and within `max_theta` */
bool board_is_visible(const std::map<int, Vec3> &points,
                             const CameraPose &pose,
                             const CameraGeometry &camera,
                             const double max_theta) {
  const Mat4 T_CT = pose.T_TC.inverse();
  const double border = 5.0;
  for (const auto &[id, p_T] : points) {
    const Vec3 p_C = tf_point(T_CT, p_T);
    if (p_C.z() <= 0.0 || rad2deg(acos(p_C.z() / p_C.norm())) > max_theta) {
      return false;
    }
    Vec2 z;
    camera.project(p_C, z);
    if (z.x() < border || z.y() < border ||
        z.x() > camera.resolution.x() - border ||
        z.y() > camera.resolution.y() - border) {
      return false;
    }
  }
  return true;
}

/** Sample poses, keeping only those where the whole board is visible */
std::vector<CameraPose>
sample_poses_full_board(const PoseSampling &sampling,
                        const SimTarget &target,
                        const CameraGeometry &camera,
                        const double max_theta) {
  PoseSampling pool_sampling = sampling;
  pool_sampling.num_poses = 40 * sampling.num_poses;
  const auto pool =
      sample_poses_spread(
          pool_sampling, target.width, target.height, camera, max_theta);

  const auto points = target.corners;
  std::vector<CameraPose> poses;
  for (const auto &pose : pool) {
    if (board_is_visible(points, pose, camera, max_theta)) {
      poses.push_back(pose);
    }
    if (static_cast<int>(poses.size()) == sampling.num_poses) {
      break;
    }
  }
  return poses;
}

} // namespace cartesian
