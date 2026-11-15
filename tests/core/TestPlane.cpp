#include <gtest/gtest.h>
#include "core/Logger.hpp"
#include "core/Plane.hpp"

namespace cartesian {

/** Log a grid of points on a plane for Rerun visualization. */
void log_plane_grid(Logger &log,
                    const std::string &topic,
                    const Plane &plane,
                    const double size,
                    const int divisions,
                    const Vec3i &color) {
  // Compute plane center
  const Vec3 center = -plane.d * plane.normal;

  // Build two orthogonal basis vectors `u` and `v` spanning the plane
  Vec3 u = (std::abs(plane.normal.x()) < 0.9)
               ? plane.normal.cross(Vec3{1.0, 0.0, 0.0}).normalized()
               : plane.normal.cross(Vec3{0.0, 1.0, 0.0}).normalized();
  const Vec3 v = plane.normal.cross(u).normalized();

  const double step = size / divisions;
  const double half = size / 2.0;

  // Generate (divisions+1) x (divisions+1) grid of points
  std::vector<Vec3> points;
  std::vector<Vec3i> colors;
  std::vector<double> radii;
  for (int i = 0; i <= divisions; ++i) {
    for (int j = 0; j <= divisions; ++j) {
      points.push_back(center + u * (step * i - half) + v * (step * j - half));
      colors.push_back(color);
      radii.push_back(0.015);
    }
  }
  log.log_points(topic, points, colors, radii);
}

TEST(Plane, construct_from_normal_and_d) {
  const Vec3 normal = Vec3{0.0, 0.0, 1.0}.normalized();
  const double d = -2.0;
  Plane plane{normal, d};

  EXPECT_NEAR(plane.normal.x(), 0.0, 1e-12);
  EXPECT_NEAR(plane.normal.y(), 0.0, 1e-12);
  EXPECT_NEAR(plane.normal.z(), 1.0, 1e-12);
  EXPECT_NEAR(plane.d, -2.0, 1e-12);
}

TEST(Plane, construct_from_three_points) {
  const Vec3 p1{1.0, 1.0, 0.0};
  const Vec3 p2{2.0, 1.0, 0.0};
  const Vec3 p3{1.0, 2.0, 0.0};

  const Plane plane{p1, p2, p3};

  EXPECT_NEAR(plane.normal.x(), 0.0, 1e-12);
  EXPECT_NEAR(plane.normal.y(), 0.0, 1e-12);
  EXPECT_NEAR(plane.normal.z(), 1.0, 1e-12);
}

TEST(Plane, dist_zero_for_point_on_plane) {
  const Plane plane{Vec3{0.0, 0.0, 1.0}, 0.0};
  const double d = plane.dist(Vec3{5.0, -3.0, 0.0});
  EXPECT_NEAR(d, 0.0, 1e-12);
}

TEST(Plane, dist_positive_for_point_off_plane) {
  const Plane plane{Vec3{0.0, 0.0, 1.0}.normalized(), 0.0};
  const double d = plane.dist(Vec3{0.0, 0.0, 3.0});
  EXPECT_NEAR(d, 3.0, 1e-12);
}

TEST(Plane, ransac_fits_plane_to_noisy_data) {
  std::mt19937 rng(42);
  std::normal_distribution<double> noise{0.0, 0.05};

  std::vector<Vec3> pts;
  for (int i = 0; i < 100; ++i) {
    const double x = randf(-5.0, 5.0);
    const double y = randf(-5.0, 5.0);
    const double z = 2.0 * x - 3.0 * y + 1.0 + noise(rng);
    pts.emplace_back(x, y, z);
  }

  std::vector<int> inliers;
  const Plane plane = plane_ransac(pts, 200, 0.15, inliers);

  EXPECT_GT(inliers.size(), 50);
  EXPECT_NEAR(plane.normal.norm(), 1.0, 1e-12);

  // Debug-only Rerun visualization: log input points, classified
  // inliers/outliers, and the fitted plane grid.
  bool debug = false;
  if (debug) {
    Logger log("plane_refine");

    // Log all input points in gray
    {
      std::vector<Vec3i> colors(pts.size(), Vec3i{100, 100, 100});
      std::vector<double> radii(pts.size(), 0.04);
      log.log_points("ransac/input", 0, pts, colors, radii);
    }

    // Classify each point as inlier (green) or outlier (red)
    std::vector<bool> is_inlier(pts.size(), false);
    for (int idx : inliers) {
      is_inlier[idx] = true;
    }

    std::vector<Vec3i> colors;
    std::vector<double> radii;
    for (size_t i = 0; i < pts.size(); ++i) {
      colors.push_back(is_inlier[i] ? Vec3i{0, 255, 0} : Vec3i{255, 0, 0});
      radii.push_back(is_inlier[i] ? 0.06 : 0.03);
    }
    log.log_points("ransac/classified", 1, pts, colors, radii);

    // Log the fitted plane as a point grid
    log_plane_grid(log, "ransac/plane", plane, 14.0, 20, Vec3i{255, 0, 0});
  }
}

TEST(Plane, refine_fits_plane_to_inliers) {
  std::vector<Vec3> pts;
  for (int i = 0; i < 20; ++i) {
    const double x = randf(-5.0, 5.0);
    const double y = randf(-5.0, 5.0);
    const double z = 2.0 * x - 3.0 * y + 1.0;
    pts.emplace_back(x, y, z);
  }

  const Plane plane = plane_refine(pts);
  EXPECT_NEAR(plane.normal.norm(), 1.0, 1e-10);
  for (const auto &p : pts) {
    EXPECT_NEAR(plane.dist(p), 0.0, 1e-10);
  }

  // Log fitted plane when debugging
  bool debug = false;
  if (debug) {
    Logger log("plane_refine");
    log_plane_grid(log, "refine/plane", plane, 14.0, 20, Vec3i{255, 0, 0});
  }
}

} // namespace cartesian
