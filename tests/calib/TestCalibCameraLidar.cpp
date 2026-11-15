#include <fstream>
#include <gtest/gtest.h>

// PCL core
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

// Segmentation
#include <pcl/segmentation/sac_segmentation.h>

// Types
using PointT = pcl::PointXYZI;

#include "core/core.hpp"

namespace cartesian {

static std::vector<std::array<float, 4>> load_pcd_points(const fs::path &path) {
  std::ifstream file(path, std::ios::binary);
  assert(file.is_open());

  // Parse ASCII header
  std::string line;
  int num_points = 0;
  while (std::getline(file, line)) {
    if (line.rfind("POINTS ", 0) == 0) {
      num_points = std::stoi(line.substr(7));
    } else if (line.rfind("DATA ", 0) == 0) {
      break;
    }
  }

  // Read binary data: each point is 4 consecutive float32 (x, y, z, intensity)
  std::vector<std::array<float, 4>> points(num_points);
  file.read(reinterpret_cast<char *>(points.data()),
            num_points * 4 * sizeof(float));

  return points;
}

void lidar_detect_target(const pcl::PointCloud<PointT>::Ptr &pcl_in,
                         Logger &log) {
  // ---------------------------------------------------
  // 1. Intensity filter (keep high-reflectivity points)
  // ---------------------------------------------------
  const double intensity_threshold = 100.0;
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  for (const auto &pt : *pcl_in) {
    if (pt.intensity >= intensity_threshold) {
      cloud_filtered->push_back(pt);
    }
  }

  // Log high-intensity points
  {
    std::vector<Vec3> hi_points;
    std::vector<Vec3i> hi_colors;
    std::vector<double> hi_radii;
    double max_intensity = 0;
    for (const auto &pt : *cloud_filtered) {
      if (pt.intensity > max_intensity)
        max_intensity = pt.intensity;
    }
    for (const auto &pt : *cloud_filtered) {
      hi_points.emplace_back(pt.x, pt.y, pt.z);
      hi_colors.push_back(colormap::jet(pt.intensity / max_intensity));
      hi_radii.emplace_back(0.008);
    }
    log.log_points("/lidar/high_intensity", 0, hi_points, hi_colors, hi_radii);
  }

  // Sequential RANSAC: detect multiple planes
  const double plane_dist_thresh = 0.1;
  const size_t min_plane_inliers = 100;
  const int max_planes = 5;

  pcl::PointCloud<PointT>::Ptr remaining(new pcl::PointCloud<PointT>);
  *remaining = *cloud_filtered;

  std::vector<Vec3i> palette = {
      {100, 100, 255},
      {255, 100, 100},
      {100, 255, 100},
      {255, 255, 100},
      {255, 100, 255},
      {100, 255, 255},
  };

  for (int plane_id = 0; plane_id < max_planes; ++plane_id) {
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_dist_thresh);
    seg.setMaxIterations(1000);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    seg.setInputCloud(remaining);
    seg.segment(*inliers, *coeffs);

    if (inliers->indices.empty() ||
        inliers->indices.size() < min_plane_inliers) {
      if (plane_id == 0)
        std::cerr << "No plane found!" << std::endl;
      break;
    }

    // Extract plane points
    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    // Log plane coefficients
    Vec3 plane_normal =
        Vec3(coeffs->values[0], coeffs->values[1], coeffs->values[2])
            .normalized();

    // Render plane as rectangular surface
    {
      Vec3 ref =
          (std::abs(plane_normal.x()) < 0.9) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
      Vec3 u = plane_normal.cross(ref).normalized();
      Vec3 v = plane_normal.cross(u).normalized();

      Vec3 centroid(0, 0, 0);
      for (const auto &pt : *plane_cloud)
        centroid += Vec3(pt.x, pt.y, pt.z);
      centroid /= plane_cloud->size();

      double u_min = 0, u_max = 0, v_min = 0, v_max = 0;
      bool first = true;
      for (const auto &pt : *plane_cloud) {
        Vec3 p(pt.x, pt.y, pt.z);
        Vec3 d = p - centroid;
        double du = d.dot(u), dv = d.dot(v);
        if (first) {
          u_min = u_max = du;
          v_min = v_max = dv;
          first = false;
        } else {
          u_min = std::min(u_min, du);
          u_max = std::max(u_max, du);
          v_min = std::min(v_min, dv);
          v_max = std::max(v_max, dv);
        }
      }

      const int grid_res = 20;
      std::vector<Vec3> plane_grid;
      for (int i = 0; i <= grid_res; ++i)
        for (int j = 0; j <= grid_res; ++j) {
          double fu = u_min + (u_max - u_min) * i / grid_res;
          double fv = v_min + (v_max - v_min) * j / grid_res;
          plane_grid.push_back(centroid + fu * u + fv * v);
        }
      Vec3i color = palette[plane_id % palette.size()];
      std::string prefix = "/lidar/plane_" + std::to_string(plane_id);
      log.log_points(prefix + "/surface",
                     0,
                     plane_grid,
                     std::vector<Vec3i>(plane_grid.size(), color),
                     std::vector<double>(plane_grid.size(), 0.01));

      std::vector<Vec3> boundary = {
          centroid + u_min * u + v_min * v,
          centroid + u_max * u + v_min * v,
          centroid + u_max * u + v_max * v,
          centroid + u_min * u + v_max * v,
          centroid + u_min * u + v_min * v,
      };
      log.log_line(prefix + "/boundary", boundary, Vec3i{255, 255, 0}, 0.02);
    }

    // Log plane inlier points
    {
      std::vector<Vec3> pts;
      std::vector<Vec3i> colors;
      std::vector<double> radii;
      for (const auto &pt : *plane_cloud) {
        pts.emplace_back(pt.x, pt.y, pt.z);
        colors.push_back(colormap::jet(0.8));
        radii.emplace_back(0.01);
      }
      std::string prefix = "/lidar/plane_" + std::to_string(plane_id);
      log.log_points(prefix + "/inliers", 0, pts, colors, radii);
    }

    // Circle detection on this plane
    {
      pcl::SACSegmentation<PointT> seg_circle;
      seg_circle.setOptimizeCoefficients(true);
      seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
      seg_circle.setMethodType(pcl::SAC_RANSAC);
      seg_circle.setDistanceThreshold(0.1);
      seg_circle.setMaxIterations(1000);
      seg_circle.setRadiusLimits(0.5, 2.0);

      pcl::PointIndices::Ptr circle_inliers(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr circle_coeffs(new pcl::ModelCoefficients);
      seg_circle.setInputCloud(plane_cloud);
      seg_circle.segment(*circle_inliers, *circle_coeffs);

      if (!circle_inliers->indices.empty()) {
        // Extract and log circle inlier points
        pcl::ExtractIndices<PointT> extract_circle;
        pcl::PointCloud<PointT>::Ptr circle_cloud(new pcl::PointCloud<PointT>);
        extract_circle.setInputCloud(plane_cloud);
        extract_circle.setIndices(circle_inliers);
        extract_circle.setNegative(false);
        extract_circle.filter(*circle_cloud);

        std::vector<Vec3> circle_pts;
        std::vector<Vec3i> circle_colors;
        std::vector<double> circle_radii;
        for (const auto &pt : *circle_cloud) {
          circle_pts.emplace_back(pt.x, pt.y, pt.z);
          circle_colors.push_back(colormap::jet(0.4));
          circle_radii.emplace_back(0.02);
        }
        std::string prefix = "/lidar/plane_" + std::to_string(plane_id);
        log.log_points(prefix + "/circle/inliers",
                       0,
                       circle_pts,
                       circle_colors,
                       circle_radii);

        // Render circle circumference (project onto the detected plane
        // so rendering is consistent with the plane surface)
        const auto &c = circle_coeffs->values;
        Vec3 circle_center(c[0], c[1], c[2]);
        Vec3 circle_normal(c[3], c[4], c[5]);
        double radius = c[6];

        // Project circle center onto the detected plane
        double plane_norm = std::sqrt(coeffs->values[0] * coeffs->values[0] +
                                      coeffs->values[1] * coeffs->values[1] +
                                      coeffs->values[2] * coeffs->values[2]);
        double dist =
            plane_normal.dot(circle_center) + coeffs->values[3] / plane_norm;
        Vec3 center = circle_center - dist * plane_normal;

        Vec3 circle_normal_n = circle_normal.normalized();
        Vec3 ref = (std::abs(circle_normal_n.x()) < 0.9) ? Vec3(1, 0, 0)
                                                         : Vec3(0, 1, 0);
        Vec3 u = circle_normal_n.cross(ref).normalized();
        Vec3 v = circle_normal_n.cross(u).normalized();
        const int num_segments = 64;
        std::vector<Vec3> circle_line;
        for (int i = 0; i <= num_segments; ++i) {
          double theta = 2.0 * M_PI * i / num_segments;
          circle_line.push_back(
              center + radius * (std::cos(theta) * u + std::sin(theta) * v));
        }
        log.log_line(prefix + "/circle/rendered",
                     circle_line,
                     Vec3i{0, 255, 0},
                     0.015);
      }
    }

    // Remove plane inliers for next iteration
    pcl::PointCloud<PointT>::Ptr next(new pcl::PointCloud<PointT>);
    extract.setNegative(true);
    extract.filter(*next);
    remaining = next;
  }
}

// TEST(CalibCameraLidar, test) {
//   const auto pcd_path = fs::path("/home/chutsu/camera_lidar_va_observations/"
//                                  "velodyne_points/1690418045700958976.pcd");
//
//   // const auto pcd_points = load_pcd_points(pcd_path);
//   //
//   // std::vector<Vec3> points_3d;
//   // std::vector<Vec3i> points_colors;
//   // std::vector<double> points_radii;
//   // for (const auto pt : pcd_points) {
//   //   const auto x = pt[0];
//   //   const auto y = pt[1];
//   //   const auto z = pt[2];
//   //   const auto intensity = pt[3] / 255.0;
//   //
//   //   points_3d.emplace_back(x, y, z);
//   //   points_colors.push_back(colormap::jet(intensity));
//   //   points_radii.emplace_back(0.01);
//   // }
//   //
//   // Logger log;
//   // log.log_points("/lidar", 0, points_3d, points_colors, points_radii);
//
//   Logger log;
//
//   // -----------------------------
//   // Load point cloud
//   // -----------------------------
//   pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//   if (pcl::io::loadPCDFile<PointT>(pcd_path, *cloud) == -1) {
//     PCL_ERROR("Couldn't read file\n");
//   }
//   std::cout << "Loaded " << cloud->size() << " points" << std::endl;
//
//   // Log full lidar point cloud (color by intensity)
//   {
//     std::vector<Vec3> all_points;
//     std::vector<Vec3i> all_colors;
//     std::vector<double> all_radii;
//     double max_intensity = 0;
//     for (const auto &pt : *cloud) {
//       if (pt.intensity > max_intensity)
//         max_intensity = pt.intensity;
//     }
//     for (const auto &pt : *cloud) {
//       all_points.emplace_back(pt.x, pt.y, pt.z);
//       all_colors.push_back(colormap::jet(pt.intensity / max_intensity));
//       all_radii.emplace_back(0.005);
//     }
//     log.log_points("/lidar/points", 0, all_points, all_colors, all_radii);
//   }
//
//   lidar_detect_target(cloud, log);
// }

} // namespace cartesian
