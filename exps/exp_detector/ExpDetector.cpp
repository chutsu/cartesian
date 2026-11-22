/**
 * Detector experiment
 *
 * Simulates a camera at different poses in-front of a calibration target and
 * compares how well the following targets are detected:
 *
 * - AprilGrid, kalibr style (black squares between tags)
 * - AprilGrid, without black squares
 * - Checkerboard
 * - ChArUco
 *
 * Every target is viewed from the same set of camera poses. Images are
 * rendered by ray casting each pixel onto the target plane, so the ground
 * truth corner locations are known exactly.
 *
 * Settings are read from a YAML file, see config.yaml.
 *
 * Usage: exp_detector <config.yaml>
 */
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "sim/SimPoseSampling.hpp"
#include "sim/SimRenderer.hpp"
#include "sim/SimCamera.hpp"
#include "sim/SimTarget.hpp"
#include "camera/CameraGeometry.hpp"


using namespace cartesian;

/******************************************************************************
 * SETTINGS
 *****************************************************************************/

struct Settings {
  // General
  fs::path output_dir;
  bool save_images;
  int num_poses;
  unsigned int seed;
  struct {
    bool aprilgrid;
    bool checkerboard;
    bool charuco;
  } refine_corners; // Refine corners to sub-pixel accuracy, per target type
  std::vector<std::string> apriltag_detectors;

  // Camera
  double fov;   // Horizontal field of view [deg]
  bool distort; // Add lens distortion

  // Camera pose sampling
  double min_dist; // Distance to board center [m]
  double max_dist;
  double max_tilt; // Angle between optical axis and target normal [deg]

  // Image rendering
  int supersample;    // Anti-aliasing
  double margin;      // White margin around board [m]
  double blur_sigma;  // Gaussian blur [px]
  double noise_sigma; // Gaussian noise [intensity]

  // Evaluation
  double outlier_px; // Corner error larger than this is an outlier

  Settings(const std::string &config_file) {
    config_t config{config_file};
    parse(config, "output_dir", output_dir);
    parse(config, "save_images", save_images);
    parse(config, "num_poses", num_poses);
    parse(config, "seed", seed);
    parse(config, "refine_corners.aprilgrid", refine_corners.aprilgrid);
    parse(config, "refine_corners.checkerboard", refine_corners.checkerboard);
    parse(config, "refine_corners.charuco", refine_corners.charuco);
    parse(config, "apriltag_detectors", apriltag_detectors, false);
    parse(config, "camera.fov", fov);
    parse(config, "camera.distort", distort);
    parse(config, "poses.min_dist", min_dist);
    parse(config, "poses.max_dist", max_dist);
    parse(config, "poses.max_tilt", max_tilt);
    parse(config, "render.supersample", supersample);
    parse(config, "render.margin", margin);
    parse(config, "render.blur_sigma", blur_sigma);
    parse(config, "render.noise_sigma", noise_sigma);
    parse(config, "evaluation.outlier_px", outlier_px);
  }
};

/******************************************************************************
 * METRICS
 *****************************************************************************/

struct Result {
  int pose_index = 0;
  std::string target;
  double dist = 0.0;
  double tilt = 0.0;
  double roll = 0.0;
  int num_visible = 0;    // Ground truth corners inside the image
  int num_detected = 0;   // Detected corners that are visible
  int num_outliers = 0;   // Detected corners with error > outlier_px
  int num_inliers = 0;    // Detected corners with error <= outlier_px
  double sse = 0.0;       // Inlier squared error sum [px^2]
  Vec2 sum_err{0.0, 0.0}; // Inlier signed error sum [px]
  double detect_ms = 0.0; // Detection time [ms]
  std::vector<std::pair<int, Vec2>> errors; // Inlier corner errors (id, error)

  double rmse() const {
    return (num_inliers > 0) ? sqrt(sse / num_inliers) : NAN;
  }
};

static Result evaluate(const double outlier_px,
                       const CameraGeometry &camera,
                       const SimTarget &target,
                       const SimPoseSampling::CameraPose &pose,
                       const int pose_index,
                       const cv::Mat &image) {
  Result r;
  r.pose_index = pose_index;
  r.target = target.name;
  r.dist = pose.dist;
  r.tilt = pose.tilt;
  r.roll = pose.roll;

  // Ground truth corner projections
  const Mat4 T_CT = pose.T_TC.inverse();
  std::map<int, Vec2> gt;
  for (const auto &[id, p_T] : target.corners) {
    Vec2 z;
    const Vec3 p_C = tf_point(T_CT, p_T);
    if (camera.project(p_C, z) != 0) {
      continue;
    }
    // Keep a border so that corners are fully visible to the detector
    const double b = 5.0;
    if (z.x() < b || z.y() < b || z.x() > image.cols - b ||
        z.y() > image.rows - b) {
      continue;
    }
    gt[id] = z;
  }
  r.num_visible = gt.size();

  // Compare detections
  const auto t_start = std::chrono::steady_clock::now();
  const auto detected = target.detect(image);
  const auto t_end = std::chrono::steady_clock::now();
  r.detect_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

  for (const auto &[id, z] : detected) {
    if (gt.count(id) == 0) {
      continue; // Not expected to be visible
    }

    r.num_detected++;
    const Vec2 err = z - gt.at(id);
    if (err.norm() > outlier_px) {
      r.num_outliers++;
    } else {
      r.sse += err.squaredNorm();
      r.sum_err += err;
      r.num_inliers++;
      r.errors.emplace_back(id, err);
    }
  }

  return r;
}

/******************************************************************************
 * OUTPUT
 *****************************************************************************/

static void save_csv(const fs::path &path, const std::vector<Result> &results) {
  std::ofstream f{path};
  f << "pose,target,dist_m,tilt_deg,roll_deg,num_visible,num_detected,"
       "num_outliers,rmse_px,bias_x_px,bias_y_px,detect_ms\n";
  for (const auto &r : results) {
    f << r.pose_index << "," << r.target << "," << r.dist << "," << r.tilt
      << "," << r.roll << "," << r.num_visible << "," << r.num_detected << ","
      << r.num_outliers << "," << r.rmse() << ","
      << (r.num_inliers ? r.sum_err.x() / r.num_inliers : NAN) << ","
      << (r.num_inliers ? r.sum_err.y() / r.num_inliers : NAN) << ","
      << r.detect_ms << "\n";
  }
}

/** Save inlier corner errors, one row per detected corner */
static void save_errors_csv(const fs::path &path,
                            const std::vector<Result> &results) {
  std::ofstream f{path};
  f << "pose,target,corner_id,dist_m,tilt_deg,roll_deg,err_x_px,err_y_px\n";
  for (const auto &r : results) {
    for (const auto &[id, err] : r.errors) {
      f << r.pose_index << "," << r.target << "," << id << "," << r.dist << ","
        << r.tilt << "," << r.roll << "," << err.x() << "," << err.y() << "\n";
    }
  }
}

/** Print summary table, results grouped by `bin(result)` for each target */
template <typename F>
static void print_summary(const std::string &title,
                          const std::vector<std::string> &targets,
                          const std::vector<Result> &results,
                          const std::vector<std::string> &bin_names,
                          F bin) {
  printf("\n%s\n", title.c_str());
  printf("%-24s %-9s %6s %9s %9s %9s %9s %9s %9s\n",
         "target",
         "bin",
         "poses",
         "detect[%]",
         "recall[%]",
         "outlier[%]",
         "rmse[px]",
         "bias[px]",
         "time[ms]");
  printf("-----------------------------------------------------------------"
         "-----------------------------\n");

  for (const auto &target : targets) {
    for (size_t b = 0; b < bin_names.size(); ++b) {
      int poses = 0;
      int detected_poses = 0;
      double recall_sum = 0.0;
      int detected = 0;
      int outliers = 0;
      int inliers = 0;
      double sse = 0.0;
      Vec2 sum_err{0.0, 0.0};
      double time_sum = 0.0;
      for (const auto &r : results) {
        if (r.target != target || bin(r) != static_cast<int>(b) ||
            r.num_visible == 0) {
          continue;
        }
        poses++;
        detected_poses += (r.num_inliers > 0) ? 1 : 0;
        recall_sum += double(r.num_inliers) / r.num_visible;
        detected += r.num_detected;
        outliers += r.num_outliers;
        inliers += r.num_inliers;
        sse += r.sse;
        sum_err += r.sum_err;
        time_sum += r.detect_ms;
      }
      if (poses == 0) {
        continue;
      }

      printf("%-24s %-9s %6d %9.1f %9.1f %9.1f %9.3f %9.3f %9.2f\n",
             target.c_str(),
             bin_names[b].c_str(),
             poses,
             100.0 * detected_poses / poses,
             100.0 * recall_sum / poses,
             (detected > 0) ? 100.0 * outliers / detected : NAN,
             (inliers > 0) ? sqrt(sse / inliers) : NAN,
             (inliers > 0) ? (sum_err / inliers).norm() : NAN,
             time_sum / poses);
    }
  }
}

/******************************************************************************
 * MAIN
 *****************************************************************************/

int main(int argc, char *argv[]) {
  if (argc != 2) {
    printf("Usage: %s <config.yaml>\n", argv[0]);
    return -1;
  }
  const Settings settings{argv[1]};
  fs::create_directories(settings.output_dir);

  // Setup
  SimCamera sim_camera;
  sim_camera.fov = settings.fov;
  Vec4 dist = Vec4::Zero();
  if (settings.distort) {
    dist = Vec4{-0.28, 0.07, 1e-4, -1e-4};
  }
  const auto camera = sim_camera.make_brown_conrady4(dist);
  const SimRenderer::RenderParams render_params{settings.supersample,
                                   settings.margin,
                                   settings.blur_sigma,
                                   settings.noise_sigma};
  SimRenderer renderer{render_params, camera};
  std::vector<SimTarget> targets;
  for (const auto &detector_type : settings.apriltag_detectors) {
    targets.push_back(SimTarget::aprilgrid(
        true, detector_type, settings.refine_corners.aprilgrid));
    targets.push_back(SimTarget::aprilgrid(
        false, detector_type, settings.refine_corners.aprilgrid));
  }
  targets.push_back(
      SimTarget::checkerboard(settings.refine_corners.checkerboard));
  targets.push_back(SimTarget::charuco(settings.refine_corners.charuco));

  // Camera poses (shared by all targets)
  const SimPoseSampling sampling{settings.seed,
                              settings.num_poses,
                              settings.min_dist,
                              settings.max_dist,
                              settings.max_tilt};
  const auto poses =
      sampling.sample_poses(targets.front().width, targets.front().height);

  // Sanity check - frontal view without blur and noise, the ground truth and
  // detections should agree to within a fraction of a pixel
  {
    SimRenderer::RenderParams clean = render_params;
    clean.blur_sigma = 0.0;
    clean.noise_sigma = 0.0;
    SimRenderer clean_renderer{clean, camera};
    std::mt19937 gen{0};

    printf("Sanity check (frontal view, 1m, no blur, no noise)\n");
    for (auto &target : targets) {
      const Vec3 center{target.width / 2.0, target.height / 2.0, 0.0};
      SimPoseSampling::CameraPose pose;
      pose.T_TC = tf(Mat3{Mat3::Identity()}, center + Vec3{0.0, 0.0, 1.0});
      pose.T_TC.block<3, 3>(0, 0) = rotx(M_PI); // Look down at the board
      pose.dist = 1.0;

      const auto texture = target.render(clean.margin, clean.ppm);
      const auto image = clean_renderer.render(texture, pose.T_TC, gen);
      const auto r =
          evaluate(settings.outlier_px, camera, target, pose, 0, image);
      printf("  %-24s visible: %3d  detected: %3d  outliers: %3d  rmse: %.3f "
             "px\n",
             r.target.c_str(),
             r.num_visible,
             r.num_detected,
             r.num_outliers,
             r.rmse());
    }
    printf("\n");
  }

  // Simulate
  std::mt19937 noise_gen{settings.seed + 1};
  std::vector<Result> results;
  std::vector<std::string> target_names;
  for (auto &target : targets) {
    target_names.push_back(target.name);
    const auto texture = target.render(settings.margin, render_params.ppm);
    if (settings.save_images) {
      fs::create_directories(settings.output_dir / target.name);
      cv::imwrite(settings.output_dir / target.name / "texture.png",
                  texture);
    }

    printf("Simulating [%s] ", target.name.c_str());
    fflush(stdout);
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto image = renderer.render(texture, poses[i].T_TC, noise_gen);
      if (settings.save_images) {
        char name[32];
        snprintf(name, sizeof(name), "%04zu.png", i);
        cv::imwrite(settings.output_dir / target.name / name, image);
      }
      results.push_back(
          evaluate(settings.outlier_px, camera, target, poses[i], i, image));
      if (i % 20 == 0) {
        printf(".");
        fflush(stdout);
      }
    }
    printf("\n");
  }
  save_csv(settings.output_dir / "results.csv", results);
  save_errors_csv(settings.output_dir / "errors.csv", results);

  // Summaries
  print_summary("All poses",
                target_names,
                results,
                {"all"},
                [](const Result &) { return 0; });

  const std::vector<std::string> tilt_bins{"0-15",
                                           "15-30",
                                           "30-45",
                                           "45-60",
                                           "60-75"};
  print_summary("By tilt (angle between optical axis and board normal) [deg]",
                target_names,
                results,
                tilt_bins,
                [](const Result &r) {
                  return std::min(int(r.tilt / 15.0), 4);
                });

  const std::vector<std::string> dist_bins{"0.8-1.0",
                                           "1.0-1.5",
                                           "1.5-2.0",
                                           "2.0-2.5"};
  print_summary("By distance [m]",
                target_names,
                results,
                dist_bins,
                [](const Result &r) {
                  if (r.dist < 1.0) {
                    return 0;
                  } else if (r.dist < 1.5) {
                    return 1;
                  } else if (r.dist < 2.0) {
                    return 2;
                  }
                  return 3;
                });
  printf("\nResults saved to [%s]\n", settings.output_dir.c_str());

  return 0;
}
