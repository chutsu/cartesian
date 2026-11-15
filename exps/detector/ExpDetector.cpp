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

#include "camera/CameraGeometry.hpp"
#include "camera/Pinhole.hpp"

#include "AprilGridTarget.hpp"
#include "CharucoTarget.hpp"
#include "CheckerboardTarget.hpp"
#include "DetectorTarget.hpp"

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
 * CAMERA
 *****************************************************************************/

static CameraGeometry make_camera(const double fov, const bool distort) {
  const Vec2i resolution{752, 480};
  const double fx = pinhole_focal(resolution.x(), fov);
  const double fy = fx;
  const double cx = resolution.x() / 2.0;
  const double cy = resolution.y() / 2.0;

  VecX intrinsic = zeros(8, 1); // fx, fy, cx, cy, k1, k2, p1, p2
  intrinsic << fx, fy, cx, cy, 0.0, 0.0, 0.0, 0.0;
  if (distort) {
    intrinsic.tail(4) << -0.28, 0.07, 1e-4, -1e-4;
  }

  return CameraGeometry{0, "BrownConrady4", resolution, intrinsic};
}

/******************************************************************************
 * CAMERA POSES
 *****************************************************************************/

struct CameraPose {
  Mat4 T_TC; // Camera pose in the target frame
  double dist = 0.0;
  double tilt = 0.0; // [deg]
  double roll = 0.0; // [deg]
};

/** Sample camera poses T_TC looking at the board from the front */
static std::vector<CameraPose> sample_poses(const Settings &s,
                                            const DetectorTarget &target) {
  std::mt19937 gen{s.seed};
  std::uniform_real_distribution<double> uni{0.0, 1.0};
  auto uniform = [&](const double a, const double b) {
    return a + (b - a) * uni(gen);
  };

  const Vec3 center{target.width() / 2.0, target.height() / 2.0, 0.0};
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
    const Vec3 aim = center + Vec3{uniform(-0.1, 0.1) * target.width(),
                                   uniform(-0.1, 0.1) * target.height(),
                                   0.0};
    const Vec3 z = (aim - r_TC).normalized();
    const Vec3 x = Vec3{0.0, -1.0, 0.0}.cross(z).normalized();
    const Vec3 y = z.cross(x);
    Mat3 C_TC;
    C_TC << x, y, z;

    // Roll about the optical axis
    const double roll = deg2rad(uniform(-180.0, 180.0));
    C_TC = C_TC * rotz(roll);

    poses.push_back({tf(C_TC, r_TC), dist, rad2deg(tilt), rad2deg(roll)});
  }

  return poses;
}

/******************************************************************************
 * RENDERER
 *****************************************************************************/

/** Renders a planar target texture as seen by a camera */
class Renderer {
private:
  const Settings &settings_;
  const CameraGeometry &camera_;
  std::vector<Vec3> rays_; // Rays of every (super sampled) pixel
  int rays_w_ = 0;
  int rays_h_ = 0;

public:
  Renderer(const Settings &settings, const CameraGeometry &camera)
      : settings_{settings}, camera_{camera} {
    // Pre-compute rays of every pixel
    const int k = settings_.supersample;
    rays_w_ = camera_.resolution.x() * k;
    rays_h_ = camera_.resolution.y() * k;
    rays_.resize(rays_w_ * rays_h_);

    cv::parallel_for_(cv::Range{0, rays_h_}, [&](const cv::Range &rows) {
      for (int v = rows.start; v < rows.end; ++v) {
        for (int u = 0; u < rays_w_; ++u) {
          // Pixel center of the super sampled pixel in the image (the center
          // of pixel (0, 0) is at (0, 0))
          const Vec2 z{(u + 0.5) / k - 0.5, (v + 0.5) / k - 0.5};
          camera_.back_project(z, rays_[v * rays_w_ + u]);
        }
      }
    });
  }

  cv::Mat render(const cv::Mat &texture,
                 const double margin,
                 const Mat4 &T_TC,
                 std::mt19937 &gen) const {
    const Mat3 C_TC = T_TC.block<3, 3>(0, 0);
    const Vec3 r_TC = T_TC.block<3, 1>(0, 3);
    const double ppm = DetectorTarget::PPM;
    const double texture_h = texture.rows;

    // Ray cast pixels onto the target plane (z = 0) to find texture lookup
    cv::Mat map_x(rays_h_, rays_w_, CV_32FC1);
    cv::Mat map_y(rays_h_, rays_w_, CV_32FC1);
    cv::parallel_for_(cv::Range{0, rays_h_}, [&](const cv::Range &rows) {
      for (int v = rows.start; v < rows.end; ++v) {
        for (int u = 0; u < rays_w_; ++u) {
          const Vec3 ray = C_TC * rays_[v * rays_w_ + u];
          const double s = -r_TC.z() / ray.z();
          float mx = -1.0f;
          float my = -1.0f;
          if (s > 0.0) {
            const double x = r_TC.x() + s * ray.x();
            const double y = r_TC.y() + s * ray.y();
            // Meters to texture pixels (board frame has y up, image y down)
            mx = (x + margin) * ppm - 0.5;
            my = texture_h - (y + margin) * ppm - 0.5;
          }
          map_x.at<float>(v, u) = mx;
          map_y.at<float>(v, u) = my;
        }
      }
    });

    // Sample texture (gray outside of the texture)
    cv::Mat image;
    cv::remap(texture,
              image,
              map_x,
              map_y,
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT,
              cv::Scalar{128});

    // Anti-alias, blur and add noise
    cv::resize(image,
               image,
               cv::Size{camera_.resolution.x(), camera_.resolution.y()},
               0,
               0,
               cv::INTER_AREA);
    if (settings_.blur_sigma > 0.0) {
      cv::GaussianBlur(image, image, cv::Size{}, settings_.blur_sigma);
    }
    if (settings_.noise_sigma > 0.0) {
      std::normal_distribution<double> noise{0.0, settings_.noise_sigma};
      for (int v = 0; v < image.rows; ++v) {
        for (int u = 0; u < image.cols; ++u) {
          const double p = image.at<uchar>(v, u) + noise(gen);
          image.at<uchar>(v, u) = cv::saturate_cast<uchar>(p);
        }
      }
    }

    return image;
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

static Result evaluate(const Settings &s,
                       const CameraGeometry &camera,
                       DetectorTarget &target,
                       const CameraPose &pose,
                       const int pose_index,
                       const cv::Mat &image) {
  Result r;
  r.pose_index = pose_index;
  r.target = target.name();
  r.dist = pose.dist;
  r.tilt = pose.tilt;
  r.roll = pose.roll;

  // Ground truth corner projections
  const Mat4 T_CT = pose.T_TC.inverse();
  std::map<int, Vec2> gt;
  for (const auto &[id, p_T] : target.corners()) {
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
    if (err.norm() > s.outlier_px) {
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
  const auto camera = make_camera(settings.fov, settings.distort);
  Renderer renderer{settings, camera};
  std::vector<std::unique_ptr<DetectorTarget>> targets;
  for (const auto &detector_type : settings.apriltag_detectors) {
    targets.push_back(std::make_unique<AprilGridTarget>(true, detector_type));
    targets.push_back(std::make_unique<AprilGridTarget>(false, detector_type));
  }
  targets.push_back(std::make_unique<CheckerboardTarget>());
  targets.push_back(std::make_unique<CharucoTarget>());

  // Camera poses (shared by all targets)
  const auto poses = sample_poses(settings, *targets.front());

  // Sanity check - frontal view without blur and noise, the ground truth and
  // detections should agree to within a fraction of a pixel
  {
    Settings clean = settings;
    clean.blur_sigma = 0.0;
    clean.noise_sigma = 0.0;
    Renderer clean_renderer{clean, camera};
    std::mt19937 gen{0};

    printf("Sanity check (frontal view, 1m, no blur, no noise)\n");
    for (auto &target : targets) {
      const Vec3 center{target->width() / 2.0, target->height() / 2.0, 0.0};
      CameraPose pose;
      pose.T_TC = tf(Mat3{Mat3::Identity()}, center + Vec3{0.0, 0.0, 1.0});
      pose.T_TC.block<3, 3>(0, 0) = rotx(M_PI); // Look down at the board
      pose.dist = 1.0;

      const auto texture = target->render(clean.margin);
      const auto image =
          clean_renderer.render(texture, clean.margin, pose.T_TC, gen);
      const auto r = evaluate(clean, camera, *target, pose, 0, image);
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
    target_names.push_back(target->name());
    const auto texture = target->render(settings.margin);
    if (settings.save_images) {
      fs::create_directories(settings.output_dir / target->name());
      cv::imwrite(settings.output_dir / target->name() / "texture.png",
                  texture);
    }

    printf("Simulating [%s] ", target->name().c_str());
    fflush(stdout);
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto image =
          renderer.render(texture, settings.margin, poses[i].T_TC, noise_gen);
      if (settings.save_images) {
        char name[32];
        snprintf(name, sizeof(name), "%04zu.png", i);
        cv::imwrite(settings.output_dir / target->name() / name, image);
      }
      results.push_back(
          evaluate(settings, camera, *target, poses[i], i, image));
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
