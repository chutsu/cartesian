/**
 * Camera calibration experiment
 *
 * Simulates calibration datasets of a plain AprilGrid for cameras with
 * different fields of view (90 - 180 deg) and amounts of distortion (including
 * fisheye lenses), detects the corners, and puts them through `CalibCamera` to
 * see whether the ground truth camera parameters are recovered.
 *
 * Settings are read from a YAML file, see config.yaml.
 *
 * Usage: exp_calib_camera <config.yaml>
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/CalibCamera.hpp"

#include "AprilGridTarget.hpp"
#include "CharucoTarget.hpp"
#include "CheckerboardTarget.hpp"
#include "Sim.hpp"

using namespace cartesian;

/******************************************************************************
 * SETTINGS
 *****************************************************************************/

struct Case {
  std::string name;
  std::string model;
  int fov;
  std::string level;
  CameraSpec spec;
  Vec2 dist_range;
  std::string skip_reason; // Empty if the camera can be simulated
};

struct Settings {
  fs::path output_dir;
  bool save_samples;
  int num_views;
  unsigned int seed;
  std::vector<std::string> apriltag_detectors;
  std::vector<std::string> inits;
  double max_tilt;
  double max_theta;
  RenderParams render;
  std::vector<Case> cases;

  Settings(const std::string &config_file) {
    config_t config{config_file};
    parse(config, "output_dir", output_dir);
    parse(config, "save_samples", save_samples);
    parse(config, "num_views", num_views);
    parse(config, "seed", seed);
    parse(config, "apriltag_detectors", apriltag_detectors, false);
    parse(config, "initial_guesses", inits, false);
    parse(config, "poses.max_tilt", max_tilt);
    parse(config, "poses.max_theta", max_theta);
    parse(config, "render.supersample", render.supersample);
    parse(config, "render.margin", render.margin);
    parse(config, "render.blur_sigma", render.blur_sigma);
    parse(config, "render.noise_sigma", render.noise_sigma);

    // Ground truth cameras
    Vec2i resolution;
    double cx_offset, cy_offset, fy_ratio;
    std::vector<int> fovs;
    {
      std::vector<int> res;
      parse(config, "camera.resolution", res, false);
      resolution = Vec2i{res.at(0), res.at(1)};
    }
    parse(config, "camera.cx_offset", cx_offset);
    parse(config, "camera.cy_offset", cy_offset);
    parse(config, "camera.fy_ratio", fy_ratio);
    parse(config, "fovs", fovs, false);

    for (const auto &model_node : config.root["models"]) {
      const std::string model = model_node.first.as<std::string>();
      for (const auto &level_node : model_node.second) {
        const std::string level = level_node.first.as<std::string>();
        const auto level_cfg = level_node.second;

        for (const int fov : fovs) {
          Case c;
          c.model = model;
          c.fov = fov;
          c.level = level;
          c.name = model + "_" + std::to_string(fov) + "_" + level;

          // Camera spec
          c.spec.model = model;
          c.spec.resolution = resolution;
          c.spec.fov = fov;
          c.spec.cx_offset = cx_offset;
          c.spec.cy_offset = cy_offset;
          c.spec.fy_ratio = fy_ratio;
          if (model == "KannalaBrandt4") {
            const auto k = level_cfg["k"].as<std::vector<double>>();
            c.spec.dist = Vec4{k[0], k[1], k[2], k[3]};
          } else {
            const double r = tan(deg2rad(fov / 2.0)); // Radius at the edge
            const auto tan_d = level_cfg["tangential"].as<std::vector<double>>();
            c.spec.dist = Vec4{level_cfg["radial_edge"].as<double>() / (r * r),
                               level_cfg["radial2_edge"].as<double>() /
                                   (r * r * r * r),
                               tan_d[0],
                               tan_d[1]};
          }

          // Distance range
          const auto range = config.root["poses"]["dist_range"]
                                 [std::to_string(fov)]
                                     .as<std::vector<double>>();
          c.dist_range = Vec2{range[0], range[1]};

          // Check the camera can be simulated
          if (model != "KannalaBrandt4" && fov >= 180) {
            c.skip_reason = "a pinhole camera cannot see 180 deg";
          } else {
            const auto camera = make_camera(c.spec);
            if (camera_is_monotonic(camera) == false) {
              c.skip_reason = "distortion folds the image over";
            }
          }
          cases.push_back(c);
        }
      }
    }
  }
};

/******************************************************************************
 * CALIBRATION
 *****************************************************************************/

struct CaseResult {
  Case c;
  std::string detector;
  std::string init; // Initial intrinsic guess: "default" or "near_truth"
  bool ok = false;
  std::string note;
  int num_views = 0;           // Views where the grid was detected
  int num_views_used = 0;      // Views used by the calibrator
  int num_corners = 0;         // Corners used by the calibrator
  int num_outliers = 0;        // Corners with an error > 5 px
  VecX gt = zeros(8, 1);       // Ground truth intrinsic
  VecX est = zeros(8, 1);      // Estimated intrinsic
  double model_rmse = NAN;     // Projection difference to ground truth [px]
  double model_max = NAN;
  double reproj_rmse = NAN;    // Calibrator reprojection error [px]
  double solve_s = NAN;
};

/** Difference between the estimated and ground truth projections [px] */
static void model_error(const CameraGeometry &gt,
                        const CameraGeometry &est,
                        const double max_theta,
                        double &rmse,
                        double &max_err) {
  double sse = 0.0;
  int n = 0;
  max_err = 0.0;
  for (double v = 0.0; v < gt.resolution.y(); v += 8.0) {
    for (double u = 0.0; u < gt.resolution.x(); u += 8.0) {
      const Vec2 z{u, v};
      const Vec3 ray = camera_ray(gt, z);
      if (rad2deg(acos(ray.z())) > max_theta) {
        continue;
      }

      Vec2 z_est;
      est.project(ray, z_est);
      const double err = (z_est - z).norm();
      if (std::isfinite(err) == false) {
        rmse = NAN;
        max_err = NAN;
        return;
      }
      sse += err * err;
      max_err = std::max(max_err, err);
      n++;
    }
  }
  rmse = (n > 0) ? sqrt(sse / n) : NAN;
}

static cv::Mat draw_overlay(const cv::Mat &image,
                            const std::vector<Vec2> &good,
                            const std::vector<Vec2> &bad) {
  cv::Mat bgr;
  cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  for (const auto &z : good) {
    cv::circle(bgr, cv::Point2f(z.x(), z.y()), 3, cv::Scalar{0, 200, 0}, 1);
  }
  for (const auto &z : bad) {
    cv::circle(bgr, cv::Point2f(z.x(), z.y()), 4, cv::Scalar{0, 0, 255}, 1);
  }
  return bgr;
}

/** Calibrate from the detected grids with the given initial intrinsic guess */
static CaseResult calibrate(const Settings &s,
                            CaseResult r,
                            const std::string &init,
                            const AprilGridTarget &target,
                            const std::vector<std::shared_ptr<AprilGrid>> &grids,
                            const CameraGeometry &gt_camera) {
  r.init = init;
  const auto &c = r.c;
  if (r.num_views < 5) {
    r.note = "fewer than 5 views with a detected grid";
    return r;
  }

  try {
    CalibCamera calib;
    calib.verbose = false;

    // Initial guess: the default (a 90 deg camera, no distortion), or the
    // ground truth with the focal lengths and principal point perturbed and
    // no distortion
    VecX initial;
    if (init == "near_truth") {
      initial = gt_camera.intrinsic;
      initial(0) *= 1.10;
      initial(1) *= 0.92;
      initial(2) += 8.0;
      initial(3) -= 6.0;
      initial.tail(4).setZero();
    }
    calib.add_camera(0, c.model, gt_camera.resolution, initial, tf_vec());
    calib.add_target(target.config, tf_vec());
    for (const auto &grid : grids) {
      calib.add_camera_measurement(grid->ts, 0, grid);
    }

    const auto t_start = std::chrono::steady_clock::now();
    calib.solve();
    const auto t_end = std::chrono::steady_clock::now();
    r.solve_s = std::chrono::duration<double>(t_end - t_start).count();

    r.est = calib.get_camera_geometry(0)->intrinsic;
    r.num_views_used = calib.poses.size();

    double sse = 0.0;
    int n = 0;
    for (const auto &[ts, cams] : calib.resblocks) {
      for (const auto &[cam_id, blocks] : cams) {
        for (const auto &block : blocks) {
          double error = 0.0;
          if (block->get_reproj_error(&error)) {
            sse += error * error;
            n++;
          }
        }
      }
    }
    r.num_corners = n;
    r.reproj_rmse = (n > 0) ? sqrt(sse / n) : NAN;

    // Compare against the ground truth
    if (r.est.allFinite() == false) {
      r.note = "non-finite parameters";
      return r;
    }
    const CameraGeometry est_camera{0,
                                    c.model,
                                    gt_camera.resolution,
                                    r.est};
    model_error(gt_camera, est_camera, s.max_theta, r.model_rmse, r.model_max);
    r.ok = std::isfinite(r.model_rmse);

  } catch (const std::exception &e) {
    r.note = std::string("exception: ") + e.what();
  }

  return r;
}

/** Render and detect a calibration dataset, and calibrate from it */
static std::vector<CaseResult>
run_case(const Settings &s,
         const Case &c,
         const std::string &detector_type,
         const std::vector<std::string> &inits,
         const std::vector<CameraPose> &poses,
         const CameraGeometry &gt_camera,
         const Renderer &renderer,
         std::ofstream &corners_csv,
         const bool save_overlays) {
  CaseResult r;
  r.c = c;
  r.detector = detector_type;
  r.gt = gt_camera.intrinsic;

  // Render and detect
  AprilGridTarget target{false, detector_type};
  const auto texture = target.render(s.render.margin);
  std::mt19937 noise_gen{s.seed + 1};
  std::vector<std::shared_ptr<AprilGrid>> grids;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto image = renderer.render(texture, poses[i].T_TC, noise_gen);
    const auto grid = target.detect_grid(image, (i + 1) * 100000000);

    // Ground truth projections, in order to drop corners that the camera
    // model cannot represent (too far off the optical axis) and to count
    // outliers
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    Vec2s keypoints;
    Vec3s object_points;
    grid->get_measurements(tag_ids, corner_indicies, keypoints, object_points);

    const Mat4 T_CT = poses[i].T_TC.inverse();
    std::vector<Vec2> good;
    std::vector<Vec2> bad;
    for (size_t j = 0; j < tag_ids.size(); ++j) {
      const Vec3 p_C = tf_point(T_CT, object_points[j]);
      const double theta = rad2deg(acos(p_C.z() / p_C.norm()));
      if (p_C.z() <= 0.0 || theta > s.max_theta) {
        grid->remove(tag_ids[j], corner_indicies[j]);
        continue;
      }

      Vec2 z_gt;
      gt_camera.project(p_C, z_gt);
      if ((keypoints[j] - z_gt).norm() > 5.0) {
        r.num_outliers++;
        bad.push_back(keypoints[j]);
      } else {
        good.push_back(keypoints[j]);
      }
      corners_csv << c.name << "," << detector_type << "," << keypoints[j].x()
                  << "," << keypoints[j].y() << "\n";
    }

    if (grid->get_num_detected() >= 10) {
      r.num_views++;
      grids.push_back(grid);
    }
    if (save_overlays && s.save_samples && i < 6) {
      char name[64];
      snprintf(name, sizeof(name), "view_%02zu.png", i);
      const auto dir = s.output_dir / "samples" / "views" /
                       (c.name + "_" + detector_type);
      fs::create_directories(dir);
      cv::imwrite(dir / name, draw_overlay(image, good, bad));
    }
  }

  // Calibrate
  std::vector<CaseResult> results;
  for (const auto &init : inits) {
    results.push_back(calibrate(s, r, init, target, grids, gt_camera));
  }
  return results;
}

/******************************************************************************
 * OUTPUT
 *****************************************************************************/

static void save_results(const fs::path &path,
                         const std::vector<CaseResult> &results) {
  std::ofstream f{path};
  f << "case,model,fov,level,detector,init,ok,note,views,views_used,corners,"
       "outlier_corners";
  const char *names[8] = {"fx", "fy", "cx", "cy", "d1", "d2", "d3", "d4"};
  for (const auto *suffix : {"_gt", "_est"}) {
    for (const auto *name : names) {
      f << "," << name << suffix;
    }
  }
  f << ",model_rmse_px,model_max_px,reproj_rmse_px,solve_s\n";

  for (const auto &r : results) {
    f << r.c.name << "," << r.c.model << "," << r.c.fov << "," << r.c.level
      << "," << r.detector << "," << r.init << "," << r.ok << ",\""
      << r.note << "\","
      << r.num_views << "," << r.num_views_used << "," << r.num_corners << ","
      << r.num_outliers;
    for (const auto &v : {r.gt, r.est}) {
      for (int i = 0; i < 8; ++i) {
        f << "," << v(i);
      }
    }
    f << "," << r.model_rmse << "," << r.model_max << "," << r.reproj_rmse
      << "," << r.solve_s << "\n";
  }
}

static void save_target_samples(const Settings &s) {
  const auto dir = s.output_dir / "samples" / "targets";
  fs::create_directories(dir);

  AprilGridTarget kalibr_kaess{true, "kaess"};
  AprilGridTarget plain_kaess{false, "kaess"};
  CheckerboardTarget checkerboard;
  CharucoTarget charuco;
  for (DetectorTarget *target : std::vector<DetectorTarget *>{&kalibr_kaess,
                                                              &plain_kaess,
                                                              &checkerboard,
                                                              &charuco}) {
    cv::imwrite(dir / (target->name() + ".png"), target->render(s.render.margin));
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
  const Settings s{argv[1]};
  fs::create_directories(s.output_dir);
  if (s.save_samples) {
    save_target_samples(s);
  }

  std::ofstream corners_csv{s.output_dir / "corners.csv"};
  corners_csv << "case,detector,u,v\n";

  std::vector<CaseResult> results;
  for (const auto &c : s.cases) {
    if (c.skip_reason.empty() == false) {
      printf("%-28s skipped: %s\n", c.name.c_str(), c.skip_reason.c_str());
      for (const auto &detector_type : s.apriltag_detectors) {
        for (const auto &init : s.inits) {
          CaseResult r;
          r.c = c;
          r.detector = detector_type;
          r.init = init;
          r.note = "skipped: " + c.skip_reason;
          results.push_back(r);
        }
      }
      continue;
    }

    // Camera, poses and renderer (shared by the detectors)
    const auto gt_camera = make_camera(c.spec);
    AprilGridTarget board{false, "kaess"};
    const PoseSampling sampling{s.seed,
                                s.num_views,
                                c.dist_range.x(),
                                c.dist_range.y(),
                                s.max_tilt};
    const auto poses = sample_poses_spread(sampling, board, gt_camera, s.max_theta);
    const Renderer renderer{s.render, gt_camera};

    // Sample render, looking straight at the board
    if (s.save_samples) {
      const auto dir = s.output_dir / "samples" / "cameras";
      fs::create_directories(dir);
      const Vec3 center{board.width() / 2.0, board.height() / 2.0, 0.0};
      Mat4 T_TC = tf(rotx(M_PI), center + Vec3{0.0, 0.0, c.dist_range.x()});
      std::mt19937 gen{0};
      cv::imwrite(dir / (c.name + ".png"),
                  renderer.render(board.render(s.render.margin), T_TC, gen));
    }

    bool first = true;
    for (const auto &detector_type : s.apriltag_detectors) {
      const auto case_results = run_case(s,
                                         c,
                                         detector_type,
                                         s.inits,
                                         poses,
                                         gt_camera,
                                         renderer,
                                         corners_csv,
                                         first);
      first = false;
      for (const auto &r : case_results) {
        printf("%-26s %-6s %-10s views %3d/%3d corners %6d outliers %4d  ",
               c.name.c_str(),
               detector_type.c_str(),
               r.init.c_str(),
               r.num_views_used,
               r.num_views,
               r.num_corners,
               r.num_outliers);
        if (r.ok) {
          printf("model rmse %9.3f px  max %10.3f px  reproj %.3f px\n",
                 r.model_rmse,
                 r.model_max,
                 r.reproj_rmse);
        } else {
          printf("FAILED: %s\n", r.note.c_str());
        }
        fflush(stdout);
        results.push_back(r);
      }
    }
  }
  save_results(s.output_dir / "results.csv", results);
  printf("\nResults saved to [%s]\n", s.output_dir.c_str());

  return 0;
}
