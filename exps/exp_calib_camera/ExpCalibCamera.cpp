/**
 * Camera calibration experiment
 *
 * Simulates calibration datasets of an AprilGrid, a checkerboard and a ChArUco
 * board for cameras with different fields of view (90 - 180 deg) and amounts of
 * distortion (including fisheye lenses), detects the corners, and puts them
 * through `CalibCamera` to see whether the ground truth camera parameters are
 * recovered.
 *
 * Settings are read from a YAML file, see config.yaml.
 *
 * Usage: exp_calib_camera <config.yaml>
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/AprilGridDetector.hpp"
#include "calib/CalibCamera.hpp"
#include "calib/CharucoDetector.hpp"
#include "calib/CheckerboardDetector.hpp"
#include "sim/PoseSampling.hpp"
#include "sim/Renderer.hpp"
#include "sim/SimTarget.hpp"
#include "camera/CameraSpec.hpp"


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

/** Refine corners to sub-pixel accuracy, per target type */
struct RefineCorners {
  bool aprilgrid;
  bool checkerboard;
  bool charuco;
};

struct Settings {
  fs::path output_dir;
  bool save_samples;
  int num_views;
  unsigned int seed;
  RefineCorners refine_corners;
  std::vector<std::string> targets;
  std::vector<std::string> fit_models; // Camera models to calibrate with
  std::vector<std::string> inits;
  double max_tilt;
  double max_theta;      // Corner limit off the optical axis [deg]
  double max_theta_wide; // Same, of the models that support rays beyond 90 deg
  RenderParams render;
  std::vector<Case> cases;

  /**
   * Corner limit off the optical axis [deg] when calibrating data of `c` with
   * `fit_model`. BrownConrady4 cannot represent rays 90 deg or more off the
   * optical axis, the other models can, which only matters for lenses with a
   * field of view above 180 deg.
   */
  double limit(const Case &c, const std::string &fit_model) const {
    const bool wide = (fit_model != "BrownConrady4" && c.fov > 180);
    return wide ? max_theta_wide : max_theta;
  }

  Settings(const std::string &config_file) {
    config_t config{config_file};
    parse(config, "output_dir", output_dir);
    parse(config, "save_samples", save_samples);
    parse(config, "num_views", num_views);
    parse(config, "seed", seed);
    parse(config, "refine_corners.aprilgrid", refine_corners.aprilgrid);
    parse(config, "refine_corners.checkerboard", refine_corners.checkerboard);
    parse(config, "refine_corners.charuco", refine_corners.charuco);
    parse(config, "targets", targets, false);
    parse(config, "fit_models", fit_models, false);
    parse(config, "initial_guesses", inits, false);
    parse(config, "poses.max_tilt", max_tilt);
    parse(config, "poses.max_theta", max_theta);
    parse(config, "poses.max_theta_wide", max_theta_wide);
    parse(config, "render.supersample", render.supersample);
    parse(config, "render.margin", render.margin);
    parse(config, "render.blur_sigma", render.blur_sigma);
    parse(config, "render.noise_sigma", render.noise_sigma);

    // Ground truth cameras
    Vec2i resolution;
    double cx_offset, cy_offset, fy_ratio, ucm_xi;
    std::vector<int> fovs;
    {
      std::vector<int> res;
      parse(config, "camera.resolution", res, false);
      resolution = Vec2i{res.at(0), res.at(1)};
    }
    parse(config, "camera.cx_offset", cx_offset);
    parse(config, "camera.cy_offset", cy_offset);
    parse(config, "camera.fy_ratio", fy_ratio);
    parse(config, "camera.ucm_xi", ucm_xi);
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
          c.spec.xi = ucm_xi;
          if (model == "KannalaBrandt4") {
            const auto k = level_cfg["k"].as<std::vector<double>>();
            c.spec.dist = Vec4{k[0], k[1], k[2], k[3]};
          } else {
            // Radius of the normalized image plane at the horizontal edge
            const double half_fov = deg2rad(fov / 2.0);
            const double r = (model == "UnifiedCameraModel")
                                 ? sin(half_fov) / (cos(half_fov) + ucm_xi)
                                 : tan(half_fov);
            const auto tan_d =
                level_cfg["tangential"].as<std::vector<double>>();
            c.spec.dist =
                Vec4{level_cfg["radial_edge"].as<double>() / (r * r),
                     level_cfg["radial2_edge"].as<double>() / (r * r * r * r),
                     tan_d[0],
                     tan_d[1]};
          }

          // Distance range
          const auto range =
              config.root["poses"]["dist_range"][std::to_string(fov)]
                  .as<std::vector<double>>();
          c.dist_range = Vec2{range[0], range[1]};

          // Check the camera can be simulated
          if (model == "BrownConrady4" && fov >= 180) {
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
 * CALIBRATION TARGETS
 *****************************************************************************/

/**
 * A calibration target of the experiment: the simulated target (renders the
 * texture, ground truth corners) and what `CalibCamera` needs to use it (the
 * library detector and the target config).
 */
struct CalibSimTarget {
  std::string name;
  SimTarget sim;
  bool full_board = false; // Detection needs the whole board in the image

  /** Detect the target in an image */
  std::function<CalibTargetPtr(const cv::Mat &, const timestamp_t)> detect;

  /** Add the target to the calibration problem */
  std::function<void(CalibProblem &)> add_to;

  /** Remove a corner measurement, by point id */
  std::function<void(CalibTarget &, const int)> remove_corner;
};

static CalibSimTarget make_calib_sim_target(const std::string &name,
                                            const RefineCorners &refine) {
  CalibSimTarget t;
  t.name = name;

  if (name == "aprilgrid_kaess" || name == "aprilgrid_olsen") {
    const std::string detector_type = name.substr(strlen("aprilgrid_"));
    const auto config = sim_aprilgrid_config();
    auto detector = std::make_shared<AprilGridDetector>(config);
    detector->set_detector(detector_type);
    detector->set_refine_corners(refine.aprilgrid);
    t.sim = make_sim_aprilgrid(false, detector_type, refine.aprilgrid);
    t.detect = [detector](const cv::Mat &image,
                          const timestamp_t ts) -> CalibTargetPtr {
      return detector->detect(ts, 0, image)[0];
    };
    t.add_to = [config](CalibProblem &calib) {
      calib.add_target(config, tf_vec());
    };
    t.remove_corner = [config](CalibTarget &grid, const int point_id) {
      // Point ids are tag_id * 4 + corner_index
      static_cast<AprilGrid &>(grid).remove(point_id / 4 + config.tag_id_offset,
                                            point_id % 4);
    };

  } else if (name == "checkerboard") {
    const auto config = sim_checkerboard_config();
    auto detector =
        std::make_shared<CheckerboardDetector>(config, refine.checkerboard);
    t.sim = make_sim_checkerboard(refine.checkerboard);
    t.full_board = true;
    t.detect = [detector](const cv::Mat &image,
                          const timestamp_t ts) -> CalibTargetPtr {
      return detector->detect(ts, 0, image);
    };
    t.add_to = [config](CalibProblem &calib) {
      calib.add_target(config, tf_vec());
    };
    t.remove_corner = [](CalibTarget &board, const int point_id) {
      static_cast<CornerTarget &>(board).remove(point_id);
    };

  } else if (name == "charuco") {
    const auto config = sim_charuco_config();
    auto detector = std::make_shared<CharucoDetector>(config, refine.charuco);
    t.sim = make_sim_charuco(refine.charuco);
    t.detect = [detector](const cv::Mat &image,
                          const timestamp_t ts) -> CalibTargetPtr {
      return detector->detect(ts, 0, image);
    };
    t.add_to = [config](CalibProblem &calib) {
      calib.add_target(config, tf_vec());
    };
    t.remove_corner = [](CalibTarget &board, const int point_id) {
      static_cast<CornerTarget &>(board).remove(point_id);
    };

  } else {
    FATAL("Unknown target [%s], must be one of [aprilgrid_kaess, "
          "aprilgrid_olsen, checkerboard, charuco]!",
          name.c_str());
  }

  return t;
}

/******************************************************************************
 * CALIBRATION
 *****************************************************************************/

struct CaseResult {
  Case c;
  std::string target;
  std::string fit_model; // Camera model of the calibration
  std::string init;      // Initial intrinsic guess: "default" or "near_truth"
  bool ok = false;
  std::string note;
  int num_views = 0;       // Views where the target was detected
  int num_views_used = 0;  // Views used by the calibrator
  int num_corners = 0;     // Corners used by the calibrator
  int num_outliers = 0;    // Corners with an error > 5 px
  VecX gt = zeros(8, 1);   // Ground truth intrinsic
  VecX est = zeros(8, 1);  // Estimated intrinsic
  double model_rmse = NAN; // Projection difference to ground truth [px]
  double model_max = NAN;
  double reproj_rmse = NAN; // Calibrator reprojection error [px]
  double solve_s = NAN;
};

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

/** Calibrate from the detected targets with an initial intrinsic guess */
static CaseResult calibrate(const Settings &s,
                            CaseResult r,
                            const std::string &init,
                            const std::string &fit_model,
                            const CalibSimTarget &target,
                            const std::vector<CalibTargetPtr> &views,
                            const CameraGeometry &gt_camera,
                            const double max_theta) {
  r.init = init;
  r.fit_model = fit_model;
  if (r.num_views < 5) {
    r.note = "fewer than 5 views with a detected target";
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
    calib.add_camera(0, fit_model, gt_camera.resolution, initial, tf_vec());
    target.add_to(calib);
    for (const auto &view : views) {
      calib.add_camera_measurement(view->ts, 0, view);
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
    const CameraGeometry est_camera{0, fit_model, gt_camera.resolution, r.est};
    model_error(gt_camera, est_camera, max_theta, r.model_rmse, r.model_max);
    r.ok = std::isfinite(r.model_rmse);

  } catch (const std::exception &e) {
    r.note = std::string("exception: ") + e.what();
  }

  return r;
}

/** Render and detect a calibration dataset, and calibrate from it */
static std::vector<CaseResult> run_case(const Settings &s,
                                        const Case &c,
                                        const CalibSimTarget &target,
                                        const std::vector<CameraPose> &poses,
                                        const CameraGeometry &gt_camera,
                                        const Renderer &renderer,
                                        std::ofstream &corners_csv) {
  CaseResult r;
  r.c = c;
  r.target = target.name;
  r.gt = gt_camera.intrinsic;

  // Fit models by corner limit, widest first. Corners beyond a limit are
  // removed from the detections after the models of the wider limits have been
  // calibrated.
  std::map<double, std::vector<std::string>, std::greater<double>> groups;
  for (const auto &fit_model : s.fit_models) {
    groups[s.limit(c, fit_model)].push_back(fit_model);
  }
  const double widest = groups.begin()->first;

  // Render and detect
  struct Corner {
    int id;
    double theta; // Angle off the optical axis [deg]
    double error; // Distance to the ground truth projection [px]
  };
  const auto texture = target.sim.render(s.render.margin, s.render.ppm);
  const auto gt_points = target.sim.corners;
  std::mt19937 noise_gen{s.seed + 1};
  std::vector<CalibTargetPtr> views;
  std::vector<std::vector<Corner>> view_corners;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto image = renderer.render(texture, poses[i].T_TC, noise_gen);
    const auto detection = target.detect(image, (i + 1) * 100000000);

    // Ground truth projections, in order to drop corners that the camera
    // model cannot represent (too far off the optical axis) and to count
    // outliers
    std::vector<int> point_ids;
    Vec2s keypoints;
    Vec3s object_points;
    detection->get_measurements(point_ids, keypoints, object_points);

    const Mat4 T_CT = poses[i].T_TC.inverse();
    std::vector<Corner> corners;
    std::vector<Vec2> good;
    std::vector<Vec2> bad;
    for (size_t j = 0; j < point_ids.size(); ++j) {
      const Vec3 p_C = tf_point(T_CT, gt_points.at(point_ids[j]));
      const double theta = rad2deg(acos(p_C.z() / p_C.norm()));
      if (theta > widest) {
        target.remove_corner(*detection, point_ids[j]);
        continue;
      }

      Vec2 z_gt;
      gt_camera.project(p_C, z_gt);
      const double error = (keypoints[j] - z_gt).norm();
      corners.push_back(Corner{point_ids[j], theta, error});
      (error > 5.0 ? bad : good).push_back(keypoints[j]);
      corners_csv << c.name << "," << target.name << "," << keypoints[j].x()
                  << "," << keypoints[j].y() << "\n";
    }
    views.push_back(detection);
    view_corners.push_back(corners);

    if (s.save_samples && i < 6) {
      char name[64];
      snprintf(name, sizeof(name), "view_%02zu.png", i);
      const auto dir =
          s.output_dir / "samples" / "views" / (c.name + "_" + target.name);
      fs::create_directories(dir);
      cv::imwrite(dir / name, draw_overlay(image, good, bad));
    }
  }

  // Calibrate with every camera model. The near truth initial guess is the
  // ground truth camera, which only a calibration with the same model has.
  std::vector<CaseResult> results;
  for (const auto &[limit, fit_models] : groups) {
    // Drop the corners beyond the limit, and count what is left
    r.num_views = 0;
    r.num_outliers = 0;
    std::vector<CalibTargetPtr> used;
    for (size_t i = 0; i < views.size(); ++i) {
      for (const auto &corner : view_corners[i]) {
        if (corner.theta > limit) {
          target.remove_corner(*views[i], corner.id);
        } else if (corner.error > 5.0) {
          r.num_outliers++;
        }
      }
      if (views[i]->get_num_detected() >= 10) {
        r.num_views++;
        used.push_back(views[i]);
      }
    }

    for (const auto &fit_model : fit_models) {
      for (const auto &init : s.inits) {
        if (init == "near_truth" && fit_model != c.model) {
          continue;
        }
        results.push_back(
            calibrate(s, r, init, fit_model, target, used, gt_camera, limit));
      }
    }
  }
  return results;
}

/******************************************************************************
 * OUTPUT
 *****************************************************************************/

static void save_results(const fs::path &path,
                         const std::vector<CaseResult> &results) {
  std::ofstream f{path};
  f << "case,model,fov,level,target,fit_model,init,ok,note,views,views_used,"
       "corners,outlier_corners";
  const char *names[4] = {"fx", "fy", "cx", "cy"};
  for (const auto *suffix : {"_gt", "_est"}) {
    for (const auto *name : names) {
      f << "," << name << suffix;
    }
  }
  f << ",model_rmse_px,model_max_px,reproj_rmse_px,solve_s,est_params\n";

  for (const auto &r : results) {
    f << r.c.name << "," << r.c.model << "," << r.c.fov << "," << r.c.level
      << "," << r.target << "," << r.fit_model << "," << r.init << "," << r.ok
      << ",\"" << r.note << "\"," << r.num_views << "," << r.num_views_used
      << "," << r.num_corners << "," << r.num_outliers;
    for (const auto &v : {r.gt, r.est}) {
      for (int i = 0; i < 4; ++i) {
        f << "," << v(i);
      }
    }
    f << "," << r.model_rmse << "," << r.model_max << "," << r.reproj_rmse
      << "," << r.solve_s << ",\"";
    for (int i = 0; i < r.est.size(); ++i) {
      f << (i ? " " : "") << r.est(i);
    }
    f << "\"\n";
  }
}

static void save_target_samples(const Settings &s) {
  const auto dir = s.output_dir / "samples" / "targets";
  fs::create_directories(dir);

  const std::vector<SimTarget> targets{make_sim_aprilgrid(true, "kaess"),
                                       make_sim_aprilgrid(false, "kaess"),
                                       make_sim_checkerboard(),
                                       make_sim_charuco()};
  for (const auto &target : targets) {
    cv::imwrite(dir / (target.name + ".png"), target.render(s.render.margin, s.render.ppm));
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
  corners_csv << "case,target,u,v\n";

  std::vector<CalibSimTarget> targets;
  for (const auto &name : s.targets) {
    targets.push_back(make_calib_sim_target(name, s.refine_corners));
  }

  std::vector<CaseResult> results;
  for (const auto &c : s.cases) {
    if (c.skip_reason.empty() == false) {
      printf("%-26s skipped: %s\n", c.name.c_str(), c.skip_reason.c_str());
      for (const auto &target : targets) {
        for (const auto &fit_model : s.fit_models) {
          for (const auto &init : s.inits) {
            if (init == "near_truth" && fit_model != c.model) {
              continue;
            }
            CaseResult r;
            r.c = c;
            r.target = target.name;
            r.fit_model = fit_model;
            r.init = init;
            r.note = "skipped: " + c.skip_reason;
            results.push_back(r);
          }
        }
      }
      continue;
    }

    // Camera and renderer (shared by the targets)
    const auto gt_camera = make_camera(c.spec);
    const Renderer renderer{s.render, gt_camera};
    const double pose_theta = s.limit(c, c.model);
    const PoseSampling sampling{s.seed,
                                s.num_views,
                                c.dist_range.x(),
                                c.dist_range.y(),
                                s.max_tilt};

    // Sample render, looking straight at an AprilGrid
    if (s.save_samples) {
      const auto dir = s.output_dir / "samples" / "cameras";
      fs::create_directories(dir);
      const auto board = make_sim_aprilgrid(false, "kaess");
      const Vec3 center{board.width / 2.0, board.height / 2.0, 0.0};
      Mat4 T_TC = tf(rotx(M_PI), center + Vec3{0.0, 0.0, c.dist_range.x()});
      std::mt19937 gen{0};
      cv::imwrite(dir / (c.name + ".png"),
                  renderer.render(board.render(s.render.margin, s.render.ppm), T_TC, gen));
    }

    for (const auto &target : targets) {
      const auto poses = target.full_board
                             ? sample_poses_full_board(sampling,
                                                       target.sim,
                                                       gt_camera,
                                                       pose_theta)
                             : sample_poses_spread(sampling,
                                                   target.sim.width,
                                                   target.sim.height,
                                                   gt_camera,
                                                   pose_theta);
      const auto case_results =
          run_case(s, c, target, poses, gt_camera, renderer, corners_csv);
      for (const auto &r : case_results) {
        printf("%-26s %-15s fit %-6.6s %-10s views %3d/%3d corners %6d "
               "outliers %4d  ",
               c.name.c_str(),
               target.name.c_str(),
               r.fit_model.c_str(),
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
