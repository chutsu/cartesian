#include <gtest/gtest.h>

#include <fstream>
#include <random>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "calib/CalibCamera.hpp"
#include "sim/SimCalib.hpp"

namespace cartesian {

static void perturb_camera_intrinsic(CameraGeometryPtr &camera) {
  auto intrinsic = camera->intrinsic.data();
  intrinsic[0] += randf(0, 100);
  intrinsic[1] += randf(0, 100);
  intrinsic[2] += randf(0, 100);
  intrinsic[3] += randf(0, 100);
  intrinsic[4] = 0.0;
  intrinsic[5] = 0.0;
  intrinsic[6] = 0.0;
  intrinsic[7] = 0.0;
}

static void perturb_camera_extrinsic(CameraGeometryPtr &camera) {
  const auto phi = randf(-1.0, 1.0);
  const auto theta = randf(-1.0, 1.0);
  const auto psi = randf(-1.0, 1.0);
  const auto quat = euler2quat(Vec3{phi, theta, psi});

  auto extrinsic = camera->extrinsic.data();
  extrinsic[0] = randf(0, 0.5);
  extrinsic[1] = randf(0, 0.5);
  extrinsic[2] = randf(0, 0.5);
  extrinsic[3] = quat.x();
  extrinsic[4] = quat.y();
  extrinsic[5] = quat.z();
  extrinsic[6] = quat.w();
}

static void perturb_camera(CameraGeometryPtr &camera) {
  perturb_camera_intrinsic(camera);
  if (camera->camera_id != 0) {
    perturb_camera_extrinsic(camera);
  }
}

TEST(CalibCamera, solve_sim) {
  const double camera_rate = 10.0;
  const double sample_x = 1.0;
  const double sample_y = 1.0;
  const double sample_z = 0.3;
  const double sample_z_offset = 1.0;
  const int sample_num_x = 2;
  const int sample_num_y = 2;
  const int sample_num_z = 1;
  SimCalib sim;
  sim.sim_camera_calib(camera_rate,
                       sample_x,
                       sample_y,
                       sample_z,
                       sample_z_offset,
                       sample_num_x,
                       sample_num_y,
                       sample_num_z);

  // Setup calibration problem
  CalibCamera calib;

  // -- Add cameras
  for (const auto &[_, camera] : sim.cameras) {
    calib.add_camera(camera.camera_id,
                     camera.camera_model->type(),
                     camera.resolution,
                     camera.intrinsic,
                     camera.extrinsic);
  }

  // -- Add targets
  for (const auto &[_, config] : sim.target_configs) {
    calib.add_target(config, tf_vec());
  }

  // -- Add camera data
  auto timeline = sim.get_timeline();
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto camera_id = target_event->camera_id;
        const auto calib_target = target_event->calib_target;
        calib.add_camera_measurement(ts, camera_id, calib_target);
      }
    }
  }

  // Perturb camera intrinsic and extrinsic
  perturb_camera(calib.get_camera_geometry(0));
  perturb_camera(calib.get_camera_geometry(1));

  // Solve
  calib.solve();
}

/******************************************************************************
 * CHECKERBOARD AND CHARUCO
 *****************************************************************************/

/** Ground truth camera of the checkerboard and ChArUco tests */
static CameraGeometry make_test_camera() {
  VecX intrinsic = zeros(8, 1);
  intrinsic << 460.0, 458.0, 378.0, 238.0, -0.10, 0.02, 0.0005, -0.0005;
  return CameraGeometry{0, "BrownConrady4", Vec2i{752, 480}, intrinsic};
}

/**
 * Simulate views of a corner target: the corners are projected by the ground
 * truth camera for random poses, with some noise.
 */
template <typename T, typename Config>
static std::vector<std::shared_ptr<T>>
simulate_views(const CameraGeometry &camera,
               const Config &config,
               const int num_views) {
  // Board center and size
  const auto points = config.get_object_points();
  Vec3 center = zeros(3, 1);
  for (const auto &[id, p] : points) {
    center += p / points.size();
  }
  double extent = 0.0;
  for (const auto &[id, p] : points) {
    extent = std::max(extent, (p - center).norm());
  }

  std::mt19937 gen{0};
  std::uniform_real_distribution<double> uni{-1.0, 1.0};
  std::normal_distribution<double> noise{0.0, 0.15};

  std::vector<std::shared_ptr<T>> views;
  while (static_cast<int>(views.size()) < num_views) {
    // Camera pose, looking at the front of the board with some tilt
    const double d = (3.0 + 0.8 * uni(gen)) * extent;
    const Vec3 r_TC =
        center + Vec3{0.1 * uni(gen) * extent, 0.1 * uni(gen) * extent, d};
    const Mat3 C_TC = rotx(M_PI) * rotx(deg2rad(30.0 * uni(gen))) *
                      roty(deg2rad(30.0 * uni(gen))) *
                      rotz(deg2rad(180.0 * uni(gen)));
    const Mat4 T_CT = tf(C_TC, r_TC).inverse();

    // Project corners
    const timestamp_t ts = (views.size() + 1) * 100000000;
    auto target = std::make_shared<T>(ts, 0, config);
    for (const auto &[id, p_T] : points) {
      Vec2 z;
      if (camera.project(tf_point(T_CT, p_T), z) == 0) {
        target->add(id, z + Vec2{noise(gen), noise(gen)});
      }
    }
    if (target->get_num_detected() >= 20) {
      views.push_back(target);
    }
  }

  return views;
}

template <typename T, typename Config>
static void test_solve_sim(const Config &config) {
  const auto gt = make_test_camera();
  const auto views = simulate_views<T>(gt, config, 30);

  // Calibrate from the default initial guess
  CalibCamera calib;
  calib.add_camera(0, "BrownConrady4", gt.resolution, VecX{}, tf_vec());
  calib.add_target(config, tf_vec());
  for (const auto &view : views) {
    calib.add_camera_measurement(view->ts, 0, view);
  }
  calib.solve();

  // Check the ground truth camera is recovered
  const VecX est = calib.get_camera_geometry(0)->intrinsic;
  ASSERT_EQ(calib.get_num_targets(), 1);
  ASSERT_EQ(calib.poses.size(), views.size());
  ASSERT_NEAR(est(0) / gt.intrinsic(0), 1.0, 0.01); // fx
  ASSERT_NEAR(est(1) / gt.intrinsic(1), 1.0, 0.01); // fy
  ASSERT_NEAR(est(2), gt.intrinsic(2), 1.0);        // cx
  ASSERT_NEAR(est(3), gt.intrinsic(3), 1.0);        // cy
  ASSERT_NEAR(est(4), gt.intrinsic(4), 0.02);       // k1
  ASSERT_NEAR(est(5), gt.intrinsic(5), 0.05);       // k2
}

TEST(CalibCamera, solve_checkerboard_sim) {
  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.04;
  test_solve_sim<Checkerboard>(config);
}

TEST(CalibCamera, solve_sliding_window) {
  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.04;
  const auto gt = make_test_camera();
  const int num_views = 40;
  const int window_size = 15;
  const auto views = simulate_views<Checkerboard>(gt, config, num_views);

  // Calibrate with a sliding window of views
  CalibCamera calib;
  calib.window_size = window_size;
  calib.add_camera(0, "BrownConrady4", gt.resolution, VecX{}, tf_vec());
  calib.add_target(config, tf_vec());
  for (const auto &view : views) {
    calib.add_camera_measurement(view->ts, 0, view);
  }
  calib.solve();

  // Only the last views are left in the problem
  const timestamp_t ts_oldest = views[num_views - window_size]->ts;
  ASSERT_EQ(calib.view_timestamps.size(), window_size);
  ASSERT_EQ(calib.view_timestamps.front(), ts_oldest);
  ASSERT_EQ(calib.view_timestamps.back(), views.back()->ts);
  ASSERT_EQ(calib.poses.size(), window_size);
  ASSERT_EQ(calib.resblocks.size(), window_size);
  ASSERT_EQ(calib.resblock_ids.size(), window_size);

  // The problem has the residual blocks of the views in the window
  int num_resblocks = 0;
  for (const auto &[ts, camera_resblocks] : calib.resblocks) {
    num_resblocks += camera_resblocks.at(0).size();
  }
  ASSERT_EQ(calib.problem->NumResidualBlocks(), num_resblocks);

  // Check the ground truth camera is recovered, from the views in the window
  const VecX est = calib.get_camera_geometry(0)->intrinsic;
  ASSERT_NEAR(est(0) / gt.intrinsic(0), 1.0, 0.02); // fx
  ASSERT_NEAR(est(1) / gt.intrinsic(1), 1.0, 0.02); // fy
  ASSERT_NEAR(est(2), gt.intrinsic(2), 2.0);        // cx
  ASSERT_NEAR(est(3), gt.intrinsic(3), 2.0);        // cy
  ASSERT_NEAR(est(4), gt.intrinsic(4), 0.03);       // k1
  ASSERT_NEAR(est(5), gt.intrinsic(5), 0.08);       // k2
}

TEST(CalibCamera, solve_sliding_window_all_views_fit) {
  // A window larger than the number of views is the same as no window
  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.04;
  const auto gt = make_test_camera();
  const auto views = simulate_views<Checkerboard>(gt, config, 20);

  CalibCamera calib;
  calib.window_size = 100;
  calib.add_camera(0, "BrownConrady4", gt.resolution, VecX{}, tf_vec());
  calib.add_target(config, tf_vec());
  for (const auto &view : views) {
    calib.add_camera_measurement(view->ts, 0, view);
  }
  calib.solve();

  ASSERT_EQ(calib.view_timestamps.size(), views.size());
  ASSERT_EQ(calib.poses.size(), views.size());
  const VecX est = calib.get_camera_geometry(0)->intrinsic;
  ASSERT_NEAR(est(0) / gt.intrinsic(0), 1.0, 0.01); // fx
  ASSERT_NEAR(est(1) / gt.intrinsic(1), 1.0, 0.01); // fy
}

TEST(CalibCamera, solve_charuco_sim) {
  CharucoConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.07;
  config.marker_size = 0.056;
  config.dictionary = "DICT_5X5_250";
  test_solve_sim<Charuco>(config);
}

/** RMS pixel difference between two cameras, for the rays of `gt` through the
 * pixels of the image inside `margin` pixels of the border */
static double projection_error(const CameraGeometry &gt,
                               const CameraGeometry &est,
                               const double margin = 0.0) {
  double sse = 0.0;
  int n = 0;
  for (double v = margin; v < gt.resolution.y() - margin; v += 8.0) {
    for (double u = margin; u < gt.resolution.x() - margin; u += 8.0) {
      Vec3 ray;
      gt.back_project(Vec2{u, v}, ray);
      Vec2 z;
      est.project(ray, z);
      sse += (z - Vec2{u, v}).squaredNorm();
      n++;
    }
  }
  return sqrt(sse / n);
}

TEST(CalibCamera, solve_ucm_sim) {
  // A wide angle UnifiedCameraModel camera (about 120 degrees), calibrated from
  // the default initial guess
  VecX intrinsic(9);
  intrinsic << 607.0, 610.0, 378.0, 238.0, 0.9, -0.05, 0.01, 0.0005, -0.0005;
  const CameraGeometry gt{0, "UnifiedCameraModel", Vec2i{752, 480}, intrinsic};

  CheckerboardConfig config;
  config.target_id = 0;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.04;
  const auto views = simulate_views<Checkerboard>(gt, config, 30);

  CalibCamera calib;
  calib.add_camera(0, "UnifiedCameraModel", gt.resolution, VecX{}, tf_vec());
  calib.add_target(config, tf_vec());
  for (const auto &view : views) {
    calib.add_camera_measurement(view->ts, 0, view);
  }
  calib.solve();

  const auto &est = *calib.get_camera_geometry(0);
  ASSERT_EQ(est.intrinsic.size(), 9);

  // The fit reaches the noise of the corners (0.15 px per axis). The focal
  // length, xi and the radial distortion trade off against each other when the
  // views cover part of the image, so the parameters differ from the truth,
  // but the projection stays close.
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
  ASSERT_GT(n, 1000);
  ASSERT_LT(sqrt(sse / n), 0.3);
  ASSERT_LT(projection_error(gt, est), 1.0);
}

TEST(CalibCamera, solve_checkerboard_and_aprilgrid) {
  // Targets of different types can be used in the same problem
  CalibCamera calib;
  CheckerboardConfig checkerboard;
  checkerboard.target_id = 0;
  checkerboard.rows = 8;
  checkerboard.cols = 9;
  checkerboard.square_size = 0.04;
  AprilGridConfig aprilgrid{1, 6, 6, 0.08, 0.25, 0};
  calib.add_target(checkerboard, tf_vec());
  calib.add_target(aprilgrid, tf_vec());
  ASSERT_EQ(calib.get_num_targets(), 2);
  ASSERT_EQ(calib.get_target_geometry(0)->points.size(), 7 * 8);
  ASSERT_EQ(calib.get_target_geometry(1)->points.size(), 6 * 6 * 4);

  // Target ids are unique across target types
  CharucoConfig charuco;
  charuco.target_id = 1;
  charuco.rows = 8;
  charuco.cols = 9;
  charuco.square_size = 0.07;
  charuco.marker_size = 0.056;
  ASSERT_ANY_THROW(calib.add_target(charuco, tf_vec()));
}

/******************************************************************************
 * DATASET
 *****************************************************************************/

/** Make a dataset (images and config file) with views of a board */
static fs::path make_dataset(const std::string &name,
                             const std::string &target_yaml,
                             const cv::Mat &board,
                             const int num_views) {
  const fs::path dir = fs::temp_directory_path() / name;
  fs::remove_all(dir);
  fs::create_directories(dir / "cam0" / "data");

  // Warp the board to a different quad in every image
  const std::vector<cv::Point2f> src{{0.f, 0.f},
                                     {(float) board.cols, 0.f},
                                     {(float) board.cols, (float) board.rows},
                                     {0.f, (float) board.rows}};
  for (int i = 0; i < num_views; ++i) {
    const float j = 20.0f * i;
    const std::vector<cv::Point2f> dst{{150.f + j, 60.f},
                                       {600.f - j, 80.f + j},
                                       {580.f, 420.f - j},
                                       {170.f + j, 400.f}};
    cv::Mat image;
    cv::warpPerspective(board,
                        image,
                        cv::getPerspectiveTransform(src, dst),
                        cv::Size{752, 480},
                        cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT,
                        cv::Scalar{255});
    cv::imwrite(dir / "cam0" / "data" /
                    (std::to_string(100000000 * (i + 1)) + ".png"),
                image);
  }

  std::ofstream config{dir / "config.yaml"};
  config << "data_path: \"" << dir.string() << "\"\n\n";
  config << target_yaml << "\n";
  config << "cam0:\n";
  config << "  resolution: [752, 480]\n";
  config << "  camera_model: \"BrownConrady4\"\n";
  return dir;
}

static void test_load_dataset(const std::string &name,
                              const std::string &target_yaml,
                              const cv::Mat &board,
                              const std::string &target_type,
                              const int num_corners) {
  const int num_views = 4;
  const auto dir = make_dataset(name, target_yaml, board, num_views);

  // Detects the target in every image, and saves them to disk
  CalibProblem problem{(dir / "config.yaml").string()};
  ASSERT_EQ(problem.get_num_targets(), 1);
  ASSERT_EQ(problem.camera_data.at(0).size(), num_views);
  for (const auto &[ts, targets] : problem.camera_data.at(0)) {
    ASSERT_EQ(targets.at(0)->target_type, target_type);
    ASSERT_EQ(targets.at(0)->get_num_detected(), num_corners);
  }
  ASSERT_TRUE(fs::exists(dir / "target0" / "cam0" / "100000000.csv"));

  // Loads the saved targets the next time
  fs::remove(dir / "cam0" / "data" / "100000000.png");
  fs::remove(dir / "cam0" / "data" / "200000000.png");
  CalibProblem cached{(dir / "config.yaml").string()};
  ASSERT_EQ(cached.camera_data.at(0).size(), num_views - 2);
  for (const auto &[ts, targets] : cached.camera_data.at(0)) {
    ASSERT_EQ(targets.at(0)->target_type, target_type);
    ASSERT_EQ(targets.at(0)->get_num_detected(), num_corners);
  }

  fs::remove_all(dir);
}

TEST(CalibCamera, load_dataset_checkerboard) {
  CheckerboardConfig config;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.05;

  const int sq = 60;
  cv::Mat board(config.rows * sq, config.cols * sq, CV_8UC1, cv::Scalar{255});
  for (int r = 0; r < config.rows; ++r) {
    for (int c = 0; c < config.cols; ++c) {
      if ((r + c) % 2 == 0) {
        cv::rectangle(board,
                      cv::Rect{c * sq, r * sq, sq, sq},
                      cv::Scalar{0},
                      cv::FILLED);
      }
    }
  }
  cv::copyMakeBorder(board,
                     board,
                     sq,
                     sq,
                     sq,
                     sq,
                     cv::BORDER_CONSTANT,
                     cv::Scalar{255});

  const std::string yaml = "target0:\n"
                           "  target_type: \"checkerboard\"\n"
                           "  rows: 8\n"
                           "  cols: 9\n"
                           "  square_size: 0.05\n";
  test_load_dataset("test_calib_camera_checkerboard",
                    yaml,
                    board,
                    "checkerboard",
                    config.get_num_corners());
}

TEST(CalibCamera, load_dataset_charuco) {
  CharucoConfig config;
  config.rows = 8;
  config.cols = 9;
  config.square_size = 0.07;
  config.marker_size = 0.056;

  const int sq = 80;
  cv::Mat board;
  charuco_board(config)
      .generateImage(cv::Size{config.cols * sq, config.rows * sq}, board, 0, 1);
  cv::copyMakeBorder(board,
                     board,
                     sq,
                     sq,
                     sq,
                     sq,
                     cv::BORDER_CONSTANT,
                     cv::Scalar{255});

  const std::string yaml = "target0:\n"
                           "  target_type: \"charuco\"\n"
                           "  rows: 8\n"
                           "  cols: 9\n"
                           "  square_size: 0.07\n"
                           "  marker_size: 0.056\n"
                           "  dictionary: \"DICT_5X5_250\"\n";
  test_load_dataset("test_calib_camera_charuco",
                    yaml,
                    board,
                    "charuco",
                    config.get_num_corners());
}

} // namespace cartesian
