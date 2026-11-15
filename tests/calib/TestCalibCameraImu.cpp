#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "core/SO3.hpp"
#include "calib/CalibCameraImu.hpp"
#include "calib/CalibInit.hpp"
#include "imu/ImuPreintegrate.hpp"
#include "sim/SimCalib.hpp"
#include "timeline/timeline.hpp"

#define TEST_EUROC "/data/euroc/imu_april"
#define TEST_CONFIG TEST_DATA "/calib_camera_imu.yaml"

namespace cartesian {

const fs::path imu_path = TEST_EUROC "/mav0/imu0/data.csv";
const fs::path cam0_dir = TEST_EUROC "/mav0/cam0/data";
const fs::path cam1_dir = TEST_EUROC "/mav0/cam0/data";

static Timeline form_timeline(const timestamp_t camera_time_delay = 0) {
  Timeline timeline;

  // -- Load imu data (original timestamps)
  {
    FILE *imu_csv = fopen(imu_path.c_str(), "r");
    if (imu_csv == nullptr) {
      FATAL("IMU data not found [%s]!", imu_path.c_str());
    }
    skip_line(imu_csv);

    const char *scan_format = "%ld,%lf,%lf,%lf,%lf,%lf,%lf";
    const int num_rows = file_rows(imu_path) - 1;
    for (int i = 0; i < num_rows; i++) {
      timestamp_t ts;
      Vec3 imu_acc;
      Vec3 imu_gyr;
      const int retval = fscanf(imu_csv,
                                scan_format,
                                &ts,
                                &imu_gyr.data()[0],
                                &imu_gyr.data()[1],
                                &imu_gyr.data()[2],
                                &imu_acc.data()[0],
                                &imu_acc.data()[1],
                                &imu_acc.data()[2]);
      if (retval != 7) {
        FATAL("Failed to parse data line %d in [%s]\n", i, imu_path.c_str());
      }
      timeline.add(ts, imu_acc, imu_gyr);
    }
  }

  // -- Load camera data (timestamps shifted forward by camera_time_delay)
  // Shifting camera timestamps forward simulates the camera capturing
  // camera_time_delay after its reported timestamp, creating a real
  // inconsistency between the visual pose interval and the IMU integration
  // interval that the time delay parameter must resolve.  We also update
  // the target's internal timestamp to match the shifted timeline timestamp
  // so that CalibCameraImu::add_measurement's ts-check assertion passes.
  auto load_targets = [&](const int camera_id, const fs::path data_dir) {
    for (const auto &target_csv : fs::directory_iterator(data_dir)) {
      const std::string ts_str = target_csv.path().stem();
      const timestamp_t ts = std::stoull(ts_str);
      const timestamp_t ts_shifted = ts + camera_time_delay;

      auto calib_target = AprilGrid::load(target_csv);
      calib_target->ts = ts_shifted;
      timeline.add(ts_shifted, camera_id, calib_target);
    }
  };
  load_targets(0, TEST_EUROC "/mav0/target0/cam0");
  load_targets(1, TEST_EUROC "/mav0/target0/cam1");

  return timeline;
}

static AprilGridConfig setup_target_config() {
  AprilGridConfig target_config;
  target_config.target_id = 0;
  target_config.tag_rows = 6;
  target_config.tag_cols = 6;
  target_config.tag_size = 0.088;
  target_config.tag_spacing = 0.3;
  target_config.tag_id_offset = 0;
  return target_config;
}

static CalibCameraImu setup_calibrator(const AprilGridConfig &target_config,
                                       const Timeline &timeline) {
  // Calibrate
  CalibCameraImu calib;

  // -- Add target
  calib.add_target(target_config, tf_vec());

  // -- Add cam0
  {
    // Intrinsics
    const int cam0_id = 0;
    const std::string camera_model = "BrownConrady4";
    const Vec2i resolution{752, 480};
    Vec8 cam0_intrinsic;
    cam0_intrinsic[0] = 458.654;        // fx
    cam0_intrinsic[1] = 457.296;        // fy
    cam0_intrinsic[2] = 367.215;        // cx
    cam0_intrinsic[3] = 248.375;        // cy
    cam0_intrinsic[4] = -0.28340811;    // k1
    cam0_intrinsic[5] = 0.07395907;     // k2
    cam0_intrinsic[6] = 0.00019359;     // p1
    cam0_intrinsic[7] = 1.76187114e-05; // p2

    // Extrinsics
    Vec7 cam0_extrinsic;
    cam0_extrinsic[0] = 0.0;
    cam0_extrinsic[1] = 0.0;
    cam0_extrinsic[2] = 0.0;
    cam0_extrinsic[3] = 0.0;
    cam0_extrinsic[4] = 0.0;
    cam0_extrinsic[5] = 0.0;
    cam0_extrinsic[6] = 1.0;

    // Add camera
    calib.add_camera(cam0_id,
                     camera_model,
                     resolution,
                     cam0_intrinsic,
                     cam0_extrinsic);
  }

  // -- Add cam1
  {
    // Intrinsics
    const int cam1_id = 1;
    const std::string camera_model = "BrownConrady4";
    const Vec2i resolution{752, 480};
    Vec8 cam1_intrinsic;
    cam1_intrinsic[0] = 457.587;         // fx
    cam1_intrinsic[1] = 456.134;         // fy
    cam1_intrinsic[2] = 379.999;         // cx
    cam1_intrinsic[3] = 255.238;         // cy
    cam1_intrinsic[4] = -0.28368365;     // k1
    cam1_intrinsic[5] = 0.07451284;      // k2
    cam1_intrinsic[6] = -0.00010473;     // p1
    cam1_intrinsic[7] = -3.55590700e-05; // p2

    // Extrinsics
    Vec7 cam1_extrinsic = zeros(7, 1);
    cam1_extrinsic[0] = 0.109935;
    cam1_extrinsic[1] = -0.000336639;
    cam1_extrinsic[2] = 0.000643293;
    cam1_extrinsic[3] = 0.00728166;
    cam1_extrinsic[4] = -0.00287444;
    cam1_extrinsic[5] = 0.00119855;
    cam1_extrinsic[6] = 0.999969;

    // Add camera
    calib.add_camera(cam1_id,
                     camera_model,
                     resolution,
                     cam1_intrinsic,
                     cam1_extrinsic);
  }

  // -- Add Imu
  {
    ImuParams imu_params;
    imu_params.imu_id = 0;
    imu_params.noise_acc = 0.08;
    imu_params.noise_gyr = 0.004;
    imu_params.noise_ba = 0.00004;
    imu_params.noise_bg = 2.0e-6;

    const Mat4 &T_cam0_imu0 =
        CalibInit::initialize_camera_imu_extrinsic(timeline,
                                                   calib.camera_geometries,
                                                   imu_params);

    calib.add_imu(imu_params.imu_id, imu_params, tf_vec(T_cam0_imu0));
  }

  // Add to calibration problem
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto &cam_id = target_event->camera_id;
        const auto &calib_target = target_event->calib_target;
        calib.add_measurement(ts, cam_id, calib_target);
      }

      if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
        calib.add_measurement(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }

  return calib;
}

TEST(CalibCameraImu, test_euroc) {
  // Calibrate
  const auto target_config = setup_target_config();
  const auto timeline = form_timeline();
  CalibCameraImu calib = setup_calibrator(target_config, timeline);

  // Solve
  calib.solve();

  // Debug
  bool debug = false;
  if (debug) {
    const Vec3i red{255.0, 0.0, 0.0};
    const Vec3i green{0.0, 255.0, 0.0};
    const Vec3i blue{0.0, 0.0, 255.0};
    Logger log;

    // -- Setup plots
    log.init_series_line("imu/acc/x", red, 1.0f);
    log.init_series_line("imu/acc/y", green, 1.0f);
    log.init_series_line("imu/acc/z", blue, 1.0f);
    log.init_series_line("imu/gyr/x", red, 1.0f);
    log.init_series_line("imu/gyr/y", green, 1.0f);
    log.init_series_line("imu/gyr/z", blue, 1.0f);

    // -- Log images
    for (const auto &entry : fs::directory_iterator(cam0_dir)) {
      const auto image_path = entry.path();
      const timestamp_t ts = stol(image_path.stem().string().c_str());
      const auto image = cv::imread(image_path.string());
      log.log_image("/cam0/image", ts, image);
    }

    for (const auto &entry : fs::directory_iterator(cam1_dir)) {
      const auto image_path = entry.path();
      const timestamp_t ts = stol(image_path.stem().string().c_str());
      const auto image = cv::imread(image_path.string());
      log.log_image("/cam1/image", ts, image);
    }

    // -- Log imu data
    for (size_t k = 0; k < calib.imu_data[0].timestamps.size(); ++k) {
      const timestamp_t ts = calib.imu_data[0].timestamps[k];
      const Vec3 &acc = calib.imu_data[0].acc_data[k];
      const Vec3 &gyr = calib.imu_data[0].gyr_data[k];

      log.log_scalar("imu/acc/x", ts, acc.x());
      log.log_scalar("imu/acc/y", ts, acc.y());
      log.log_scalar("imu/acc/z", ts, acc.z());
      log.log_scalar("imu/gyr/x", ts, gyr.x());
      log.log_scalar("imu/gyr/y", ts, gyr.y());
      log.log_scalar("imu/gyr/z", ts, gyr.z());
    }

    // -- Log poses
    std::map<timestamp_t, Mat4> sensor_poses;
    std::map<timestamp_t, Mat4> cam0_poses;
    std::map<timestamp_t, Mat4> cam1_poses;
    const Mat4 T_WT0 = tf(calib.target_pose);
    const Mat4 T_SC0 = tf(calib.imu_geometries[0]->extrinsic).inverse();
    const Mat4 T_C0C1 = tf(calib.camera_geometries[1]->extrinsic);
    for (const auto &[ts, pose] : calib.poses) {
      const Mat4 T_WS = tf(pose);
      sensor_poses[ts] = T_WS;
      cam0_poses[ts] = T_WS * T_SC0;
      cam1_poses[ts] = T_WS * T_SC0 * T_C0C1;
    }
    log.log_poses("sensor_poses", sensor_poses, 0.1);
    log.log_poses("cam0_poses", cam0_poses, 0.1);
    log.log_poses("cam1_poses", cam1_poses, 0.1);
    log.log_target("calib_target", target_config, T_WT0);
  }
}

TEST(CalibCameraImu, test_sim) {
  // Simulate with a known time delay
  const double gt_time_delay = 0.02; // 20 ms
  SimCalib sim;
  sim.sim_camimu_calib(10.0, "fig8-horiz", 0.5, 10.0, gt_time_delay);

  // Calibrate
  CalibCameraImu calib;

  // -- Add target
  calib.add_target(sim.target_configs[0], sim.targets.at(0).extrinsic);

  // -- Add camera
  for (auto &[camera_id, camera] : sim.cameras) {
    calib.add_camera(camera.camera_id,
                     camera.camera_model->type(),
                     camera.resolution,
                     camera.intrinsic,
                     camera.extrinsic);
  }

  // -- Add Imu
  for (auto &[imu_id, imu] : sim.imus) {
    calib.add_imu(imu_id, imu.imu_params, imu.extrinsic);
  }

  // -- Add data
  auto timeline = sim.get_timeline();
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        calib.add_measurement(ts,
                              target_event->camera_id,
                              target_event->calib_target);
      }

      if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
        calib.add_measurement(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }

  // clang-format off
  Mat4 T_C0S;
  T_C0S <<
    0.0, 1.0, 0.0, 0.1,
    1.0, 0.0, 0.0, 0.2,
    0.0, 0.0, 1.0, 0.3,
    0.0, 0.0, 0.0, 1.0;
  Vec7 imu_extrinsic = tf_vec(T_C0S);

  double *imu0_ext = calib.imu_geometries.at(0)->extrinsic.data();
  imu0_ext[0] = imu_extrinsic[0];
  imu0_ext[1] = imu_extrinsic[1];
  imu0_ext[2] = imu_extrinsic[2];
  imu0_ext[3] = imu_extrinsic[3];
  imu0_ext[4] = imu_extrinsic[4];
  imu0_ext[5] = imu_extrinsic[5];
  imu0_ext[6] = imu_extrinsic[6];
  // clang-format on

  calib.solve();

  // Verify time delay was estimated correctly
  EXPECT_NEAR(calib.time_delay_, gt_time_delay, 0.005);
}

// -----------------------------------------------------------------------
// Error-model comparison test
// -----------------------------------------------------------------------

struct ExpResult {
  double estimated_delay;
  double abs_error;
  double final_cost;
  int num_iterations;
};

static ExpResult
run_calibration(const double gt_time_delay,
                const CalibCameraImu::TimeDelayMethod td_method) {
  SimCalib sim;
  sim.sim_camimu_calib(10.0, "fig8-horiz", 0.5, 10.0, gt_time_delay);

  CalibCameraImu calib;
  calib.verbose = false;
  calib.td_method = td_method;

  calib.add_target(sim.target_configs[0], sim.targets.at(0).extrinsic);
  for (auto &[camera_id, camera] : sim.cameras) {
    calib.add_camera(camera.camera_id,
                     camera.camera_model->type(),
                     camera.resolution,
                     camera.intrinsic,
                     camera.extrinsic);
  }
  for (auto &[imu_id, imu] : sim.imus) {
    calib.add_imu(imu_id, imu.imu_params, imu.extrinsic);
  }

  auto timeline = sim.get_timeline();
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        calib.add_measurement(ts,
                              target_event->camera_id,
                              target_event->calib_target);
      }
      if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
        calib.add_measurement(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }

  // clang-format off
  Mat4 T_C0S;
  T_C0S << 0.0, 1.0, 0.0, 0.1,
           1.0, 0.0, 0.0, 0.2,
           0.0, 0.0, 1.0, 0.3,
           0.0, 0.0, 0.0, 1.0;
  // clang-format on
  Vec7 imu_extrinsic = tf_vec(T_C0S);
  double *imu0_ext = calib.imu_geometries.at(0)->extrinsic.data();
  for (int i = 0; i < 7; i++) {
    imu0_ext[i] = imu_extrinsic[i];
  }
  calib.solve();

  // Form result
  ExpResult result;
  result.estimated_delay = calib.time_delay_;
  result.abs_error = std::abs(calib.time_delay_ - gt_time_delay);
  result.final_cost = calib.final_cost;
  result.num_iterations = calib.num_iters;

  return result;
}

TEST(CalibCameraImu, test_time_delay_sim) {
  printf("\n");
  printf("  Error Model Comparison: PIXEL_VELOCITY vs POSE_INTERP\n");
  printf("  %s\n", std::string(78, '=').c_str());
  printf("  %-7s  %-10s  %-10s  %-10s  %-10s  %9s  %9s   %s\n",
         "gt[ms]",
         "pv_est[ms]",
         "pi_est[ms]",
         "pv_err[ms]",
         "pi_err[ms]",
         "pv_iter",
         "pi_iter",
         "");
  printf("  %s\n", std::string(78, '-').c_str());

  const std::vector<double> delays = {0.001, 0.002, 0.005, 0.010, 0.020};
  for (const double delay : delays) {
    const auto r2 = run_calibration(delay, CalibCameraImu::PIXEL_VELOCITY);
    const auto r3 = run_calibration(delay, CalibCameraImu::POSE_INTERP);
    printf("  %7.2f  %10.3f  %10.3f  %10.3f  %10.3f  %9d  %9d\n",
           delay * 1000,
           r2.estimated_delay * 1000.0,
           r3.estimated_delay * 1000.0,
           r2.abs_error * 1000.0,
           r3.abs_error * 1000.0,
           r2.num_iterations,
           r3.num_iterations);
  }

  printf("  %s", std::string(78, '=').c_str());
  printf("\n\n");
}

TEST(CalibCameraImu, test_time_delay_euroc) {
  printf("\n");
  printf("  EuRoC Synthetic Delay: PIXEL_VELOCITY vs POSE_INTERP\n");
  printf("  %s\n", std::string(78, '=').c_str());
  printf("  %-10s  %-10s  %-10s  %11s   %s\n",
         "model",
         "td_est[ms]",
         "cost",
         "iter",
         "");
  printf("  %s\n", std::string(78, '-').c_str());

  auto run_exp = [&](const CalibCameraImu::TimeDelayMethod method,
                     const char *label,
                     const double time_delay) {
    const auto target_config = setup_target_config();
    const timestamp_t delay_ns = static_cast<timestamp_t>(time_delay * 1e9);
    const auto shifted_timeline = form_timeline(delay_ns);
    CalibCameraImu calib = setup_calibrator(target_config, shifted_timeline);
    calib.verbose = false;
    calib.td_method = method;
    calib.max_iters = 15;
    calib.solve();

    printf("  %-10s  %10.3f  %10.2e  %11d\n",
           label,
           calib.time_delay_ * 1000.0,
           calib.final_cost,
           calib.num_iters);

    return calib;
  };

  const double td = 0.01;
  const auto calib_pv = run_exp(CalibCameraImu::PIXEL_VELOCITY, "pv", td);
  const auto calib_pi = run_exp(CalibCameraImu::POSE_INTERP, "pi", td);

  printf("  %s\n", std::string(78, '-').c_str());
  printf("  gt_delay: %.3f ms\n", td * 1000.0);
  printf("  pv_est:   %.3f ms\n", calib_pv.time_delay_ * 1000.0);
  printf("  pi_est:   %.3f ms\n", calib_pi.time_delay_ * 1000.0);
  printf("  %s", std::string(78, '=').c_str());
  printf("\n\n");

  EXPECT_NEAR(calib_pv.time_delay_, calib_pi.time_delay_, 0.010);
}

} // namespace cartesian
