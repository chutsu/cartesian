#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "calib/AprilGrid.hpp"
#include "calib/CalibInit.hpp"
#include "sim/SimCalib.hpp"

#define TEST_EUROC "/data/euroc/imu_april"
#define TEST_CONFIG TEST_DATA "/calib_camera_imu.yaml"

namespace cartesian {

const fs::path imu_path = TEST_EUROC "/mav0/imu0/data.csv";
const fs::path cam0_dir = TEST_EUROC "/mav0/cam0/data";
const fs::path cam1_dir = TEST_EUROC "/mav0/cam0/data";

static Timeline form_timeline() {
  Timeline timeline;

  // -- Load imu data
  {
    // Open IMU data.csv
    FILE *imu_csv = fopen(imu_path.c_str(), "r");
    if (imu_csv == nullptr) {
      FATAL("IMU data not found [%s]!", imu_path.c_str());
    }
    skip_line(imu_csv); // Skip header

    // Parse csv file
    const char *scan_format = "%ld,%lf,%lf,%lf,%lf,%lf,%lf";
    const int num_rows = file_rows(imu_path) - 1;
    for (int i = 0; i < num_rows; i++) {
      // Parse line
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

  // -- Load camera data
  auto load_targets = [&](const int camera_id, const fs::path data_dir) {
    for (const auto &target_csv : fs::directory_iterator(data_dir)) {
      const std::string ts_str = target_csv.path().stem();
      const timestamp_t ts = std::stoull(ts_str);
      timeline.add(ts, camera_id, AprilGrid::load(target_csv));
    }
  };
  load_targets(0, TEST_EUROC "/mav0/target0/cam0");
  load_targets(1, TEST_EUROC "/mav0/target0/cam1");

  return timeline;
}

static ImuParams setup_imu_params() {
  ImuParams imu_params;
  imu_params.imu_id = 0;
  imu_params.noise_acc = 0.08;
  imu_params.noise_gyr = 0.004;
  imu_params.noise_ba = 0.00004;
  imu_params.noise_bg = 2.0e-6;
  return imu_params;
}

static std::shared_ptr<CameraGeometry> setup_camera_geometry() {
  const int cam_id = 0;
  const std::string cam_model = "BrownConrady4";
  const Vec2i &cam_res{752, 480};
  // -- Intrinsics
  Vec8 cam0_int;
  cam0_int[0] = 458.654;        // fx
  cam0_int[1] = 457.296;        // fy
  cam0_int[2] = 367.215;        // cx
  cam0_int[3] = 248.375;        // cy
  cam0_int[4] = -0.28340811;    // k1
  cam0_int[5] = 0.07395907;     // k2
  cam0_int[6] = 0.00019359;     // p1
  cam0_int[7] = 1.76187114e-05; // p2
  // -- Extrinsics
  Vec7 cam0_ext = tf_vec();
  // -- Camera geometry
  auto cam0_geom = std::make_shared<CameraGeometry>(cam_id,
                                                    cam_model,
                                                    cam_res,
                                                    cam0_int,
                                                    cam0_ext);
  return cam0_geom;
}

TEST(CalibInit, initialize_intrinsics) {
  SimCalib sim;
  sim.sim_camera_calib();

  // Build camera data from synthetic observations (camera 0 only)
  std::map<int, CalibTargetData> camera_data;
  for (const auto &[ts, cam_map] : sim.camera_views) {
    for (const auto &[cam_id, target_map] : cam_map) {
      if (cam_id != 0) {
        continue;
      }

      for (const auto &[tgt_id, target] : target_map) {
        camera_data[cam_id][ts][tgt_id] = target;
      }
    }
  }

  // Perturb ground-truth intrinsics as initial guess
  const auto &gt_cam = sim.cameras.at(0);
  Vec8 init_intrinsic = gt_cam.intrinsic;
  init_intrinsic[0] += 50.0; // fx
  init_intrinsic[1] += 50.0; // fy
  init_intrinsic[2] += 20.0; // cx
  init_intrinsic[3] += 20.0; // cy
  init_intrinsic[4] = 0.0;   // k1
  init_intrinsic[5] = 0.0;   // k2
  init_intrinsic[6] = 0.0;   // p1
  init_intrinsic[7] = 0.0;   // p2

  auto cam0_geom = std::make_shared<CameraGeometry>(gt_cam.camera_id,
                                                    gt_cam.camera_model->type(),
                                                    gt_cam.resolution,
                                                    init_intrinsic,
                                                    gt_cam.extrinsic);
  std::map<int, std::shared_ptr<CameraGeometry>> cam_geoms;
  cam_geoms[0] = cam0_geom;

  // Initialize intrinsics
  CalibInit::initialize_camera_intrinsics(camera_data,
                                          sim.target_configs,
                                          cam_geoms,
                                          false);

  // Check intrinsics converged to ground truth
  const Vec8 &gt = gt_cam.intrinsic;
  const Vec8 &est = cam0_geom->intrinsic;

  EXPECT_NEAR(est[0], gt[0], 1.0);  // fx
  EXPECT_NEAR(est[1], gt[1], 1.0);  // fy
  EXPECT_NEAR(est[2], gt[2], 5.0);  // cx
  EXPECT_NEAR(est[3], gt[3], 5.0);  // cy
  EXPECT_NEAR(est[4], gt[4], 0.05); // k1
  EXPECT_NEAR(est[5], gt[5], 0.01); // k2
  EXPECT_NEAR(est[6], gt[6], 0.01); // p1
  EXPECT_NEAR(est[7], gt[7], 0.01); // p2
}

TEST(CalibInit, initialize_extrinsics) {
  const Timeline &timeline = form_timeline();
  const auto &imu_params = setup_imu_params();
  const auto &cam0_geom = setup_camera_geometry();

  std::map<int, std::shared_ptr<CameraGeometry>> cam_geoms;
  cam_geoms[0] = cam0_geom;

  const Mat4 &T_cam0_imu0_est =
      CalibInit::initialize_camera_imu_extrinsic(timeline, cam_geoms, imu_params);

  // clang-format off
  Mat4 T_imu0_cam0_gnd;
  T_imu0_cam0_gnd <<
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0;
  Mat4 T_cam0_imu0_gnd = T_imu0_cam0_gnd.inverse();

  print_matrix("T_cam0_imu0", T_cam0_imu0_gnd);
  print_matrix("T_cam0_imu0", T_cam0_imu0_est);
  // clang-format on
}

} // namespace cartesian
