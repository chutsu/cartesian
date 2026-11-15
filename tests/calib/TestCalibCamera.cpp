#include <gtest/gtest.h>

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

} // namespace cartesian
