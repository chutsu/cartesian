#include "SimCalib.hpp"

namespace cartesian {

/******************************************************************************
 * SIMULATION SCENARIOS
 *****************************************************************************/

void SimCalib::sim_camera_calib(const double camera_rate,
                                const double sample_x,
                                const double sample_y,
                                const double sample_z,
                                const double sample_z_offset,
                                const int sample_num_x,
                                const int sample_num_y,
                                const int sample_num_z) {
  setup_calib_targets();
  setup_camera_geometries();
  setup_imu_geometries();
  setup_camera_poses(camera_rate,
                     sample_x,
                     sample_y,
                     sample_z,
                     sample_z_offset,
                     sample_num_x,
                     sample_num_y,
                     sample_num_z);
  simulate_camera_views();
}

void SimCalib::sim_camimu_calib(const double camera_rate,
                                const std::string traj_type,
                                const double R,
                                const double T,
                                const double td_s) {
  setup_calib_targets();
  setup_camera_geometries();
  setup_imu_geometries();
  setup_camera_poses(camera_rate, traj_type, R, T, td_s);
  simulate_camera_views();
}

/******************************************************************************
 * SETUP
 *****************************************************************************/

void SimCalib::setup_calib_targets() {
  // Add target config
  const int target_id = 0;
  const int tag_rows = 8;
  const int tag_cols = 10;
  const double tag_size = 0.08;
  const double tag_spacing = 0.3;
  const int tag_id_offset = 0;
  const AprilGridConfig target_config{
      target_id,
      tag_rows,
      tag_cols,
      tag_size,
      tag_spacing,
      tag_id_offset,
  };
  target_configs.emplace(target_id, target_config);

  // Add target pose
  const auto target_center = target_config.get_center();
  const Vec3 target_pos{1.0, target_center.x(), 0.0};
  const Vec3 target_euler{M_PI / 2.0, 0.0, -M_PI / 2.0};
  const Mat3 target_rot = euler321(target_euler);
  const Mat4 T_WTj = tf(target_rot, target_pos);
  target_poses.emplace(0, T_WTj);

  // Add target geoemtry
  const Vec7 extrinsic{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  const auto pts = target_config.get_object_points();
  targets.emplace(target_id, CalibTargetGeometry(target_id, extrinsic, pts));
}

void SimCalib::setup_camera_geometries() {
  const std::string cam_model = "BrownConrady4";
  const Vec2i cam_res{640, 480};
  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const double k1 = 0.1;
  const double k2 = 0.01;
  const double p1 = 0.001;
  const double p2 = 0.001;

  // cam0
  // -- Intrinsic
  VecX cam0_int;
  cam0_int.resize(8);
  cam0_int << fx, fy, cx, cy, k1, k2, p1, p2;
  // -- Extrinsic
  const Vec3 cam0_ext_pos{0.0, 0.0, 0.0};
  const Vec3 cam0_ext_euler{0.0, 0.0, 0.0};
  const Mat3 cam0_ext_rot = euler321(cam0_ext_euler);
  const Vec7 cam0_ext = tf_vec(tf(cam0_ext_rot, cam0_ext_pos));
  // -- Camera geometry
  cameras.emplace(0, CameraGeometry(0, cam_model, cam_res, cam0_int, cam0_ext));

  // cam1
  // -- Intrinsic
  VecX cam1_int;
  cam1_int.resize(8);
  cam1_int << fx, fy, cx, cy, k1, k2, p1, p2;
  // -- Extrinsic
  const Vec3 cam1_ext_pos{0.05, 0.0, 0.0};
  const Vec3 cam1_ext_euler{0.0, 0.0, 0.0};
  const Mat3 cam1_ext_rot = euler321(cam1_ext_euler);
  const Vec7 cam1_ext = tf_vec(tf(cam1_ext_rot, cam1_ext_pos));
  // -- Camera geometry
  cameras.emplace(1, CameraGeometry(1, cam_model, cam_res, cam1_int, cam1_ext));
}

void SimCalib::setup_imu_geometries() {
  ImuParams imu_params;
  imu_params.acc_noise_density = 0.08;
  imu_params.gyr_noise_density = 0.004;
  imu_params.bias_walk_acc = 0.00004;
  imu_params.bias_walk_gyr = 2.0e-6;

  Vec7 extrinsic{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  imus.emplace(0, ImuGeometry(0, imu_params, extrinsic));
}

void SimCalib::setup_camera_poses(const double camera_rate,
                                  const double sample_x,
                                  const double sample_y,
                                  const double sample_z,
                                  const double sample_z_offset,
                                  const int sample_num_x,
                                  const int sample_num_y,
                                  const int sample_num_z) {
  const auto half_x = sample_x / 2.0;
  const auto half_y = sample_y / 2.0;
  const auto half_z = sample_z / 2.0;
  const auto range_x = linspace(-half_x, half_x, sample_num_x);
  const auto range_y = linspace(-half_y, half_y, sample_num_y);
  const auto range_z = linspace(-half_z, half_z, sample_num_z);

  const auto target = target_configs.at(0);
  const Mat4 T_WT0 = target_poses.at(0);
  const Vec2 center = target.get_center();
  const Vec3 p_center = tf_point(T_WT0, Vec3{center.x(), center.y(), 0.0});

  // const time_t time_now = time(NULL);
  // timestamp_t ts = time_now * 1e9;
  timestamp_t ts = 0;
  timestamp_t ts_diff = (1.0 / camera_rate) * 1e9;

  for (const auto y : range_y) {
    for (const auto z : range_z) {
      for (const auto x : range_x) {
        // Transform camera position in target frame to world frame
        const Vec3 p_T{x + center.x(), y + center.y(), z + sample_z_offset};
        const Vec3 p_W = tf_point(T_WT0, p_T);

        // Form camera rotation
        // clang-format off
        const Vec3 world_up{0.0, 0.0, 1.0};
        const Vec3 dir = (p_center - p_W).normalized();
        const Vec3 right= dir.cross(world_up).normalized();
        const Vec3 up = dir.cross(right);
        Mat3 R;
        R << right.x(), up.x(), dir.x(),
             right.y(), up.y(), dir.y(),
             right.z(), up.z(), dir.z();
        // clang-format on

        // Form pose transform
        Mat4 T_WC0 = eye(4);
        T_WC0.block<3, 3>(0, 0) = R;
        T_WC0.block<3, 1>(0, 3) = p_W;

        camera_poses[ts] = T_WC0;
        ts += ts_diff;
      }
    }
  }
}

void SimCalib::setup_camera_poses(const double camera_rate,
                                  const std::string traj_type,
                                  const double R,
                                  const double T,
                                  const double td_s) {
  const auto target = target_configs.at(0);
  const Mat4 T_WT0 = target_poses.at(0);
  const Mat4 T_TO = target.get_center_relative_pose();
  const timestamp_t ts_start = 0;
  const double calib_width = target.get_width_height().x();
  const double calib_height = target.get_width_height().y();
  LissajousTrajectory
      traj{traj_type, ts_start, T_WT0, T_TO, calib_width, calib_height, R, T};

  // Simulate camera measurements
  {
    // Camera rate and time variables
    const double dt = 1.0 / camera_rate;
    timestamp_t ts_k = 0;
    timestamp_t ts_end = sec2ts(T);

    // Camera measurements at true timestamps (no delay)
    while (ts_k <= ts_end) {
      const auto T_WS = traj.get_pose(ts_k);
      camera_poses[ts_k] = T_WS;
      ts_k += sec2ts(dt);
    }
  }

  // Simulate imu measurements with time delay
  // The IMU at reported timestamp ts_k measured the physical state at
  // time ts_k - td_s (IMU clock runs ahead of camera clock by
  // td_s).  This reverses the earlier approach where the time
  // delay was introduced in the camera.
  {
    const double imu_rate = 1000.0;
    const double dt = 1.0 / imu_rate;
    const timestamp_t td_ns = sec2ts(td_s);
    timestamp_t ts_k = td_ns; // first IMU event corresponds to phys t=0
    timestamp_t ts_end_imu = sec2ts(T) + td_ns;

    while (ts_k <= ts_end_imu) {
      const timestamp_t ts_td_k = ts_k - td_ns;
      const Mat4 T_WS = traj.get_pose(ts_td_k);
      const Mat3 C_WS = tf_rot(T_WS);
      const Vec3 v_WS = traj.get_velocity(ts_td_k);
      const Vec3 a_WS_W = traj.get_acceleration(ts_td_k);
      const Vec3 w_WS_W = traj.get_angular_velocity(ts_td_k);

      // Transform acceleration and angular velocity to body frame
      const Vec3 g{0.0, 0.0, 9.81};
      const Vec3 a_WS_S = C_WS.transpose() * (a_WS_W + g);
      const Vec3 w_WS_S = C_WS.transpose() * w_WS_W;

      // Track imu measurements
      imu_vel[ts_k] = v_WS;
      imu_acc[ts_k] = a_WS_S;
      imu_gyr[ts_k] = w_WS_S;

      // Update
      ts_k += sec2ts(dt);
    }
  }
}

/******************************************************************************
 * SIMULATE
 *****************************************************************************/

void SimCalib::simulate_camera_views() {
  // Simulate single camera view
  auto sim_view = [&](const timestamp_t ts,
                      const int camera_id,
                      const int target_id,
                      const Mat4 T_WC0) {
    const auto res = cameras.at(camera_id).resolution;
    const auto intrinsic = cameras.at(camera_id).intrinsic;
    const auto T_C0Ci = tf(cameras.at(camera_id).extrinsic);
    const auto camera = cameras.at(camera_id).camera_model;
    const auto T_WTj = target_poses[target_id];
    const auto T_CiTj = T_C0Ci.inverse() * T_WC0.inverse() * T_WTj;

    const AprilGridConfig &config = target_configs[target_id];
    auto target = std::make_shared<AprilGrid>(ts, camera_id, config);

    for (int tag_id = 0; tag_id < config.get_num_tags(); ++tag_id) {
      for (int corner_index = 0; corner_index < 4; ++corner_index) {
        const Vec3 p_Tj = config.get_object_point(tag_id, corner_index);
        const Vec3 p_Ci = tf_point(T_CiTj, p_Tj);
        Vec2 z{0.0, 0.0};
        if (camera->project(res, intrinsic, p_Ci, z) != 0) {
          continue;
        }
        target->add(tag_id, corner_index, z);
      }
    }

    return target;
  };

  // Simulate camera views
  for (const auto &[ts, T_WC0] : camera_poses) {
    for (const auto &[camera_id, _] : cameras) {
      for (const auto &[target_id, _] : target_configs) {
        const auto target = sim_view(ts, camera_id, target_id, T_WC0);
        camera_views[ts][camera_id][target_id] = target;
      }
    }
  }
}

/******************************************************************************
 * TIMELINE
 *****************************************************************************/

Timeline SimCalib::get_timeline() const {
  Timeline timeline;

  for (const auto &[ts, camera_map] : camera_views) {
    for (const auto &[camera_id, target_map] : camera_map) {
      for (const auto &[target_id, target] : target_map) {
        timeline.add(ts, camera_id, target);
      }
    }
  }

  for (const auto &[ts, _] : imu_acc) {
    timeline.add(ts, imu_acc.at(ts), imu_gyr.at(ts));
  }

  return timeline;
}

/******************************************************************************
 * SAVE
 *****************************************************************************/

int SimCalib::save_target_configs(const fs::path &yaml_path) const {
  const auto fp = fopen(yaml_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", yaml_path.c_str());
    return -1;
  }

  for (const auto &[target_id, config] : target_configs) {
    fprintf(fp, "target%d:\n", target_id);
    fprintf(fp, "  tag_rows: %d\n", config.tag_rows);
    fprintf(fp, "  tag_cols: %d\n", config.tag_cols);
    fprintf(fp, "  tag_size: %f\n", config.tag_size);
    fprintf(fp, "  tag_spacing: %f\n", config.tag_spacing);
    fprintf(fp, "  tag_id_offset: %d\n", config.tag_id_offset);
    fprintf(fp, "\n");
  }
  fclose(fp);

  return 0;
}

int SimCalib::save_target_poses(const fs::path &csv_path) const {
  const auto fp = fopen(csv_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", csv_path.c_str());
    return -1;
  }

  fprintf(fp, "#target_id, x, y, z, qx, qy, qz, qw\n");
  for (const auto &[target_id, T_WTj] : target_poses) {
    const auto pos = tf_trans(T_WTj);
    const auto quat = tf_quat(T_WTj);
    fprintf(fp, "%d, ", target_id);
    fprintf(fp, "%f, %f, %f, ", pos.x(), pos.y(), pos.z());
    fprintf(fp, "%f, %f, %f, %f\n", quat.x(), quat.y(), quat.z(), quat.w());
  }
  fclose(fp);

  return 0;
}

int SimCalib::save_camera_geometries(const fs::path &yaml_path) const {
  const auto fp = fopen(yaml_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", yaml_path.c_str());
    return -1;
  }

  for (const auto &[camera_id, camera] : cameras) {
    const auto res = camera.resolution;
    const auto camera_model = camera.camera_model->type();
    fprintf(fp, "camera%d:\n", camera_id);
    fprintf(fp, "  camera_model: \"%s\"\n", camera_model.c_str());
    fprintf(fp, "  resolution:   [%d, %d]\n", res.x(), res.y());
    fprintf(fp, "  intrinsic:    %s\n", vec2str(camera.intrinsic).c_str());
    fprintf(fp, "  extrinsic:    %s\n", vec2str(camera.extrinsic).c_str());
    fprintf(fp, "\n");
  }
  fclose(fp);

  return 0;
}

int SimCalib::save_imu_geometries(const fs::path &yaml_path) const {
  const auto fp = fopen(yaml_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", yaml_path.c_str());
    return -1;
  }

  for (const auto &[imu_id, imu] : imus) {
    fprintf(fp, "imu%d:\n", imu_id);
    fprintf(fp, "  acc_noise_density: %e\n", imu.imu_params.acc_noise_density);
    fprintf(fp, "  gyr_noise_density: %e\n", imu.imu_params.gyr_noise_density);
    fprintf(fp, "  bias_walk_acc:  %e\n", imu.imu_params.bias_walk_acc);
    fprintf(fp, "  bias_walk_gyr:  %e\n", imu.imu_params.bias_walk_gyr);
    fprintf(fp, "  extrinsic: %s\n", vec2str(imu.extrinsic).c_str());
    fprintf(fp, "\n");
  }
  fclose(fp);

  return 0;
}

int SimCalib::save_camera_poses(const fs::path &csv_path) const {
  const auto fp = fopen(csv_path.c_str(), "w");
  if (fp == NULL) {
    LOG_ERROR("Failed to open [%s] for saving!", csv_path.c_str());
    return -1;
  }

  fprintf(fp, "#ts, x, y, z, qx, qy, qz, qw\n");
  for (const auto &[ts, T_WC0] : camera_poses) {
    const auto pos = tf_trans(T_WC0);
    const auto quat = tf_quat(T_WC0);
    fprintf(fp, "%ld, ", ts);
    fprintf(fp, "%f, %f, %f, ", pos.x(), pos.y(), pos.z());
    fprintf(fp, "%f, %f, %f, %f\n", quat.x(), quat.y(), quat.z(), quat.w());
  }
  fclose(fp);

  return 0;
}

int SimCalib::save_camera_views(const fs::path &save_dir) const {
  // Create target directories
  for (const auto &[target_id, _] : target_configs) {
    const fs::path &target_str = "target" + std::to_string(target_id);
    const fs::path &target_dir = save_dir / target_str;
    if (!fs::exists(target_dir) && !fs::create_directories(target_dir)) {
      LOG_ERROR("Could not create dir [%s]!", target_dir.c_str());
      return -1;
    }
  }

  // Save camera views
  for (const auto &[ts, camera_map] : camera_views) {
    for (const auto &[camera_id, target_map] : camera_map) {
      for (const auto &[target_id, target] : target_map) {
        const fs::path &target_str = "target" + std::to_string(target_id);
        const fs::path &camera_str = "cam" + std::to_string(camera_id);
        const fs::path &fname = std::to_string(ts) + ".csv";
        const fs::path &save_path = save_dir / target_str / camera_str / fname;
        target->save(save_path);
      }
    }
  }

  return 0;
}

int SimCalib::save(const fs::path &save_dir) const {
  // Check save dir
  if (!fs::exists(save_dir) && !fs::create_directories(save_dir)) {
    LOG_ERROR("Could not create dir [%s]!", save_dir.c_str());
    return -1;
  }

  // Save
  const auto target_configs_path = save_dir / "target_configs.yaml";
  const auto target_poses_path = save_dir / "target_poses.csv";
  const auto camera_geometries_path = save_dir / "camera_geometries.yaml";
  const auto camera_poses_path = save_dir / "camera_poses.csv";

  save_target_configs(target_configs_path);
  save_target_poses(target_poses_path);
  save_camera_geometries(camera_geometries_path);
  save_camera_poses(camera_poses_path);
  save_camera_views(save_dir);

  return 0;
}

} // namespace cartesian
