#include <optional>

#include "CalibCameraImu.hpp"
#include "SolvePnp.hpp"

namespace cartesian {

CalibCameraImu::CalibCameraImu(const std::string &config_file)
    : CalibProblem{config_file} {}

std::pair<int, int> CalibCameraImu::find_optimal_target() {
  int best_count = 0;
  int optimal_camera_id = 0;
  int optimal_target_id = 0;

  for (const auto &[camera_id, targets] : camera_buffers) {
    // Find the most observed calibration target by camera
    int camera_target_count = 0;
    int camera_target_id = -1;
    for (const auto &[target_id, target] : targets) {
      if (target->get_num_detected() > camera_target_count) {
        camera_target_count = target->get_num_detected();
        camera_target_id = target_id;
      }
    }

    // Track the most observed calibration target by all cameras
    if (camera_target_count > best_count) {
      best_count = camera_target_count;
      optimal_camera_id = camera_id;
      optimal_target_id = camera_target_id;
    }
  }

  return std::pair<int, int>{optimal_camera_id, optimal_target_id};
}

int CalibCameraImu::estimate_sensor_pose(Mat4 &T_WS) {
  // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  const auto [camera_id, target_id] = find_optimal_target();
  const auto target = camera_buffers.at(camera_id).at(target_id);
  target->get_measurements(point_ids, keypoints, object_points);
  if (keypoints.size() <= 8) {
    return -1;
  }

  // Estimate relative pose T_CiTj
  const auto camera_geometry = camera_geometries[camera_id];
  const auto target_geometry = target_geometries[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return -2;
  }

  // Form T_WS
  const int imu_id = 0;
  const Mat4 T_WT0 = tf(target_pose);
  const Mat4 T_T0Tj = tf(target_geometry->extrinsic);
  const Mat4 T_TjCi = T_CiTj.inverse();
  const Mat4 T_CiC0 = tf(camera_geometry->extrinsic).inverse();
  const Mat4 T_C0S = tf(imu_geometries[imu_id]->extrinsic);
  T_WS = T_WT0 * T_T0Tj * T_TjCi * T_CiC0 * T_C0S;

  return 0;
}

int CalibCameraImu::estimate_camera_pose(Mat4 &T_C0T0) {
  // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  const auto [camera_id, target_id] = find_optimal_target();
  const auto target = camera_buffers.at(camera_id).at(target_id);
  target->get_measurements(point_ids, keypoints, object_points);
  if (keypoints.size() < 10) {
    return -1;
  }

  // Estimate relative pose T_CiTj
  const auto camera_geometry = camera_geometries[camera_id];
  const auto target_geometry = target_geometries[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return -2;
  }

  // Form relative pose T_C0T0
  const Mat4 T_TjT0 = tf(target_geometry->extrinsic).inverse();
  const Mat4 T_C0Ci = tf(camera_geometry->extrinsic);
  T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

  return 0;
}

void CalibCameraImu::add_view(const timestamp_t ts, const Mat4 &T_WS) {
  // Only process the first 100 frames
  if (imu_resblocks.size() >= 100) {
    return;
  }

  // Lambda function - Get last two keys in std::map
  auto last_two_keys = [](const auto &m)
      -> std::pair<typename std::decay_t<decltype(m)>::key_type,
                   typename std::decay_t<decltype(m)>::key_type> {
    assert(m.size() >= 2);

    auto it = m.rbegin();
    auto last = it->first;
    ++it;
    auto second_last = it->first;

    return std::make_pair(second_last, last);
  };

  // Lambda function - check target overlap between current and previous frame
  auto target_overlap = [](const int camera_id,
                           const CalibTargetPtr &target,
                           const CameraBuffers &prev_camera_buffers) {
    if (target->detected() == false) {
      return false;
    }
    if (prev_camera_buffers.count(camera_id) == 0) {
      return false;
    }
    if (prev_camera_buffers.at(camera_id).count(target->target_id) == 0) {
      return false;
    }
    return true;
  };

  // Lambda function - add vision factors
  auto add_vision_factors = [&](const timestamp_t ts_km1,
                                const timestamp_t ts_k,
                                const TimeDelayMethod td_method,
                                const int cam_id,
                                const int imu_id,
                                const CalibTargetPtr &target_k) {
    // Pre-check
    if (target_overlap(cam_id, target_k, prev_camera_buffers) == false) {
      return;
    }

    // Get CalibCameraImuError mode
    CalibCameraImuError::Mode mode = CalibCameraImuError::NOT_SET;
    if (td_method == TimeDelayMethod::PIXEL_VELOCITY) {
      mode = CalibCameraImuError::PIXEL_VELOCITY;
    } else if (td_method == TimeDelayMethod::POSE_INTERP) {
      mode = CalibCameraImuError::POSE_INTERP;
    }

    // Setup
    const int target_id = target_k->target_id;
    const auto target_geometry = get_target_geometry(target_id);
    const auto &camera_geometry = get_camera_geometry(cam_id);
    const auto &imu_geometry = get_imu_geometry(imu_id);
    double *pose_km1 = get_pose_ptr(ts_km1);
    double *pose_k = get_pose_ptr(ts_k);
    double *target_pose_ptr = get_target_pose_ptr();
    double *td_ptr = get_time_delay_ptr();

    // Get previous target measurements
    std::vector<int> prev_point_ids;
    Vec2s prev_keypoints;
    Vec3s prev_object_points;
    const auto target_km1 = prev_camera_buffers.at(cam_id).at(target_id);
    target_km1->get_measurements(prev_point_ids,
                                 prev_keypoints,
                                 prev_object_points);

    // Get current target measurements
    std::vector<int> curr_point_ids;
    Vec2s curr_keypoints;
    Vec3s curr_object_points;
    target_k->get_measurements(curr_point_ids,
                               curr_keypoints,
                               curr_object_points);

    // Build point_id -> keypoint map for previous frame
    std::map<int, Vec2> prev_kp_map;
    for (size_t i = 0; i < prev_point_ids.size(); ++i) {
      prev_kp_map[prev_point_ids[i]] = prev_keypoints[i];
    }

    // Form and add residual blocks for common point_ids
    for (size_t i = 0; i < curr_point_ids.size(); ++i) {
      const int pid = curr_point_ids[i];
      if (prev_kp_map.count(pid) == 0) {
        continue;
      }

      const auto z_km1 = prev_kp_map[pid];
      const auto z_k = curr_keypoints[i];
      auto resblock = CalibCameraImuError::create(mode,
                                                  ts_km1,
                                                  ts_k,
                                                  camera_geometry,
                                                  imu_geometry,
                                                  target_geometry,
                                                  pose_km1,
                                                  pose_k,
                                                  target_pose_ptr,
                                                  td_ptr,
                                                  pid,
                                                  z_km1,
                                                  z_k);

      camera_resblocks[ts][cam_id].push_back(resblock);
      add_residual_block(resblock.get());
    }
  };

  // Add pose
  add_pose(ts, T_WS);

  // Add speed and biases
  if (poses.size() == 1) {
    const Vec3 v_WS_k{0.0, 0.0, 0.0};
    add_speed_and_biases(ts, v_WS_k);

  } else {
    // Infer velocity from two poses T_WS_k and T_WS_km1
    const auto [ts_km1, ts_k] = last_two_keys(poses);
    const double dt = ts2sec(ts_k - ts_km1);
    const double *pose_km1 = get_pose_ptr(ts_km1);
    const double *pose_k = get_pose_ptr(ts_k);
    const Mat4 T_WS_km1 = tf(pose_km1);
    const Mat4 T_WS_k = tf(pose_k);
    const Vec3 r_WS_km1 = tf_trans(T_WS_km1);
    const Vec3 r_WS_k = tf_trans(T_WS_k);
    const Vec3 v_WS_k = (r_WS_k - r_WS_km1) / dt;
    add_speed_and_biases(ts_k, v_WS_k);
  }

  // Add camera measurement if it does not exist
  for (const auto &[camera_id, targets] : camera_buffers) {
    for (const auto &[target_id, target] : targets) {
      if (has_camera_measurement(ts, camera_id, target_id) == false) {
        add_camera_measurement(ts, camera_id, target);
      }
    }
  }

  // Make sure we have 2 poses before adding vision and imu factors
  if (poses.size() < 2) {
    prev_camera_buffers = camera_buffers;
    camera_buffers.clear();
    return;
  }

  // Setup
  const auto [ts_km1, ts_k] = last_two_keys(poses);

  // Add time delay parameter
  if (time_delay_added_ == false) {
    add_time_delay();
  }

  // Add vision factors
  for (const auto &[cam_id, targets] : camera_buffers) {
    for (const auto &[target_id, target] : targets) {
      add_vision_factors(ts_km1, ts_k, td_method, cam_id, 0, target);
    }
  }

  // Add imu factor
  {
    const int imu_id = 0;
    const auto &imu_params = get_imu_geometry(imu_id)->imu_params;
    double *pose_km1 = get_pose_ptr(ts_km1);
    double *pose_k = get_pose_ptr(ts_k);
    double *sb_km1 = get_speed_and_biases_ptr(ts_km1);
    double *sb_k = get_speed_and_biases_ptr(ts_k);
    auto imu_data = imu_buffer.extract(ts_km1, ts_k);
    imu_buffer.trim(ts_k);

    auto resblock =
        ImuError::create(imu_params, imu_data, pose_km1, sb_km1, pose_k, sb_k);
    imu_resblocks[ts_km1][imu_id] = resblock;
    add_residual_block(resblock.get());
  }

  // Clean up
  prev_camera_buffers = camera_buffers;
  camera_buffers.clear();
}

void CalibCameraImu::initialize(const timestamp_t ts) {
  assert(initialized == false);

  // Estimate relative pose - T_C0T0
  Mat4 T_C0T0;
  if (estimate_camera_pose(T_C0T0) != 0) {
    return;
  }

  // Estimate initial IMU attitude
  const Mat3 C_WS = imu_buffer.estimateAttitude();

  // Sensor pose - T_WS
  Mat4 T_WS = tf(C_WS, zeros(3, 1));

  // Target pose - T_WT0
  Mat4 T_SC0 = tf(imu_geometries[0]->extrinsic).inverse();
  Mat4 T_WT0 = T_WS * T_SC0 * T_C0T0;

  // Set:
  // 1. Target as origin (with r_WT0 (0, 0, 0))
  // 2. Calculate the sensor pose offset relative to target origin
  const Vec3 offset = -1.0 * tf_trans(T_WT0);
  const Mat3 C_WT0 = tf_rot(T_WT0);
  const Vec3 r_WT0{0.0, 0.0, 0.0};
  T_WT0 = tf(C_WT0, r_WT0);
  T_WS = tf(C_WS, offset);

  // Add target pose
  set_target_pose(T_WT0);

  // Add camera residuals
  add_view(ts, T_WS);

  // Update
  initialized = true;
}

void CalibCameraImu::add_measurement(const timestamp_t ts,
                                     const Vec3 &imu_acc,
                                     const Vec3 &imu_gyr) {
  // Add Imu measurement
  imu_buffer.add(ts, imu_acc, imu_gyr);
  imu_started = true;

  // Check if calibration target buffer is filled
  if (static_cast<int>(camera_buffers.size()) != get_num_cameras()) {
    return;
  }

  // Add view
  if (initialized == false) {
    initialize(ts);

  } else {
    Mat4 T_WS;
    if (estimate_sensor_pose(T_WS) != 0) {
      return;
    }
    add_view(ts, T_WS);
  }
}

void CalibCameraImu::add_measurement(const timestamp_t ts,
                                     const int camera_id,
                                     const CalibTargetPtr &calib_target) {
  // Pre-check
  if (ts != calib_target->ts) {
    FATAL("ts != calib_target->ts");
  }

  // Do not add vision data before first imu measurement
  if (imu_started == false) {
    return;
  }

  // Add to camera buffer
  const auto target_id = calib_target->target_id;
  camera_buffers[camera_id][target_id] = calib_target;
  camera_started = true;

  return;
}

void CalibCameraImu::solve() {
  assert(camera_data.size() != 0);
  assert(camera_geometries.size() != 0);
  assert(imu_geometries.size() != 0);
  assert(target_geometries.size() != 0);

  for (auto &[camera_id, camera] : camera_geometries) {
    problem->SetParameterBlockConstant(camera->intrinsic.data());
    problem->SetParameterBlockConstant(camera->extrinsic.data());
  }
  for (auto &[target_id, target] : target_geometries) {
    problem->SetParameterBlockConstant(target->extrinsic.data());
  }

  if (verbose) {
    printf("Initial estimates\n");
    printf("-----------------\n");

    print_target_geometries(stdout);
    print_camera_geometries(stdout);
    print_imu_geometries(stdout);
    print_time_delay(stdout);

    printf("-----------------\n");
    printf("\n");
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = max_iters;
  options.num_threads = 4;
  options.min_trust_region_radius = 1e-50; // Default: 1e-32
  options.function_tolerance = 1e-20;      // Default: 1e-6
  options.gradient_tolerance = 1e-20;      // Default: 1e-10
  options.parameter_tolerance = 1e-20;     // Default: 1e-8

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  num_iters = summary.iterations.size();
  final_cost = summary.final_cost;
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl << std::endl;
    // std::cout << summary.FullReport() << std::endl << std::endl;
    print_summary(stdout);
  }
}

} // namespace cartesian
