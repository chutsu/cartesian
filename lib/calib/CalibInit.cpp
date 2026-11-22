#include "CalibInit.hpp"

namespace cartesian {

void CalibInit::initialize_camera_intrinsics(
    const std::map<int, CalibTargetData> camera_data,
    const std::map<int, AprilGridConfig> target_configs,
    std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
    const bool verbose) {
  std::map<int, std::map<int, Vec3>> target_points;
  for (const auto &[target_id, target_config] : target_configs) {
    target_points[target_id] = target_config.get_object_points();
  }
  initialize_camera_intrinsics(camera_data, target_points, cam_geoms, verbose);
}

void CalibInit::initialize_camera_intrinsics(
    const std::map<int, CalibTargetData> camera_data,
    const std::map<int, std::map<int, Vec3>> target_points,
    std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
    const bool verbose) {
  // Find the target with the most observations for a particular camera
  auto find_optimal_target = [&](const CalibTargetData &camera_data) {
    std::map<int, int> target_count; // target_id, count

    // Initialize map
    for (const auto &[target_id, _] : target_points) {
      target_count[target_id] = 0;
    }

    // Count
    for (const auto &[ts, targets] : camera_data) {
      for (const auto &[target_id, target] : targets) {
        target_count[target_id] += target->get_num_detected();
      }
    }

    // Find best count
    int best_target_count = 0;
    int best_target_id = -1;
    for (const auto &[target_id, count] : target_count) {
      if (count > best_target_count) {
        best_target_count = count;
        best_target_id = target_id;
      }
    }

    return best_target_id;
  };

  // Solve intrinsics of a single camera
  auto solve_intrinsics = [&](const int camera_id,
                              const CalibTargetData &camera_data) {
    // Setup Problem
    PoseManifold pose_plus;
    std::map<timestamp_t, Vec7> relposes;
    std::vector<std::shared_ptr<CalibCameraError>> resblocks;
    ceres::Problem::Options init_prob_options;
    init_prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    init_prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    init_prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    init_prob_options.enable_fast_removal = true;
    auto init_problem = std::make_unique<ceres::Problem>(init_prob_options);

    // -- Add camera to problem
    auto camera_geometry = cam_geoms.at(camera_id);
    const int intrinsic_size = camera_geometry->intrinsic.size();
    double *intrinsic = camera_geometry->intrinsic.data();
    double *extrinsic = camera_geometry->extrinsic.data();
    init_problem->AddParameterBlock(intrinsic, intrinsic_size);
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus);
    init_problem->SetParameterBlockConstant(extrinsic);

    // -- Add calibration target to problem
    const auto optimal_target_id = find_optimal_target(camera_data);
    const auto points = target_points.at(optimal_target_id);
    const auto target_ext = tf_vec(eye(4));
    auto target0 = std::make_shared<CalibTargetGeometry>(0, target_ext, points);
    init_problem->AddParameterBlock(target0->extrinsic.data(), 7);
    init_problem->SetManifold(target0->extrinsic.data(), &pose_plus);
    init_problem->SetParameterBlockConstant(target0->extrinsic.data());

    // -- Build problem
    for (const auto &[ts, targets] : camera_data) {
      // Check if detected
      const auto target = targets.at(optimal_target_id);
      if (target->detected() == false) {
        continue;
      }

      // Get calibration target measurements
      std::vector<int> point_ids;
      Vec2s keypoints;
      Vec3s object_points;
      target->get_measurements(point_ids, keypoints, object_points);
      if (keypoints.size() < 10) {
        continue;
      }

      // Estimate relative pose T_CT
      Mat4 T_CT;
      SolvePnp pnp{camera_geometry};
      int status = pnp.estimate(keypoints, object_points, T_CT);
      if (status != 0) {
        continue;
      }

      // Add pose
      Mat2 covar = eye(2);
      relposes[ts] = tf_vec(T_CT);
      init_problem->AddParameterBlock(relposes[ts].data(), 7);
      init_problem->SetManifold(relposes[ts].data(), &pose_plus);

      // Add residual blocks
      for (size_t i = 0; i < point_ids.size(); i++) {
        const int point_id = point_ids[i];
        Vec3 &pt = target0->points[point_id];
        init_problem->AddParameterBlock(pt.data(), 3);
        init_problem->SetParameterBlockConstant(pt.data());

        auto resblock = CalibCameraError::create(camera_geometry,
                                                 target0,
                                                 point_id,
                                                 relposes[ts].data(),
                                                 keypoints[i],
                                                 covar);
        resblocks.push_back(resblock);
        init_problem->AddResidualBlock(resblock.get(),
                                       nullptr,
                                       resblock->get_param_ptrs());
      }
    }

    // -- Solver options
    ceres::Solver::Options init_options;
    init_options.minimizer_progress_to_stdout = verbose;
    init_options.max_num_iterations = 30;
    init_options.num_threads = 1;
    init_options.initial_trust_region_radius = 10; // Default: 1e4
    init_options.min_trust_region_radius = 1e-50;  // Default: 1e-32
    init_options.function_tolerance = 1e-20;       // Default: 1e-6
    init_options.gradient_tolerance = 1e-20;       // Default: 1e-10
    init_options.parameter_tolerance = 1e-20;      // Default: 1e-8

    // -- Solve
    ceres::Solver::Summary summary;
    ceres::Solve(init_options, init_problem.get(), &summary);
    if (verbose) {
      std::cout << summary.FullReport() << std::endl << std::endl;
    }

    std::vector<double> reproj_errors;
    for (const auto &resblock : resblocks) {
      double error = 0.0;
      if (resblock->get_reproj_error(&error)) {
        reproj_errors.push_back(error);
      }
    }
    // printf("mean reproj error: %f\n", mean(reproj_errors));
    // printf("rmse reproj error: %f\n", rmse(reproj_errors));
    // printf("median reproj error: %f\n", median(reproj_errors));
  };

  // Solve all camera intrinsics
  for (const auto &[camera_id, data] : camera_data) {
    solve_intrinsics(camera_id, data);
  }
}

void CalibInit::initialize_camera_extrinsics(
    const std::map<int, CalibTargetData> cam_data,
    std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
    std::map<int, CalibTargetGeometryPtr> &target_geoms,
    const bool verbose) {
  // Initialize camera extrinsics
  CameraChain camchain(cam_geoms, cam_data);
  for (auto &[camera_id, camera_geometry] : cam_geoms) {
    Mat4 T_CiCj;
    if (camchain.find(0, camera_id, T_CiCj) != 0) {
      FATAL("No observations between camera0 and camera%d\n", camera_id);
    }
    camera_geometry->set_extrinsic(T_CiCj);
  }

  // Initialize calibration target extrinsics
  CalibTargetChain targetchain(cam_geoms, cam_data);
  for (auto &[target_id, t_geom] : target_geoms) {
    Mat4 T_T0Tj;
    if (camchain.find(0, target_id, T_T0Tj) != 0) {
      FATAL("No observations between target0 and target%d\n", target_id);
    }
    t_geom->set_extrinsic(T_T0Tj);
  }

  // Build problem
  PoseManifold pose_plus;
  Mat2 covar = eye(2);
  std::map<timestamp_t, Vec7> relposes;
  std::map<int, Vec3> target_points;
  std::vector<std::shared_ptr<CalibCameraError>> resblocks;
  ceres::Problem::Options init_problem_options;
  init_problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  init_problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  init_problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  init_problem_options.enable_fast_removal = true;
  auto init_problem = std::make_unique<ceres::Problem>(init_problem_options);

  // -- Add calibration target geometries
  for (auto &[target_id, t_geom] : target_geoms) {
    double *extrinsic = t_geom->extrinsic.data();
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus);
    if (target_id == 0) {
      init_problem->SetParameterBlockConstant(extrinsic);
    }
  }

  // -- Add camera geometries
  for (const auto &[camera_id, camera_data] : cam_data) {
    auto camera_geometry = cam_geoms.at(camera_id);
    const int intrinsic_size = camera_geometry->intrinsic.size();
    double *intrinsic = camera_geometry->intrinsic.data();
    double *extrinsic = camera_geometry->extrinsic.data();
    init_problem->AddParameterBlock(intrinsic, intrinsic_size);
    init_problem->AddParameterBlock(extrinsic, 7);
    init_problem->SetManifold(extrinsic, &pose_plus);
    if (camera_id == 0) {
      init_problem->SetParameterBlockConstant(extrinsic);
    }
  }

  // -- Add camera measurements
  for (const auto &[camera_id, camera_data] : cam_data) {
    const auto camera_geometry = cam_geoms.at(camera_id);

    for (const auto &[ts, targets] : camera_data) {
      for (const auto &[target_id, target] : targets) {
        // Check if detected
        const auto t_geom = target_geoms.at(target_id);
        if (target->detected() == false) {
          continue;
        }

        // Get calibration target measurements
        std::vector<int> point_ids;
        std::vector<int> corner_indicies;
        Vec2s keypoints;
        Vec3s object_points;
        target->get_measurements(point_ids, keypoints, object_points);
        if (keypoints.size() < 10) {
          continue;
        }

        // Estimate relative pose T_C0T0
        Mat4 T_C0T0;
        if (relposes.count(ts) == 0) {
          // Solvepnp T_CiTj
          Mat4 T_CiTj;
          SolvePnp pnp{camera_geometry};
          int status = pnp.estimate(keypoints, object_points, T_CiTj);
          if (status != 0) {
            continue;
          }

          // Form T_C0T0
          const Mat4 T_C0Ci = tf(camera_geometry->extrinsic);
          const Mat4 T_TjT0 = tf(t_geom->extrinsic).inverse();
          T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

          // Add pose
          relposes[ts] = tf_vec(T_C0T0);
          init_problem->AddParameterBlock(relposes[ts].data(), 7);
          init_problem->SetManifold(relposes[ts].data(), &pose_plus);
        } else {
          T_C0T0 = tf(relposes[ts]);
        }

        // Add residual blocks
        for (size_t i = 0; i < point_ids.size(); i++) {
          const int point_id = point_ids[i];
          Vec3 &pt = t_geom->points[point_id];
          init_problem->AddParameterBlock(pt.data(), 3);
          init_problem->SetParameterBlockConstant(pt.data());

          auto resblock = CalibCameraError::create(camera_geometry,
                                                   t_geom,
                                                   point_id,
                                                   relposes[ts].data(),
                                                   keypoints[i],
                                                   covar);
          resblocks.push_back(resblock);
          init_problem->AddResidualBlock(resblock.get(),
                                         nullptr,
                                         resblock->get_param_ptrs());
        }
      }
    }
  }

  // Solver options
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = 30;
  options.num_threads = 1;
  options.initial_trust_region_radius = 1e-4; // Default: 1e4
  options.min_trust_region_radius = 1e-50;    // Default: 1e-32
  options.function_tolerance = 1e-20;         // Default: 1e-6
  options.gradient_tolerance = 1e-20;         // Default: 1e-10
  options.parameter_tolerance = 1e-20;        // Default: 1e-8

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, init_problem.get(), &summary);
  if (verbose) {
    std::cout << summary.FullReport() << std::endl << std::endl;
  }

  std::vector<double> reproj_errors;
  for (const auto &resblock : resblocks) {
    double error = 0.0;
    if (resblock->get_reproj_error(&error)) {
      reproj_errors.push_back(error);
    }
  }
  // printf("mean reproj error: %f\n", mean(reproj_errors));
  // printf("rmse reproj error: %f\n", rmse(reproj_errors));
  // printf("median reproj error: %f\n", median(reproj_errors));
}

Mat4 CalibInit::initialize_camera_imu_extrinsic(
    const Timeline &timeline,
    const std::map<int, std::shared_ptr<CameraGeometry>> &cam_geoms,
    const ImuParams &imu_params) {
  // Get cam0 calibration targets and imu data
  ImuBuffer imu_buffer;
  std::map<timestamp_t, std::shared_ptr<CalibTarget>> cam0_targets;
  for (const auto &ts : timeline.timestamps) {
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto &cam_id = target_event->camera_id;
        if (cam_id == 0) {
          cam0_targets[ts] = target_event->calib_target;
        }
      }

      if (auto imu_event = dynamic_cast<ImuEvent *>(event)) {
        imu_buffer.add(ts, imu_event->acc, imu_event->gyr);
      }
    }
  }

  // Get camera poses
  const auto cam0_geom = cam_geoms.at(0);
  std::map<timestamp_t, Mat4> cam0_poses;
  for (const auto &[ts, target] : cam0_targets) {
    // Get calibration target measurements
    std::vector<int> point_ids;
    std::vector<int> corner_indicies;
    Vec2s keypoints;
    Vec3s object_points;
    target->get_measurements(point_ids, keypoints, object_points);
    if (point_ids.size() < 10) {
      continue;
    }

    // Estimate relative pose T_CiTj
    Mat4 T_CiTj;
    SolvePnp pnp{cam0_geom};
    int status = pnp.estimate(keypoints, object_points, T_CiTj);
    if (status != 0) {
      continue;
    }

    // Add camera pose
    cam0_poses[ts] = T_CiTj.inverse();
  }

  // Form relative camera poses T_Ckm1_Ck
  std::vector<timestamp_t> cam0_timestamps;
  std::map<timestamp_t, Mat4> cam0_relposes;
  Mat4 T_TjCi_km1 = cam0_poses.begin()->second;
  for (const auto &[ts, T_TjCi_k] : cam0_poses) {
    cam0_timestamps.push_back(ts);
    cam0_relposes[ts] = T_TjCi_km1.inverse() * T_TjCi_k;
    T_TjCi_km1 = T_TjCi_k;
  }

  // Form relative imu poses T_Skm1_Sk
  std::vector<timestamp_t> imu0_timestamps;
  std::map<timestamp_t, Mat4> imu0_relposes;
  for (size_t k = 1; k < cam0_timestamps.size(); ++k) {
    const timestamp_t ts_km1 = cam0_timestamps[k - 1];
    const timestamp_t ts_k = cam0_timestamps[k];
    if (ts_km1 < imu_buffer.timestamps.front()) {
      continue;
    }

    const ImuBuffer &imu_data = imu_buffer.extract(ts_km1, ts_k);
    const ImuPreintegrate imu_pint{imu_params, imu_data};
    imu0_timestamps.push_back(ts_k);
    imu0_relposes[ts_k] = tf(imu_pint.dq, imu_pint.dr);
  }

  Mat3 R = SO3::solve_handeye(cam0_relposes, imu0_relposes);
  Vec3 t{0.0, 0.0, 0.0};

  return tf(R, t);
}

} // namespace cartesian
