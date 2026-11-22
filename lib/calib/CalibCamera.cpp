#include <algorithm>

#include "CalibCamera.hpp"

#include "../timeline/Timeline.hpp"

namespace cartesian {

CalibCamera::CalibCamera(const std::string &config_file)
    : CalibProblem{config_file} {}

void CalibCamera::add_view(const std::map<int, CalibTargetMap> &measurements) {
  // Lambda function - Find the target with the most observations
  auto find_optimal_target =
      [&](const std::map<int, CalibTargetMap> &measurements) {
        int best_count = 0;
        int optimal_camera_id = 0;
        int optimal_target_id = 0;

        for (const auto &[camera_id, targets] : measurements) {
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
      };

  // Get calibration target measurements
  std::vector<int> point_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  const auto [camera_id, target_id] = find_optimal_target(measurements);
  const auto target = measurements.at(camera_id).at(target_id);
  target->get_measurements(point_ids, keypoints, object_points);
  if (keypoints.size() < 10) {
    return;
  }

  // Estimate relative pose T_CiTj
  const auto camera_geometry = camera_geometries[camera_id];
  const auto target_geometry = target_geometries[target_id];
  Mat4 T_CiTj;
  SolvePnp pnp{camera_geometry};
  int status = pnp.estimate(keypoints, object_points, T_CiTj);
  if (status != 0) {
    return;
  }

  // Form T_C0T0
  const Mat4 T_C0Ci = tf(camera_geometry->extrinsic);
  const Mat4 T_TjT0 = tf(target_geometry->extrinsic).inverse();
  Mat4 T_C0T0 = T_C0Ci * T_CiTj * T_TjT0;

  // Add camera measurement if it does not exist
  const timestamp_t ts = target->ts;
  for (const auto &[camera_id, targets] : measurements) {
    for (const auto &[target_id, target] : targets) {
      if (has_camera_measurement(ts, camera_id, target_id) == false) {
        add_camera_measurement(ts, camera_id, target);
      }
    }
  }

  // Add timestamp and pose
  add_pose(ts, T_C0T0);
  view_timestamps.push_back(ts);

  // Add residual blocks
  for (const auto &[camera_id, targets] : measurements) {
    const auto camera_geometry = get_camera_geometry(camera_id);

    for (const auto &[target_id, target] : targets) {
      // Check if detected
      const auto target_geometry = get_target_geometry(target_id);
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

      // Form and add residual blocks
      for (size_t i = 0; i < point_ids.size(); i++) {
        auto resblock = CalibCameraError::create(camera_geometry,
                                                 target_geometry,
                                                 point_ids[i],
                                                 poses[ts].data(),
                                                 keypoints[i]);
        resblocks[ts][camera_id].push_back(resblock);
        resblock_ids[ts].push_back(add_residual_block(resblock.get()));
      }
    }
  }
}

void CalibCamera::remove_view(const timestamp_t ts) {
  // The residual blocks first, they use the pose
  for (const auto resblock_id : resblock_ids[ts]) {
    problem->RemoveResidualBlock(resblock_id);
  }
  resblock_ids.erase(ts);
  resblocks.erase(ts);

  // Then the pose
  problem->RemoveParameterBlock(poses.at(ts).data());
  poses.erase(ts);
  timestamps.erase(ts);
  view_timestamps.erase(
      std::remove(view_timestamps.begin(), view_timestamps.end(), ts),
      view_timestamps.end());
}

ceres::Solver::Summary CalibCamera::optimize(const int max_iters) {
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.max_num_iterations = max_iters;
  options.num_threads = 1;
  options.initial_trust_region_radius = 10; // Default: 1e4
  options.min_trust_region_radius = 1e-50;  // Default: 1e-32
  options.function_tolerance = 1e-20;       // Default: 1e-6
  options.gradient_tolerance = 1e-20;       // Default: 1e-10
  options.parameter_tolerance = 1e-20;      // Default: 1e-8

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  return summary;
}

void CalibCamera::solve() {
  // Pre-check
  if (camera_data.size() == 0) {
    FATAL("No camera data?");
  }
  if (camera_geometries.size() == 0) {
    FATAL("No cameras added?");
  }
  if (target_geometries.size() == 0) {
    FATAL("No targets added?");
  }

  // Initialize intrinsics and extrinsics
  initialize_camera_intrinsics();
  initialize_camera_extrinsics();

  // Form timeline
  Timeline timeline;
  for (const auto &[camera_id, measurements] : camera_data) {
    for (const auto &[ts, targets] : measurements) {
      for (const auto &[target_id, target] : targets) {
        timeline.add(ts, camera_id, target);
      }
    }
  }

  // Loop through timeline
  for (const auto ts : timeline.timestamps) {
    std::map<int, CalibTargetMap> viewset;
    for (const auto &event : timeline.getEvents(ts)) {
      if (auto target_event = dynamic_cast<CalibTargetEvent *>(event)) {
        const auto camera_id = target_event->camera_id;
        const auto target_id = target_event->target_id;
        const auto calib_target = target_event->calib_target;
        viewset[camera_id][target_id] = calib_target;
      }
    }
    add_view(viewset);

    // Sliding window, drop the oldest view when the window is full, and
    // optimize with the views in the window. Wait until it is full, a few
    // views do not constrain the camera
    if (window_size <= 0) {
      continue;
    }
    while (static_cast<int>(view_timestamps.size()) > window_size) {
      remove_view(view_timestamps.front());
    }
    if (static_cast<int>(view_timestamps.size()) == window_size) {
      optimize(window_max_iters);
    }
  }

  // Solve
  const int max_iters = 30;
  const auto summary = optimize(max_iters);
  if (verbose) {
    std::cout << summary.FullReport() << std::endl << std::endl;
    print_summary(stdout);
  }
}

} // namespace cartesian
