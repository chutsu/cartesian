#pragma once

#include <deque>

#include "SolvePnp.hpp"
#include "CalibProblem.hpp"
#include "../ceres/PoseManifold.hpp"
#include "../ceres/CalibCameraError.hpp"

namespace cartesian {

/**
 * Camera Calibrator
 *
 * `solve()` calibrates with all the views at once. With `window_size` set, it
 * adds the views one by one instead, only keeps the most recent `window_size`
 * views in the problem, and optimizes after each new view (a sliding window).
 * The views that leave the window are dropped, not marginalized, so the
 * calibration is estimated from the last `window_size` views.
 */
struct CalibCamera : CalibProblem {
  using CameraResiduals = std::map<int, std::vector<CalibCameraErrorPtr>>;
  std::map<timestamp_t, CameraResiduals> resblocks;

  // Sliding window
  int window_size = 0;       // Number of views kept in the problem, 0 for all
  int window_max_iters = 10; // Max iterations of the optimization / view
  std::deque<timestamp_t> view_timestamps; // Views in the problem, oldest first
  std::map<timestamp_t, std::vector<ceres::ResidualBlockId>> resblock_ids;

  CalibCamera() = default;
  CalibCamera(const std::string &config_file);
  virtual ~CalibCamera() = default;

  /** Add camera calibration view */
  void add_view(const std::map<int, CalibTargetMap> &measurements);

  /** Remove the view at `ts`, its pose and residual blocks, from the problem */
  void remove_view(const timestamp_t ts);

  /** Optimize the problem for at most `max_iters` iterations */
  ceres::Solver::Summary optimize(const int max_iters);

  /** Solve */
  void solve();
};

} // namespace cartesian
