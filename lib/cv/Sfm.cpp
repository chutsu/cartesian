#include <algorithm>
#include <set>

#include "Sfm.hpp"
#include "../imu/ImuPreintegrate.hpp"

namespace cartesian {

/*******************************************************************************
 * FRAME PROCESSING METHODS
 ******************************************************************************/

void Sfm::process_frame(const PendingFrame &frame) {
  if (camera_geoms.count(frame.camera_id) == 0) {
    FATAL("Camera [%d] was not added!", frame.camera_id);
  }

  // Add the pose and speed and biases, once for all the cameras
  if (poses.count(frame.ts) == 0 && add_node(frame.ts) == false) {
    return;
  }

  trackers_[frame.camera_id].track_features(frame.ts,
                                            frame.image,
                                            *camera_geoms.at(frame.camera_id),
                                            max_features);
  add_visual_factors(frame.camera_id, frame.ts);

  // Keep only the most recent frames in the problem
  while (window_size > 0 && static_cast<int>(window.size()) > window_size) {
    slide_window();
  }

  // The imu prediction drifts, optimize to correct it before the next frame
  const bool new_pose = timestamps.back() == frame.ts;
  if (optimize_every > 0 && new_pose && frame.ts == timestamps.back() &&
      static_cast<int>(timestamps.size()) % optimize_every == 0 &&
      static_cast<int>(points.size()) >= optimize_min_points) {
    optimize(optimize_max_iters, false);
  }
}

void Sfm::process_pending_frames() {
  // A frame can be processed once there is imu data at or after its timestamp
  while (pending_frames_.empty() == false) {
    if (imu_buffer.size() == 0) {
      return;
    }
    if (pending_frames_.front().ts > imu_buffer.timestamps.back()) {
      return;
    }

    const PendingFrame frame = pending_frames_.front();
    pending_frames_.pop_front();
    process_frame(frame);
  }
}

bool Sfm::add_node(const timestamp_t ts) {
  if (imu_geoms.empty()) {
    FATAL("Imu was not added!");
  }
  const ImuParams &imu_params = imu_geoms.begin()->second->imu_params;

  // First node, the origin with the attitude from the gravity direction
  if (timestamps.empty()) {
    if (imu_buffer.size() < 2 || ts < imu_buffer.timestamps.front()) {
      return false;
    }

    // Assume the sensor starts at rest: the accelerometer measures gravity,
    // which gives the attitude, and the gyroscope its bias
    const int n = std::min(20, imu_buffer.size());
    const Mat3 C_WS = imu_buffer.estimateAttitude(n);
    Vec3 bg = Vec3::Zero();
    for (int i = 0; i < n; i++) {
      bg += imu_buffer.getGyr(i) / n;
    }
    add_pose(ts, tf(C_WS, Vec3::Zero()));
    add_speed_and_biases(ts, Vec3::Zero(), Vec3::Zero(), bg);
    timestamps.push_back(ts);
    window.push_back(ts);
    imu_buffer.trim_before(ts);
    return true;
  }

  // Check there is imu data between the last and this node
  const timestamp_t ts_km1 = timestamps.back();
  if (ts <= ts_km1 || ts > imu_buffer.timestamps.back()) {
    return false;
  }
  const ImuBuffer imu_data = imu_buffer.extract(ts_km1, ts);
  if (imu_data.size() < 2) {
    return false;
  }

  // Predict the state at k from the state at k - 1 with the imu
  const Vec9 sb_km1 = speed_biases.at(ts_km1);
  const Vec3 v_km1 = sb_km1.segment<3>(0);
  const Vec3 ba_km1 = sb_km1.segment<3>(3);
  const Vec3 bg_km1 = sb_km1.segment<3>(6);
  const Mat4 T_km1 = tf(poses.at(ts_km1));
  const Mat3 C_km1 = tf_rot(T_km1);
  const Vec3 r_km1 = tf_trans(T_km1);

  const ImuPreintegrate pint{imu_params, imu_data, ba_km1, bg_km1};
  const double Dt = pint.Dt;
  const Vec3 &g = imu_params.g;
  const Mat3 C_k = C_km1 * pint.dq.normalized().toRotationMatrix();
  const Vec3 v_k = v_km1 - g * Dt + C_km1 * pint.dv;
  const Vec3 r_k = r_km1 + v_km1 * Dt - 0.5 * g * Dt * Dt + C_km1 * pint.dr;

  // Add pose, speed and biases
  add_pose(ts, tf(C_k, r_k));
  add_speed_and_biases(ts, v_k, ba_km1, bg_km1);

  // Add imu factor
  auto resblock = ImuError::create(imu_params,
                                   imu_data,
                                   poses.at(ts_km1).data(),
                                   speed_biases.at(ts_km1).data(),
                                   poses.at(ts).data(),
                                   speed_biases.at(ts).data());
  const auto id = problem->AddResidualBlock(resblock.get(),
                                            nullptr,
                                            resblock->get_param_ptrs());
  factors_[ts_km1].push_back({resblock, id, -1}); // Of the frame it starts at

  timestamps.push_back(ts);
  window.push_back(ts);
  imu_buffer.trim_before(ts);
  return true;
}

/*******************************************************************************
 * VISUAL FACTOR METHODS
 ******************************************************************************/

void Sfm::add_visual_factors(const int camera_id, const timestamp_t ts) {
  // Triangulate a track from the poses at its first and last measurements
  auto triangulate = [&](const SfmTrack &track, Vec3 &p_W) {
    const timestamp_t ts1 = track.keypoints.begin()->first;
    const timestamp_t ts2 = track.keypoints.rbegin()->first;
    if (ts1 == ts2 || poses.count(ts1) == 0 || poses.count(ts2) == 0) {
      return false;
    }
    return track.triangulate(*camera_geoms.at(track.camera_id),
                             get_camera_pose(ts1, track.camera_id),
                             get_camera_pose(ts2, track.camera_id),
                             min_parallax_deg,
                             max_reproj_error,
                             p_W);
  };

  auto &tracker = trackers_.at(camera_id);
  for (const auto track_id : tracker.tracking) {
    auto &track = tracker.tracks.at(track_id);
    if (track.keypoints.count(ts) == 0) {
      continue;
    }

    // Has a point, add the new measurement. If the point is still the initial
    // guess, replace it once it can be triangulated
    if (track.point_id >= 0) {
      add_reproj_factor(track, ts);
      Vec3 p_W;
      if (track.triangulated == false && triangulate(track, p_W)) {
        points.at(track.point_id) = p_W;
        track.triangulated = true;
      }
      continue;
    }

    // No point yet, triangulate when there is enough parallax. Without it
    // start from a guess along the first ray, the optimization corrects the
    // depth together with the poses. Add all the measurements so far.
    if (static_cast<int>(track.keypoints.size()) < min_track_length) {
      continue;
    }
    Vec3 p_W;
    const bool triangulated = triangulate(track, p_W);
    if (triangulated == false) {
      if (init_depth <= 0.0 || initial_guess(track, p_W) == false) {
        continue;
      }
    }
    add_point(p_W);
    track.point_id = static_cast<int>(points.size()) - 1;
    track.triangulated = triangulated;
    for (const auto &[ts_k, kp] : track.keypoints) {
      if (in_window(ts_k)) {
        add_reproj_factor(track, ts_k);
      }
    }
  }
}

bool Sfm::initial_guess(const SfmTrack &track, Vec3 &p_W) const {
  const timestamp_t ts1 = track.keypoints.begin()->first;
  if (poses.count(ts1) == 0) {
    return false;
  }
  const auto &kp1 = track.keypoints.begin()->second;
  const auto &camera_geom = camera_geoms.at(track.camera_id);
  const Mat4 T_WC1 = get_camera_pose(ts1, track.camera_id);
  const Vec3 ray = camera_geom->unit_ray(Vec2{kp1.pt.x, kp1.pt.y});
  p_W = tf_point(T_WC1, init_depth * ray);
  return true;
}

void Sfm::add_reproj_factor(const SfmTrack &track, const timestamp_t ts) {
  const auto &kp = track.keypoints.at(ts);
  auto resblock = ReprojError::create(camera_geoms.at(track.camera_id),
                                      poses.at(ts).data(),
                                      points.at(track.point_id).data(),
                                      Vec2{kp.pt.x, kp.pt.y},
                                      Mat2::Identity());
  const auto id = problem->AddResidualBlock(resblock.get(),
                                            loss.get(),
                                            resblock->get_param_ptrs());
  factors_[ts].push_back({resblock, id, track.point_id});
  point_refs_[track.point_id]++;
}

/*******************************************************************************
 * WINDOW METHODS
 ******************************************************************************/

bool Sfm::in_window(const timestamp_t ts) const {
  return std::find(window.begin(), window.end(), ts) != window.end();
}

void Sfm::slide_window() {
  const timestamp_t ts = window.front();
  window.pop_front();

  // Marginalize the state of the frame and its factors, the imu factor to the
  // next frame and the visual factors, with the previous prior if there is one.
  // The result is a prior on the next frame and on the points that other frames
  // still see, which carries what is known about them forward instead of
  // holding the next frame fixed. The points that no frame sees anymore are
  // marginalized too.
  std::shared_ptr<MarginalError> prior;
  if (marginalize) {
    set_constant_params();

    ResidualBlockPtrs marg_factors;
    std::vector<const ceres::LossFunction *> losses;
    if (prior_) {
      marg_factors.push_back(prior_);
      losses.push_back(nullptr);
    }
    std::vector<double *> marg_params{poses.at(ts).data(),
                                      speed_biases.at(ts).data()};
    for (const auto &factor : factors_[ts]) {
      marg_factors.push_back(factor.block);
      losses.push_back(factor.point_id >= 0 ? loss.get() : nullptr);
      if (factor.point_id >= 0 && point_refs_.at(factor.point_id) == 1) {
        marg_params.push_back(points.at(factor.point_id).data());
      }
    }

    std::set<double *> fixed;
    for (const auto &factor : marg_factors) {
      for (auto ptr : factor->get_param_ptrs()) {
        if (problem->IsParameterBlockConstant(ptr)) {
          fixed.insert(ptr);
        }
      }
    }
    prior = MarginalError::create(marg_factors, marg_params, fixed, losses);
  }

  // Remove the factors of the frame, the visual factors and the imu factor
  // from the frame to the next one. The factors go before the parameters
  if (prior_) {
    problem->RemoveResidualBlock(prior_id_);
    prior_ = nullptr;
  }
  for (const auto &factor : factors_[ts]) {
    problem->RemoveResidualBlock(factor.id);

    // No frame in the window sees the point anymore
    if (factor.point_id >= 0 && --point_refs_[factor.point_id] == 0) {
      problem->RemoveParameterBlock(points.at(factor.point_id).data());
    }
  }
  factors_.erase(ts);

  // Remove the state of the frame, the estimate stays for the trajectory
  problem->RemoveParameterBlock(poses.at(ts).data());
  problem->RemoveParameterBlock(speed_biases.at(ts).data());

  // Add the prior on the state of the next frame
  if (prior) {
    prior_ = prior;
    prior_id_ = problem->AddResidualBlock(prior_.get(),
                                          nullptr,
                                          prior_->get_param_ptrs());
  }
}

/*******************************************************************************
 * OPTIMIZATION METHODS
 ******************************************************************************/

void Sfm::set_constant_params() {
  for (auto &[camera_id, camera] : camera_geoms) {
    if (estimate_camera == false) {
      problem->SetParameterBlockConstant(camera->intrinsic.data());
      problem->SetParameterBlockConstant(camera->extrinsic.data());
    }
  }
  for (auto &[imu_id, imu] : imu_geoms) {
    problem->SetParameterBlockConstant(imu->extrinsic.data());
  }
}

bool Sfm::optimize(const int max_iters, const bool verbose) {
  if (timestamps.empty()) {
    return false;
  }

  // Fix the origin, the attitude is observable from gravity
  // Frames left the window: the prior from their marginalization constrains
  // the oldest one, without it hold the oldest one fixed
  const bool slid = window.size() < timestamps.size();
  if (slid == false || prior_ == nullptr) {
    const timestamp_t ts_anchor = window.front();
    problem->SetParameterBlockConstant(poses.at(ts_anchor).data());
    if (slid) {
      problem->SetParameterBlockConstant(speed_biases.at(ts_anchor).data());
    }
  }
  set_constant_params();

  // Solve
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = max_iters;
  options.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem.get(), &summary);
  final_cost = summary.final_cost;
  num_iters = summary.iterations.size();
  if (verbose) {
    std::cout << summary.BriefReport() << std::endl;
  }

  return summary.IsSolutionUsable();
}

/*******************************************************************************
 * SETUP METHODS
 ******************************************************************************/

Sfm::Sfm() {
  // Setup solver
  prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.enable_fast_removal = true;
  problem = std::make_shared<ceres::Problem>(prob_options);
  loss = std::make_unique<ceres::HuberLoss>(2.0);
}

void Sfm::add_camera(const int camera_id,
                     const std::string &camera_model,
                     const Vec2i &resolution,
                     const VecX &intrinsic,
                     const Vec7 &extrinsic) {
  // Add camera geometry
  auto camera = CameraGeometry::create(camera_id,
                                       camera_model,
                                       resolution,
                                       intrinsic,
                                       extrinsic);
  camera_geoms[camera_id] = camera;

  // Add camera intrinsic
  const int intrinsic_size = camera->intrinsic.size();
  problem->AddParameterBlock(camera->intrinsic.data(), intrinsic_size);

  // Add camera extrinsic
  problem->AddParameterBlock(camera->extrinsic.data(), 7);
  problem->SetManifold(camera->extrinsic.data(), &pose_plus);
  if (camera_id == 0) {
    problem->SetParameterBlockConstant(camera->extrinsic.data());
  }
}

void Sfm::add_imu(const int imu_id,
                  const ImuParams &imu_params,
                  const Vec7 &extrinsic) {
  // Add imu geometry
  auto imu = std::make_shared<ImuGeometry>(imu_id, imu_params, extrinsic);
  imu_geoms[imu_id] = imu;

  // Add imu extrinsic
  problem->AddParameterBlock(imu->extrinsic.data(), 7);
  problem->SetManifold(imu->extrinsic.data(), &pose_plus);
}

/*******************************************************************************
 * DATA METHODS
 ******************************************************************************/

void Sfm::add_point(const Vec3 &point) {
  const size_t point_id = points.size();
  points[point_id] = point;
  problem->AddParameterBlock(points.at(point_id).data(), 3);
}

void Sfm::add_pose(const timestamp_t ts, const Mat4 &pose) {
  poses[ts] = tf_vec(pose);
  problem->AddParameterBlock(poses.at(ts).data(), 7);
  problem->SetManifold(poses.at(ts).data(), &pose_plus);
}

void Sfm::add_speed_and_biases(const timestamp_t ts,
                               const Vec3 &v_WS,
                               const Vec3 &bias_acc,
                               const Vec3 &bias_gyr) {
  Vec9 sb;
  sb << v_WS, bias_acc, bias_gyr;
  speed_biases[ts] = sb;
  problem->AddParameterBlock(speed_biases.at(ts).data(), 9);
}

void Sfm::add_camera_frame(const timestamp_t ts,
                           const int camera_id,
                           const cv::Mat &image) {
  pending_frames_.push_back({ts, camera_id, image.clone()});
  process_pending_frames();
}

void Sfm::add_imu_measurement(const timestamp_t ts,
                              const Vec3 &imu_acc,
                              const Vec3 &imu_gyr) {
  imu_buffer.add(ts, imu_acc, imu_gyr);
  process_pending_frames();
}

Mat4 Sfm::get_camera_pose(const timestamp_t ts, const int camera_id) const {
  const Mat4 T_WS = tf(poses.at(ts));
  const Mat4 T_SCi = tf(camera_geoms.at(camera_id)->extrinsic);
  return T_WS * T_SCi;
}

/*******************************************************************************
 * OPTIMIZATION METHODS
 ******************************************************************************/

bool Sfm::solve(const bool verbose) { return optimize(50, verbose); }

} // namespace cartesian
