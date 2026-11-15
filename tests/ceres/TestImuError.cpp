#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "sim/SimImu.hpp"
#include "imu/ImuState.hpp"
#include "ceres/ImuError.hpp"
#include "ceres/PoseManifold.hpp"

namespace cartesian {

TEST(ImuError, evaluate) {
  // Setup sim data
  SimImu sim;

  // Setup imu parameters
  ImuParams imu_params;
  imu_params.noise_acc = 0.08;
  imu_params.noise_gyr = 0.004;
  imu_params.noise_ba = 0.00004;
  imu_params.noise_bg = 2.0e-6;

  // Setup imu buffer
  const int start_index = 0;
  const int end_index = 20;
  const ImuBuffer imu_buffer = sim.form_imu_buffer(start_index, end_index);
  // imu_buffer.print();

  // Pose i
  const timestamp_t ts_i = sim.timestamps[start_index];
  const Mat4 T_WS_i = sim.poses[ts_i];
  VecX pose_i = tf_vec(T_WS_i);

  // Pose j
  const timestamp_t ts_j = sim.timestamps[end_index - 1];
  const Mat4 T_WS_j = sim.poses[ts_j];
  VecX pose_j = tf_vec(T_WS_j);

  // Speed and bias i
  const Vec3 vel_i = sim.vel[ts_i];
  const Vec3 ba_i{0.0, 0.0, 0.0};
  const Vec3 bg_i{0.0, 0.0, 0.0};
  Eigen::Vector<double, 9> sb_i;
  sb_i.segment<3>(0) = vel_i;
  sb_i.segment<3>(3) = ba_i;
  sb_i.segment<3>(6) = bg_i;

  // Speed and bias j
  const Vec3 vel_j = sim.vel[ts_j];
  const Vec3 ba_j{0.0, 0.0, 0.0};
  const Vec3 bg_j{0.0, 0.0, 0.0};
  Eigen::Vector<double, 9> sb_j;
  sb_j.segment<3>(0) = vel_j;
  sb_j.segment<3>(3) = ba_j;
  sb_j.segment<3>(6) = bg_j;

  // Form Imu Error
  auto residual_block = ImuError::create(imu_params,
                                         imu_buffer,
                                         pose_i.data(),
                                         sb_i.data(),
                                         pose_j.data(),
                                         sb_j.data());

  // Check residual size and parameter block sizes
  auto block_sizes = residual_block->parameter_block_sizes();
  ASSERT_EQ(residual_block->num_residuals(), 15);
  ASSERT_EQ(block_sizes[0], 7);
  ASSERT_EQ(block_sizes[1], 9);
  ASSERT_EQ(block_sizes[2], 7);
  ASSERT_EQ(block_sizes[3], 9);

  // Check param pointers
  auto param_ptrs = residual_block->get_param_ptrs();
  ASSERT_EQ(param_ptrs[0], pose_i.data());
  ASSERT_EQ(param_ptrs[1], sb_i.data());
  ASSERT_EQ(param_ptrs[2], pose_j.data());
  ASSERT_EQ(param_ptrs[3], sb_j.data());

  Eigen::Vector<double, 15> r;
  residual_block->eval(param_ptrs.data(), r.data(), nullptr);

  // Check Jacobians
  const double h = 1e-8;
  const double tol = 1e-4;
  const bool verbose = false;
  residual_block->set_sqrt_info(eye(15));
  ASSERT_TRUE(residual_block->check_jacobian(0, "J_pose_i", h, tol, verbose));
  ASSERT_TRUE(residual_block->check_jacobian(1, "J_sb_i", h, tol, verbose));
  ASSERT_TRUE(residual_block->check_jacobian(0, "J_pose_j", h, tol, verbose));
  ASSERT_TRUE(residual_block->check_jacobian(1, "J_sb_j", h, tol, verbose));
}

TEST(ImuError, propagation) {
  // Setup sim
  SimImu sim;

  // Setup imu parameters
  ImuParams imu_params;
  imu_params.noise_acc = 0.08;
  imu_params.noise_gyr = 0.004;
  imu_params.noise_ba = 0.00004;
  imu_params.noise_bg = 2.0e-6;

  // Form IMU buffer
  const int start_index = 0;
  const int end_index = 20;
  const ImuBuffer imu_buffer = sim.form_imu_buffer(start_index, end_index);

  // State at timestamp i
  const timestamp_t ts_i = sim.timestamps[start_index];
  const VecX pose_i_gnd = tf_vec(sim.poses[ts_i]);
  const Vec3 vel_i_gnd = sim.vel[ts_i];
  auto state_i = ImuState::create(ts_i, pose_i_gnd, vel_i_gnd);

  // State at timestamp j
  const timestamp_t ts_j = sim.timestamps[end_index - 1];
  const VecX pose_j_gnd = tf_vec(sim.poses[ts_j]);
  const Vec3 vel_j_gnd = sim.vel[ts_j];
  auto state_j = ImuState::create(ts_j, pose_j_gnd, vel_j_gnd);

  // Form residual block
  double *pose_i = state_i->getPosePtr();
  double *sb_i = state_i->getSpeedBiasesPtr();
  double *pose_j = state_j->getPosePtr();
  double *sb_j = state_j->getSpeedBiasesPtr();
  auto resblock =
      ImuError::create(imu_params, imu_buffer, pose_i, sb_i, pose_j, sb_j);

  // Evaluate residual block
  auto param_ptrs = resblock->get_param_ptrs();
  VecX r = zeros(15, 1);
  resblock->Evaluate(param_ptrs.data(), r.data());
  ASSERT_TRUE(r.norm() < 1.0);
}

TEST(ImuError, solve) {
  // Setup imu simulation  data
  SimImu sim;

  // Setup imu parameters
  ImuParams imu_params;
  imu_params.noise_acc = 1e-6;
  imu_params.noise_gyr = 1e-6;
  imu_params.noise_ba = 1e-7;
  imu_params.noise_bg = 1e-7;

  // Setup problem
  std::vector<timestamp_t> state_timestamps;
  std::map<timestamp_t, std::shared_ptr<ImuState>> states_gnd;
  std::map<timestamp_t, std::shared_ptr<ImuState>> states_init;
  std::map<timestamp_t, std::shared_ptr<ImuState>> states_est;
  std::vector<std::shared_ptr<ResidualBlock>> resblocks;

  ceres::Problem::Options prob_options;
  prob_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem{prob_options};
  PoseManifold pose_plus;

  const int N = sim.get_num_measurements();
  int step_size = 20;
  int start_index = 0;
  int end_index = step_size;
  const double dr = 1.0;
  const double drot = 0.1;

  // Pose and velocity at timestamp i
  const timestamp_t ts_init = sim.timestamps[start_index];
  const VecX pose_i_gnd = tf_vec(sim.poses[ts_init]);
  const VecX pose_i_est = tf_vec(tf_perturb(sim.poses[ts_init], dr, drot));
  const Vec3 vel_i = sim.vel[ts_init];
  if (states_gnd.count(ts_init) == 0) {
    state_timestamps.push_back(ts_init);
    states_gnd[ts_init] = ImuState::create(ts_init, pose_i_gnd, vel_i);
    states_init[ts_init] = ImuState::create(ts_init, pose_i_est, vel_i);
    states_est[ts_init] = ImuState::create(ts_init, pose_i_est, vel_i);
  }

  int count = 0;
  while (end_index < N) {
    // Form IMU buffer
    const timestamp_t ts_i = sim.timestamps[start_index];
    const timestamp_t ts_j = sim.timestamps[end_index];
    const ImuBuffer imu_buffer = sim.form_imu_buffer(start_index, end_index);

    // Pose and velocity at timestamp j
    const VecX pose_j_gnd = tf_vec(sim.poses[ts_j]);
    const VecX pose_j_est = tf_vec(tf_perturb(sim.poses[ts_j], dr, drot));
    const Vec3 vel_j = sim.vel[ts_j];
    if (states_gnd.count(ts_j) == 0) {
      state_timestamps.push_back(ts_j);
      states_gnd[ts_j] = ImuState::create(ts_j, pose_j_gnd, vel_j);
      states_init[ts_j] = ImuState::create(ts_j, pose_j_est, vel_j);
      states_est[ts_j] = ImuState::create(ts_j, pose_j_est, vel_j);
    }

    // Form Imu Error
    double *pose_i = states_est[ts_i]->getPosePtr();
    double *sb_i = states_est[ts_i]->getSpeedBiasesPtr();
    double *pose_j = states_est[ts_j]->getPosePtr();
    double *sb_j = states_est[ts_j]->getSpeedBiasesPtr();
    auto resblock =
        ImuError::create(imu_params, imu_buffer, pose_i, sb_i, pose_j, sb_j);
    resblocks.push_back(resblock);

    // Add to problem
    problem.AddParameterBlock(pose_i, 7);
    problem.AddParameterBlock(pose_j, 7);
    problem.AddParameterBlock(sb_i, 9);
    problem.AddParameterBlock(sb_j, 9);
    problem.SetManifold(pose_i, &pose_plus);
    problem.SetManifold(pose_j, &pose_plus);
    problem.AddResidualBlock(resblock.get(), nullptr, resblock->get_param_ptrs());

    // Update indicies
    start_index = end_index;
    end_index += step_size;
    count++;
  }

  // Solver options
  bool verbose = false;
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = verbose;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (verbose) {
    std::cout << summary.FullReport() << std::endl << std::endl;
  }

  // Log
  bool debug = false;
  if (debug) {
    Logger log;
    std::map<timestamp_t, Mat4> traj_gnd;
    std::map<timestamp_t, Mat4> traj_init;
    std::map<timestamp_t, Mat4> traj_est;
    for (size_t k = 0; k < states_gnd.size(); k++) {
      const timestamp_t ts = state_timestamps[k];
      const Mat4 &pose_gnd = tf(states_gnd[ts]->getPose());
      const Mat4 &pose_init = tf(states_init[ts]->getPose());
      const Mat4 &pose_est = tf(states_est[ts]->getPose());

      traj_gnd[ts] = pose_gnd;
      traj_init[ts] = pose_init;
      traj_est[ts] = pose_est;

      log.log_pose("gnd", ts, pose_gnd, 0.1);
      log.log_pose("init", ts, pose_init, 0.1);
      log.log_pose("est", ts, pose_est, 0.1);
    }
    log.log_trajectory("traj_gnd", traj_gnd, Vec3i{255.0, 0.0, 0.0});
    log.log_trajectory("traj_init", traj_init, Vec3i{0.0, 0.0, 255.0});
    log.log_trajectory("traj_est", traj_est, Vec3i{0.0, 255.0, 0.0});
  }
}

} // namespace cartesian
