#pragma once
#include "core/core.hpp"

namespace cartesian {

/* Feature */
struct Feature {
  size_t feature_id = 0;
  std::map<timestamp_t, std::map<int, cv::KeyPoint>> keypoints;
  bool initialized = false;
  double data[3] = {0};

  Feature() = delete;
  Feature(const size_t feature_id_) : feature_id{feature_id_} {}
  virtual ~Feature() = default;

  /** Update feature with new measurement **/
  void update(const timestamp_t ts, const int cam_idx, const cv::KeyPoint &kp);
};

// /** Initialize */
// bool initialize(const timestamp_t ts, const real_t T_WB[4 * 4]) {
//   if (initialized) {
//     return true;
//   }
//
//   // Do we have data?
//   if (keypoints.count(ts) == 0) {
//     return false;
//   }
//
//   // Is the feature tracked long enough?
//   if (length() < min_length) {
//     return false;
//   }
//
//   // Two or more measurements?
//   if (keypoints[ts].size() < 2) {
//     return false;
//   }
//
//   // Triangulate
//   // -- Form projection matrices P0 and P1
//   const real_t *params0 = cam_ints[0].data;
//   const real_t *params1 = cam_ints[1].data;
//
//   real_t I4[4 * 4] = {0};
//   eye(I4, 4, 4);
//
//   POSE2TF(cam_exts[0].data, T_BC0);
//   POSE2TF(cam_exts[1].data, T_BC1);
//   TF_INV(T_BC0, T_C0B);
//   TF_CHAIN(T_C0C1, 2, T_C0B, T_BC1);
//
//   real_t P0[4 * 4] = {0};
//   real_t P1[4 * 4] = {0};
//   pinhole_projection_matrix(params0, I4, P0);
//   pinhole_projection_matrix(params1, T_C0C1, P1);
//
//   // -- Undistort image points z0 and z1
//   const auto kp0 = keypoints[ts][0];
//   const auto kp1 = keypoints[ts][1];
//   const real_t z0_in[2] = {kp0.pt.x, kp0.pt.y};
//   const real_t z1_in[2] = {kp1.pt.x, kp1.pt.y};
//   real_t z0[2] = {0};
//   real_t z1[2] = {0};
//   cam_ints[0].undistort_func(params0, z0_in, z0);
//   cam_ints[1].undistort_func(params1, z1_in, z1);
//
//   // -- Triangulate
//   real_t p_C0[3] = {0};
//   real_t p_W[3] = {0};
//   linear_triangulation(P0, P1, z0, z1, p_C0);
//   if (p_C0[2] <= 0) {
//     return false;
//   }
//   TF_CHAIN(T_WC0, 2, T_WB, T_BC0);
//   tf_point(T_WC0, p_C0, p_W);
//
//   // Update
//   initialized = true;
//   initialize_timestamp = ts;
//   data[0] = p_W[0];
//   data[1] = p_W[1];
//   data[2] = p_W[2];
//
//   return true;
// }
} // namespace cartesian
