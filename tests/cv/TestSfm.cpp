#include <gtest/gtest.h>
#include <cstdlib>

#include "core/Logger.hpp"
#include "cv/Sfm.hpp"
#include "imu/ImuPreintegrate.hpp"
#include "core/SO3.hpp"
#include "cv/GridDetector.hpp"
#include "calib/aprilgrid/AprilGridConfig.hpp"
#include "calib/aprilgrid/AprilGrid.hpp"
#include "calib/aprilgrid/AprilGridDetector.hpp"
#include "calib/SolvePnp.hpp"
#include "camera/CameraGeometry.hpp"

namespace cartesian {

/** Load image paths **/
static std::vector<fs::path> load_image_paths(const fs::path &data_dir) {
  // Get image paths
  std::vector<fs::path> image_paths;
  for (const auto &entry : fs::directory_iterator(data_dir)) {
    image_paths.emplace_back(entry.path());
  }

  // Sort by filename
  std::sort(image_paths.begin(),
            image_paths.end(),
            [](const fs::path &a, const fs::path &b) {
              return a.filename() < b.filename();
            });

  return image_paths;
}

static cv::Mat viz_optflow(const cv::Mat &image_km1,
                           const cv::Mat &image_k,
                           const std::vector<cv::KeyPoint> &kps_km1,
                           const std::vector<cv::KeyPoint> &kps_k,
                           const std::vector<uchar> &inliers) {
  // Draw properties
  const int radius = 2;
  const cv::Scalar red{0, 0, 255};

  // Setup
  cv::Mat viz_track;
  const cv::Mat img_i = image_km1;
  const cv::Mat img_j = image_k;
  const auto kps_i = kps_km1;
  const auto kps_j = kps_k;
  cv::Mat viz_i;
  cv::Mat viz_j;
  cv::cvtColor(img_i, viz_i, cv::COLOR_GRAY2RGB);
  cv::cvtColor(img_j, viz_j, cv::COLOR_GRAY2RGB);

  // Draw KeyPoints
  for (size_t n = 0; n < kps_i.size(); n++) {
    if (inliers[n] == 0) {
      continue;
    }

    auto pt_i = kps_i[n].pt;
    auto pt_j = kps_j[n].pt;
    cv::circle(viz_i, pt_i, radius, red, -1);
    cv::circle(viz_j, pt_j, radius, red, -1);
  }

  // Stitch images horizontally
  cv::hconcat(viz_i, viz_j, viz_track);

  // Draw lines
  for (size_t n = 0; n < kps_i.size(); n++) {
    auto pt_i = kps_i[n].pt;
    auto pt_j = kps_j[n].pt;
    pt_j.x += viz_i.cols;
    const auto line = cv::LINE_AA;
    cv::line(viz_track, pt_i, pt_j, red, 1, line);
  }

  return viz_track;
}

struct Feature {
  size_t id;
  std::vector<timestamp_t> timestamps;
  std::map<timestamp_t, cv::KeyPoint> track;

  Feature() = default;
  Feature(const size_t id_, const timestamp_t ts_, const cv::KeyPoint &kp_) {
    id = id_;
    timestamps.push_back(ts_);
    track[ts_] = kp_;
  }
  virtual ~Feature() = default;

  void update(const timestamp_t ts, const cv::KeyPoint &kp) {
    timestamps.push_back(ts);
    track[ts] = kp;
  }

  cv::KeyPoint last_keypoint() const {
    const auto ts = timestamps.back();
    return track.at(ts);
  }
};

struct FrontEnd {
  // Settings
  int max_features = 100;

  // Data
  size_t frame_id = 0;
  std::map<size_t, Feature> features;
  GridDetector detector;
  cv::Mat image_km1;
  std::set<size_t> features_tracking;

  FrontEnd() { detector.max_keypoints = max_features; }

  size_t add_feature(const timestamp_t ts_k, const cv::KeyPoint &kp) {
    size_t feature_id = features.size();
    features[feature_id] = Feature{feature_id, ts_k, kp};
    return feature_id;
  }

  void update_feature(const size_t feature_id,
                      const timestamp_t ts_k,
                      const cv::KeyPoint &kp) {
    features[feature_id].update(ts_k, kp);
  }

  void get_features_tracking(std::vector<size_t> &feature_ids,
                             std::vector<cv::KeyPoint> &feature_keypoints) {
    feature_ids.clear();
    feature_keypoints.clear();
    for (const auto feature_id : features_tracking) {
      feature_ids.push_back(features[feature_id].id);
      feature_keypoints.push_back(features[feature_id].last_keypoint());
    }
  }

  std::set<size_t> update(const timestamp_t ts_k, const cv::Mat &image_k) {
    // Initialize keypoints
    if (frame_id == 0) {
      std::vector<cv::KeyPoint> kps_new;
      detector.detect(image_k, kps_new);
      for (const auto kp : kps_new) {
        const auto feature_id = add_feature(ts_k, kp);
        features_tracking.insert(feature_id);
      }

      image_km1 = image_k;
      frame_id++;
      return features_tracking;
    }

    // Get features tracking
    std::vector<size_t> feature_ids;
    std::vector<cv::KeyPoint> kps_km1;
    get_features_tracking(feature_ids, kps_km1);

    // Track
    assert(image_km1.empty() == false);
    assert(image_k.empty() == false);
    std::vector<uchar> inliers;
    std::vector<cv::KeyPoint> kps_k;
    optflow_track(image_km1, image_k, kps_km1, kps_k, inliers);

    for (size_t i = 0; i < inliers.size(); ++i) {
      const auto fid = feature_ids[i];
      if (inliers[i] == 0) {
        features_tracking.erase(fid);
        continue;
      }
      update_feature(fid, ts_k, kps_k[i]);
    }

    // Add new features
    if (features_tracking.size() < (max_features * 0.7)) {
      std::vector<cv::KeyPoint> kps_prev;
      for (const auto feature_id : features_tracking) {
        kps_prev.emplace_back(features[feature_id].last_keypoint());
      }

      std::vector<cv::KeyPoint> kps_new;
      detector.detect(image_k, kps_new, kps_prev);
      for (const auto kp : kps_new) {
        const auto feature_id = add_feature(ts_k, kp);
        features_tracking.insert(feature_id);
      }
    }

    // const auto viz_track =
    //     viz_optflow(image_km1, image_k, kps_km1, kps_k, inliers);
    // cv::imshow("Tracking", viz_track);

    // Update
    image_km1 = image_k;
    frame_id++;

    return features_tracking;
  }
};

static AprilGridConfig setup_target_config() {
  AprilGridConfig target_config;
  target_config.target_id = 0;
  target_config.tag_rows = 6;
  target_config.tag_cols = 6;
  target_config.tag_size = 0.088;
  target_config.tag_spacing = 0.3;
  target_config.tag_id_offset = 0;
  return target_config;
}

TEST(Sfm, test) {
  // Load data
  Logger log;
  const auto data_dir = fs::path("/data/euroc/imu_april/mav0/cam0/data");
  const auto image_paths = load_image_paths(data_dir);

  // Camera geometry
  const int cam_id = 0;
  const std::string cam_model = "KannalaBrandt4";
  const Vec2i res{754, 480};
  const double fx = 458.654;
  const double fy = 457.296;
  const double cx = 367.215;
  const double cy = 248.375;
  const double k1 = -0.28340811;
  const double k2 = 0.07395907;
  const double p1 = 0.00019359;
  const double p2 = 1.76187114e-05;
  const Vec8 intrinsic{fx, fy, cx, cy, k1, k2, p1, p2};
  CameraGeometry cam_geom(cam_id, cam_model, res, intrinsic);

  // Camera matrix K
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = fx;
  K.at<double>(1, 1) = fy;
  K.at<double>(0, 2) = cx;
  K.at<double>(1, 2) = cy;

  // Feature detector
  FrontEnd front_end;

  // Calibration target detector
  const auto target_config = setup_target_config();
  AprilGridDetector target_detector{target_config};
  log.log_target("/calib_target", target_config, eye(4));

  // Loop through data
  size_t frame_id = 0;
  int wait = 0;
  cv::Mat R_world_camera = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t_world_camera = cv::Mat::zeros(3, 1, CV_64F);
  std::map<timestamp_t, Mat4> poses_est;
  std::map<timestamp_t, Mat4> poses_gnd;

  for (const auto &image_path : image_paths) {
    // Load image
    const auto image_k = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

    // Detect AprilGrid
    auto grids = target_detector.detect(frame_id, 0, image_k);
    if (grids[0]->detected()) {
      // Estimate relative pose with calibration target
      std::vector<int> point_ids;
      Vec2s keypoints;
      Vec3s object_points;
      grids[0]->get_measurements(point_ids, keypoints, object_points);

      Mat4 T_camera_target;
      SolvePnp solvepnp{cam_geom};
      solvepnp.estimate(keypoints, object_points, T_camera_target);

      const Mat4 T_target_camera = T_camera_target.inverse();
      poses_gnd[frame_id] = T_target_camera;
      log.log_image("/image", frame_id, image_k);
      // log.log_pose("/solvepnp", frame_id, T_target_camera, 0.3);

      // if (frame_id == 0) {
      //   const Mat3 R_target_camera = T_target_camera.block<3, 3>(0, 0);
      //   const Vec3 t_target_camera = T_target_camera.block<3, 1>(0, 3);
      //   cv::eigen2cv(R_target_camera, R_world_camera);
      //   cv::eigen2cv(t_target_camera, t_world_camera);
      // }
    }

    // Track features
    front_end.update(frame_id, image_k);
    if (frame_id > 1) {
      // Form cv::Point2f from previous to current frame
      std::vector<size_t> feature_ids;
      std::vector<cv::Point2f> pts1;
      std::vector<cv::Point2f> pts2;
      for (const auto feature_id : front_end.features_tracking) {
        // Check feature length
        const auto &feature = front_end.features[feature_id];
        if (feature.track.size() < 2) {
          continue;
        }

        // Undistort keypoints
        Vec2 kp1{feature.track.at(frame_id - 1).pt.x,
                 feature.track.at(frame_id - 1).pt.y};
        Vec2 kp2{feature.track.at(frame_id).pt.x,
                 feature.track.at(frame_id).pt.y};
        kp1 = cam_geom.undistort(kp1);
        kp2 = cam_geom.undistort(kp2);

        // Emplace
        feature_ids.push_back(feature_id);
        pts1.emplace_back(kp1.x(), kp1.y());
        pts2.emplace_back(kp2.x(), kp2.y());
      }
      printf("frame_id: %ld\n", frame_id);
      printf("num features: %ld\n", feature_ids.size());

      // Estimate Essential matrix E
      const double threshold = 3.0;
      const double confidence = 0.999;
      std::vector<uchar> mask;
      const cv::Mat E = cv::findEssentialMat(pts1,
                                             pts2,
                                             K,
                                             cv::RANSAC,
                                             confidence,
                                             threshold,
                                             1000,
                                             mask);
      {
        size_t num_inliers = 0;
        size_t num_outliers = 0;
        size_t num_total = 0;
        float inlier_ratio = 0.0f;
        inlier_stats(mask, num_inliers, num_outliers, num_total, inlier_ratio);
        printf("cv::FindEssentialMat(): inliers: %ld/%ld, inlier ratio: %f\n",
               num_inliers,
               num_total,
               inlier_ratio);
      }

      // Recover pose
      cv::Mat dR, dt;
      cv::recoverPose(E, pts1, pts2, K, dR, dt, mask);
      {
        size_t num_inliers = 0;
        size_t num_outliers = 0;
        size_t num_total = 0;
        float inlier_ratio = 0.0f;
        inlier_stats(mask, num_inliers, num_outliers, num_total, inlier_ratio);
        printf("cv::recoverPose(): inliers: %ld/%ld, inlier ratio: %f\n",
               num_inliers,
               num_total,
               inlier_ratio);
      }
      t_world_camera = dR * t_world_camera + dt;
      R_world_camera = dR * R_world_camera;

      {
        Mat4 T_world_camera_eigen;
        Mat3 R_world_camera_eigen;
        Vec3 t_world_camera_eigen;
        cv::cv2eigen(t_world_camera, t_world_camera_eigen);
        cv::cv2eigen(R_world_camera, R_world_camera_eigen);

        T_world_camera_eigen.setIdentity();
        T_world_camera_eigen.block<3, 3>(0, 0) = R_world_camera_eigen;
        T_world_camera_eigen.block<3, 1>(0, 3) = t_world_camera_eigen;
        poses_est[frame_id] = T_world_camera_eigen;
        log.log_pose("/estimate", frame_id, T_world_camera_eigen, 0.3);
      }

      // Stop tracking outliers
      // int num_inliers = 0;
      // int num_outliers = 0;
      // for (size_t i = 0; i < mask.size(); ++i) {
      //   if (mask[i] == 0) {
      //     front_end.features_tracking.erase(feature_ids[i]);
      //     num_outliers++;
      //   } else {
      //     num_inliers++;
      //   }
      // }
      // printf("mask size: %ld\n", mask.size());
      // printf("inliers:   %d\n", num_inliers);
      // printf("outliers:  %d\n", num_outliers);
    }

    // Update
    frame_id++;

    // Visualize
    // cv::Mat viz = image_k;
    // cv::hconcat(image_color, image_debug, viz);
    // cv::imshow("Image", image_k);
    // auto ch = cv::waitKey(wait * 10);
    // if (ch == 'q') {
    //   break;
    // } else if (ch == 'p') {
    //   wait ^= 1;
    // }
  }

  // cv::waitKey(0);
}

TEST(Sfm, add_sensors_and_data) {
  Sfm sfm;

  // Camera
  const Vec8 intrinsic{458.654, 457.296, 367.215, 248.375, 0.0, 0.0, 0.0, 0.0};
  sfm.add_camera(0, "BrownConrady4", Vec2i{752, 480}, intrinsic, tf_vec());
  const auto &camera = sfm.camera_geoms.at(0);
  ASSERT_EQ(sfm.camera_geoms.size(), 1u);
  ASSERT_TRUE(sfm.problem->HasParameterBlock(camera->intrinsic.data()));
  ASSERT_TRUE(sfm.problem->IsParameterBlockConstant(camera->extrinsic.data()));

  // Imu
  ImuParams imu_params;
  imu_params.imu_id = 0;
  sfm.add_imu(0, imu_params, tf_vec());
  const auto &imu = sfm.imu_geoms.at(0);
  ASSERT_EQ(sfm.imu_geoms.size(), 1u);
  ASSERT_TRUE(sfm.problem->HasParameterBlock(imu->extrinsic.data()));

  // Points, ids are sequential
  const Vec3 p0{1.0, 2.0, 3.0};
  const Vec3 p1{4.0, 5.0, 6.0};
  sfm.add_point(p0);
  sfm.add_point(p1);
  ASSERT_EQ(sfm.points.size(), 2u);
  ASSERT_NEAR((sfm.points.at(0) - p0).norm(), 0.0, 1e-12);
  ASSERT_NEAR((sfm.points.at(1) - p1).norm(), 0.0, 1e-12);
  ASSERT_TRUE(sfm.problem->HasParameterBlock(sfm.points.at(1).data()));

  // Poses
  const Vec7 pose_vec{0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
  const Mat4 pose = tf(pose_vec);
  sfm.add_pose(10, pose);
  ASSERT_EQ(sfm.poses.size(), 1u);
  ASSERT_TRUE(sfm.problem->HasParameterBlock(sfm.poses.at(10).data()));
  ASSERT_NEAR((tf(sfm.poses.at(10)) - pose).norm(), 0.0, 1e-12);

  // Speed and biases
  const Vec3 v_WS{0.1, 0.2, 0.3};
  const Vec3 ba{0.01, 0.02, 0.03};
  const Vec3 bg{0.001, 0.002, 0.003};
  sfm.add_speed_and_biases(10, v_WS, ba, bg);
  const Vec9 &sb = sfm.speed_biases.at(10);
  ASSERT_EQ(sfm.speed_biases.size(), 1u);
  ASSERT_TRUE(sfm.problem->HasParameterBlock(sfm.speed_biases.at(10).data()));
  ASSERT_NEAR((sb.segment<3>(0) - v_WS).norm(), 0.0, 1e-12);
  ASSERT_NEAR((sb.segment<3>(3) - ba).norm(), 0.0, 1e-12);
  ASSERT_NEAR((sb.segment<3>(6) - bg).norm(), 0.0, 1e-12);

  // The biases are zero by default
  sfm.add_speed_and_biases(20, v_WS);
  const Vec9 &sb_default = sfm.speed_biases.at(20);
  ASSERT_NEAR((sb_default.segment<3>(0) - v_WS).norm(), 0.0, 1e-12);
  ASSERT_NEAR(sb_default.segment<6>(3).norm(), 0.0, 1e-12);
}

using Positions = std::map<timestamp_t, Vec3>;
using Poses = std::map<timestamp_t, Mat4>;
static const std::string kEurocRoot = "/data/euroc/imu_april/mav0";

/** Rigid transform that aligns the estimated positions to the ground truth */
static Mat4 align_trajectory(const Positions &est, const Positions &gnd) {
  std::vector<Vec3> p_est;
  std::vector<Vec3> p_gnd;
  for (const auto &[ts, p] : est) {
    if (gnd.count(ts)) {
      p_est.push_back(p);
      p_gnd.push_back(gnd.at(ts));
    }
  }

  Eigen::Matrix<double, 3, Eigen::Dynamic> src(3, p_est.size());
  Eigen::Matrix<double, 3, Eigen::Dynamic> dst(3, p_est.size());
  for (size_t i = 0; i < p_est.size(); i++) {
    src.col(i) = p_est[i];
    dst.col(i) = p_gnd[i];
  }
  return Eigen::umeyama(src, dst, false); // No scale, it is metric
}

/** Position error [m] of the camera trajectory after a rigid alignment */
static double trajectory_rmse(const Positions &est, const Positions &gnd) {
  const Mat4 T = align_trajectory(est, gnd);

  double sse = 0.0;
  int n = 0;
  for (const auto &[ts, p_est] : est) {
    if (gnd.count(ts) == 0) {
      continue;
    }
    const Vec3 p = T.block<3, 3>(0, 0) * p_est + T.block<3, 1>(0, 3);
    sse += (p - gnd.at(ts)).squaredNorm();
    n++;
  }
  return std::sqrt(sse / n);
}

/**
 * Visualize the result in rerun, in the frame of the calibration target:
 * the target, the estimated and ground truth camera trajectories and poses, the
 * points, the images and the position error. The estimate is aligned to the
 * ground truth the same way as for the error.
 */
static void visualize_sfm(const Sfm &sfm, const Poses &T_gnd) {
  const Vec3i red{255, 0, 0};
  const Vec3i green{0, 255, 0};
  const Vec3i gray{150, 150, 150};
  const int camera_id = 0;
  Logger log{"sfm_euroc_vio", "euroc imu_april"};

  // Align the estimate to the ground truth
  Positions p_est;
  Positions p_gnd;
  for (const auto ts : sfm.timestamps) {
    p_est[ts] = tf_trans(sfm.get_camera_pose(ts, camera_id));
    if (T_gnd.count(ts)) {
      p_gnd[ts] = tf_trans(T_gnd.at(ts));
    }
  }
  const Mat4 T = align_trajectory(p_est, p_gnd);

  // Calibration target
  log.log_target("/target", setup_target_config(), eye(4));

  // Trajectories
  Poses T_est;
  for (const auto ts : sfm.timestamps) {
    T_est[ts] = T * sfm.get_camera_pose(ts, camera_id);
  }
  log.log_trajectory("/est/trajectory", T_est, red);
  log.log_trajectory("/gt/trajectory", T_gnd, green);

  // Points
  std::vector<Vec3> points;
  for (const auto &[point_id, p_W] : sfm.points) {
    points.push_back(T.block<3, 3>(0, 0) * p_W + T.block<3, 1>(0, 3));
  }
  const std::vector<Vec3i> colors(points.size(), gray);
  const std::vector<double> radii(points.size(), 0.004);
  log.log_points("/map/points", points, colors, radii);

  // Images, poses and the position error, over time
  log.init_series_line("/error/position", red, 1.0f);
  for (const auto ts : sfm.timestamps) {
    log.log_pose("/est/camera", ts, T_est.at(ts), 0.05);

    const std::string name = std::to_string(ts) + ".png";
    const fs::path image_path = kEurocRoot + "/cam0/data/" + name;
    log.log_image("/cam0/image", ts, cv::imread(image_path));

    if (T_gnd.count(ts) == 0) {
      continue;
    }
    log.log_pose("/gt/camera", ts, T_gnd.at(ts), 0.05);
    const Vec3 p = tf_trans(T_est.at(ts));
    const double error = (p - p_gnd.at(ts)).norm();
    log.log_scalar("/error/position", ts, error);
  }
}

/**
 * Run the visual-inertial Sfm on the first `num_frames` frames of the EuRoC
 * sequence, with a sliding window of `window_size` frames (0 for all the
 * frames), and compare the camera trajectory to the AprilGrid poses.
 * `SFM_NUM_FRAMES` and `SFM_WINDOW_SIZE` override the arguments, and
 * `SFM_MARGINALIZE=0` holds the oldest frame fixed instead of a prior, and then the
 * results are not checked. 1440 frames is the whole sequence.
 */
static void test_euroc_vio(const int default_num_frames,
                           const int default_window_size,
                           const double max_rmse) {
  int num_frames = default_num_frames;
  const char *num_frames_env = std::getenv("SFM_NUM_FRAMES");
  if (num_frames_env) {
    num_frames = std::atoi(num_frames_env);
  }
  int window_size = default_window_size;
  const char *window_size_env = std::getenv("SFM_WINDOW_SIZE");
  if (window_size_env) {
    window_size = std::atoi(window_size_env);
  }
  const bool overridden =
      num_frames != default_num_frames || window_size != default_window_size;

  // Camera
  const Vec2i res{752, 480};
  const double fx = 458.654;
  const double fy = 457.296;
  const double cx = 367.215;
  const double cy = 248.375;
  const double k1 = -0.28340811;
  const double k2 = 0.07395907;
  const double p1 = 0.00019359;
  const double p2 = 1.76187114e-05;
  const Vec8 intrinsic{fx, fy, cx, cy, k1, k2, p1, p2};
  // clang-format off
  Mat4 T_SC0;
  T_SC0 <<
    0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
    0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
    -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
    0.0, 0.0, 0.0, 1.0;
  // clang-format on

  // Imu
  ImuParams imu_params;
  imu_params.imu_id = 0;
  // The datasheet noise densities are 2.0e-3 and 1.6968e-4, but they are too
  // optimistic: the imu factors then outweigh the visual ones, and the estimate
  // diverges once the camera and the imu disagree by a fraction of a degree.
  // Use the inflated values that are common for this sensor (see OKVIS).
  imu_params.acc_noise_density = 8.0e-3;
  imu_params.gyr_noise_density = 1.2e-3;
  imu_params.bias_walk_acc = 3.0e-3;
  imu_params.bias_walk_gyr = 1.9393e-5;

  Sfm sfm;
  sfm.window_size = window_size;
  if (const char *marg = std::getenv("SFM_MARGINALIZE")) {
    sfm.marginalize = std::atoi(marg) != 0;
  }
  sfm.add_camera(0, "BrownConrady4", res, intrinsic, tf_vec(T_SC0));
  sfm.add_imu(0, imu_params, tf_vec());

  // Load imu data
  struct Imu {
    timestamp_t ts;
    Vec3 acc;
    Vec3 gyr;
  };
  std::vector<Imu> imu_data;
  {
    const fs::path imu_path = kEurocRoot + "/imu0/data.csv";
    FILE *fp = fopen(imu_path.c_str(), "r");
    ASSERT_TRUE(fp != nullptr);
    skip_line(fp);
    const int rows = file_rows(imu_path) - 1;
    const char *fmt = "%ld,%lf,%lf,%lf,%lf,%lf,%lf";
    for (int i = 0; i < rows; i++) {
      Imu m;
      double *g = m.gyr.data();
      double *a = m.acc.data();
      int n = fscanf(fp, fmt, &m.ts, &g[0], &g[1], &g[2], &a[0], &a[1], &a[2]);
      ASSERT_EQ(n, 7);
      imu_data.push_back(m);
    }
    fclose(fp);
  }

  // Load the camera frames
  const std::string image_dir = kEurocRoot + "/cam0/data";
  std::vector<fs::path> image_paths = load_image_paths(image_dir);
  image_paths.resize(std::min<size_t>(num_frames, image_paths.size()));

  // Ground truth camera positions in the target frame, from the AprilGrid
  CameraGeometry cam_geom{0, "BrownConrady4", res, intrinsic};
  Positions p_gnd;
  Poses T_gnd;

  // Run
  size_t imu_idx = 0;
  for (const auto &image_path : image_paths) {
    const timestamp_t ts = std::stoull(image_path.stem().string());

    // Feed the imu measurements up to this frame
    while (imu_idx < imu_data.size() && imu_data[imu_idx].ts <= ts) {
      const auto &m = imu_data[imu_idx++];
      sfm.add_imu_measurement(m.ts, m.acc, m.gyr);
    }
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    sfm.add_camera_frame(ts, 0, image);

    // Ground truth
    const std::string name = std::to_string(ts) + ".csv";
    const fs::path target_path = kEurocRoot + "/target0/cam0/" + name;
    auto target = AprilGrid::load(target_path);
    if (target->detected() == false) {
      continue;
    }

    std::vector<int> point_ids;
    Vec2s keypoints;
    Vec3s object_points;
    target->get_measurements(point_ids, keypoints, object_points);
    if (keypoints.size() < 10) {
      continue;
    }

    Mat4 T_CT;
    SolvePnp pnp{cam_geom};
    if (pnp.estimate(keypoints, object_points, T_CT) != 0) {
      continue;
    }
    T_gnd[ts] = T_CT.inverse();
    p_gnd[ts] = tf_trans(T_gnd[ts]);
  }

  // Imu data after the last frame, so it gets processed
  const timestamp_t ts_last = std::stoull(image_paths.back().stem().string());
  while (imu_idx < imu_data.size() && imu_data[imu_idx].ts <= ts_last + 5e7) {
    const auto &m = imu_data[imu_idx++];
    sfm.add_imu_measurement(m.ts, m.acc, m.gyr);
  }

  // Trajectory from the optimization while the frames were added
  auto camera_positions = [&]() {
    Positions p_est;
    for (const auto ts : sfm.timestamps) {
      p_est[ts] = tf_trans(sfm.get_camera_pose(ts, 0));
    }
    return p_est;
  };
  const double rmse_online = trajectory_rmse(camera_positions(), p_gnd);
  const size_t num_points = sfm.points.size();
  const size_t num_resblocks = sfm.problem->NumResidualBlocks();
  const bool ok = sfm.solve();
  const double rmse_solved = trajectory_rmse(camera_positions(), p_gnd);

  printf("\n");
  printf("frames:       %ld (of %d)\n", sfm.timestamps.size(), num_frames);
  printf("window:       %ld frames, size %d\n", sfm.window.size(), window_size);
  printf("ground truth: %ld frames\n", p_gnd.size());
  printf("points:       %ld\n", num_points);
  printf("factors:      %ld\n", num_resblocks);
  printf("solver:       %s\n", ok ? "ok" : "failed");
  printf("cost:         %.3e, %d iters\n", sfm.final_cost, sfm.num_iters);
  printf("rmse online:  %.4f m\n", rmse_online);
  printf("rmse solved:  %.4f m\n", rmse_solved);

  // Visualize in rerun with `SFM_VISUALIZE=1`, it opens the viewer
  // if (std::getenv("SFM_VISUALIZE") != nullptr) {
    visualize_sfm(sfm, T_gnd);
  // }

  // The camera moves up to about 0.5 m, and the trajectory is metric, so the
  // error is in meters
  if (overridden == false) {
    EXPECT_GE(sfm.timestamps.size(), 0.9 * num_frames);
    EXPECT_GE(num_points, 100u);
    EXPECT_TRUE(ok);
    EXPECT_LT(rmse_online, max_rmse);
    EXPECT_LT(rmse_solved, max_rmse);
    if (window_size > 0) {
      EXPECT_EQ(sfm.window.size(), static_cast<size_t>(window_size));
    }
  }
}

TEST(Sfm, euroc_vio) {
  test_euroc_vio(150, 0, 0.05);
}

TEST(Sfm, euroc_vio_sliding_window) {
  test_euroc_vio(500, 30, 0.05);
}

} // namespace cartesian
