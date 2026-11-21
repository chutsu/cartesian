#include <gtest/gtest.h>

#include "core/Logger.hpp"
#include "cv/Sfm.hpp"
#include "cv/GridDetector.hpp"
#include "calib/aprilgrid/AprilGridConfig.hpp"
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

} // namespace cartesian
