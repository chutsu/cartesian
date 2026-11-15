#include <gtest/gtest.h>

#include "core/Core.hpp"
#include "dataset/Euroc.hpp"

namespace cartesian {

struct EurocTestData {
  const fs::path data_dir = "/tmp/euroc";
  const fs::path cam0_dir = data_dir / "mav0" / "cam0";
  const fs::path cam1_dir = data_dir / "mav0" / "cam1";
  const fs::path imu_dir = data_dir / "mav0" / "imu0";
  const fs::path gnd_dir = data_dir / "mav0" / "state_groundtruth_estimate0";
  const fs::path target_config_path = data_dir / "april_6x6.yaml";

  void setup_euroc_imu_test_data(const fs::path &data_dir) {
    // Create data directory
    fs::create_directories(data_dir);

    // Create imu0/sensor.yaml
    char sensor_config_path[100] = {0};
    sprintf(sensor_config_path, "%s/sensor.yaml", data_dir.string().c_str());
    const char sensor_config[1024] =
        "sensor_type: imu                       \n"
        "comment: VI-Sensor IMU (ADIS16448)     \n"
        "T_BS:                                  \n"
        "  cols: 4                              \n"
        "  rows: 4                              \n"
        "  data: [1.0, 0.0, 0.0, 0.0,           \n"
        "         0.0, 1.0, 0.0, 0.0,           \n"
        "         0.0, 0.0, 1.0, 0.0,           \n"
        "         0.0, 0.0, 0.0, 1.0]           \n"
        "rate_hz: 200                           \n"
        "gyroscope_noise_density: 1             \n"
        "gyroscope_random_walk: 2               \n"
        "accelerometer_noise_density: 3         \n"
        "accelerometer_random_walk: 4           \n";
    FILE *sensor_yaml = fopen(sensor_config_path, "w");
    fprintf(sensor_yaml, sensor_config);
    fclose(sensor_yaml);

    // Create imu0/data.csv
    char data_csv_path[100] = {0};
    sprintf(data_csv_path, "%s/data.csv", data_dir.string().c_str());
    FILE *data_csv = fopen(data_csv_path, "w");
    fprintf(data_csv, "#header\n");
    fprintf(data_csv, "1,2,3,4,5,6,7\n");
    fclose(data_csv);
  }

  void setup_euroc_camera_test_data(const fs::path &data_dir) {
    // Create data directory
    fs::create_directories(data_dir);

    // Create cam0/sensor.yaml
    char sensor_config_path[100] = {0};
    sprintf(sensor_config_path, "%s/sensor.yaml", data_dir.string().c_str());
    const char sensor_config[1024] = "sensor_type: camera                   \n"
                                     "comment: VI-Sensor cam0 (MT9M034)     \n"
                                     "T_BS:                                 \n"
                                     "  cols: 4                             \n"
                                     "  rows: 4                             \n"
                                     "  data: [1.0, 0.0, 0.0, 0.0,          \n"
                                     "         0.0, 1.0, 0.0, 0.0,          \n"
                                     "         0.0, 0.0, 1.0, 0.0,          \n"
                                     "         0.0, 0.0, 0.0, 1.0]          \n"
                                     "rate_hz: 20                           \n"
                                     "resolution: [752, 480]                \n"
                                     "camera_model: pinhole                 \n"
                                     "intrinsics: [1, 2, 3, 4]              \n"
                                     "distortion_model: radial-tangential   \n"
                                     "distortion_coefficients: [1, 2, 3, 4] \n";
    FILE *sensor_yaml = fopen(sensor_config_path, "w");
    fprintf(sensor_yaml, sensor_config);
    fclose(sensor_yaml);

    // Create cam0/data.csv
    char data_csv_path[100] = {0};
    sprintf(data_csv_path, "%s/data.csv", data_dir.string().c_str());
    FILE *data_csv = fopen(data_csv_path, "w");
    fprintf(data_csv, "#timestamp [ns],filename\n");
    fprintf(data_csv, "1403636579763555584,1403636579763555584.png\n");
    fclose(data_csv);

    // Create cam0/data/1403636579763555584.png
    fs::path image_dir = data_dir / "data";
    fs::path image_path = image_dir / "1403636579763555584.png";
    fs::create_directories(image_dir);
    FILE *data_png = fopen(image_path.string().c_str(), "w");
    fclose(data_png);
  }

  void setup_euroc_ground_truth_test_data(const fs::path &data_dir) {
    // Create data directory
    fs::create_directories(data_dir);

    // Create ground_truth/sensor.yaml
    char sensor_config_path[100] = {0};
    sprintf(sensor_config_path, "%s/sensor.yaml", data_dir.string().c_str());
    const char sensor_config[1024] = "sensor_type: visual-inertial          \n"
                                     "comment: Testing                      \n"
                                     "T_BS:                                 \n"
                                     "  cols: 4                             \n"
                                     "  rows: 4                             \n"
                                     "  data: [1.0, 0.0, 0.0, 0.0,          \n"
                                     "         0.0, 1.0, 0.0, 0.0,          \n"
                                     "         0.0, 0.0, 1.0, 0.0,          \n"
                                     "         0.0, 0.0, 0.0, 1.0]          \n";
    FILE *sensor_yaml = fopen(sensor_config_path, "w");
    fprintf(sensor_yaml, sensor_config);
    fclose(sensor_yaml);

    // Create ground_truth/data.csv
    char data_csv_path[100] = {0};
    sprintf(data_csv_path, "%s/data.csv", data_dir.string().c_str());
    FILE *data_csv = fopen(data_csv_path, "w");
    fprintf(data_csv, "#header\n");
    fprintf(data_csv, "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17\n");
    fclose(data_csv);
  }

  void setup_euroc_calib_target_test_config(const fs::path &data_dir) {
    // Create data directory
    fs::create_directories(data_dir);

    // Create calibration target config
    char calib_config_path[100] = {0};
    sprintf(calib_config_path, "%s/april_6x6.yaml", data_dir.string().c_str());
    FILE *fp = fopen(calib_config_path, "w");
    fprintf(fp, "target_type: 'aprilgrid'\n");
    fprintf(fp, "tagCols: 6              \n");
    fprintf(fp, "tagRows: 6              \n");
    fprintf(fp, "tagSize: 0.088          \n");
    fprintf(fp, "tagSpacing: 0.3         \n");
    fclose(fp);
  }

  EurocTestData() {
    // Create data directory
    fs::create_directories(data_dir);

    // Create euroc test data
    const fs::path imu_data_dir = data_dir / "mav0/imu0";
    const fs::path cam0_data_dir = data_dir / "mav0/cam0";
    const fs::path cam1_data_dir = data_dir / "mav0/cam1";
    const fs::path gnd_data_dir = data_dir / "mav0/state_groundtruth_estimate0";
    setup_euroc_imu_test_data(imu_data_dir);
    setup_euroc_camera_test_data(cam0_data_dir);
    setup_euroc_camera_test_data(cam1_data_dir);
    setup_euroc_ground_truth_test_data(gnd_data_dir);
    setup_euroc_calib_target_test_config(gnd_data_dir);
    setup_euroc_calib_target_test_config(data_dir);
  }

  ~EurocTestData() {
    // Remove data directory
    fs::remove_all(data_dir);
  }
};

TEST(Euroc, load_imu) {
  EurocTestData test_data;
  EurocImu data(test_data.imu_dir);

  ASSERT_TRUE(data.timestamps.size() == 1);
  ASSERT_TRUE(data.timestamps[0] == 1);
  ASSERT_FLOAT_EQ(data.w_B[0].x(), 2.0);
  ASSERT_FLOAT_EQ(data.w_B[0].y(), 3.0);
  ASSERT_FLOAT_EQ(data.w_B[0].z(), 4.0);
  ASSERT_FLOAT_EQ(data.a_B[0].x(), 5.0);
  ASSERT_FLOAT_EQ(data.a_B[0].y(), 6.0);
  ASSERT_FLOAT_EQ(data.a_B[0].z(), 7.0);
  ASSERT_TRUE((data.T_BS - eye(4)).norm() < 1e-8);
  ASSERT_TRUE(data.sensor_type == "imu");
  ASSERT_TRUE(data.comment == "VI-Sensor IMU (ADIS16448)");
  ASSERT_FLOAT_EQ(data.rate_hz, 200.0);
  ASSERT_FLOAT_EQ(data.gyro_noise_density, 1.0);
  ASSERT_FLOAT_EQ(data.gyro_random_walk, 2.0);
  ASSERT_FLOAT_EQ(data.accel_noise_density, 3.0);
  ASSERT_FLOAT_EQ(data.accel_random_walk, 4.0);
}

TEST(Euroc, load_camera) {
  EurocTestData test_data;
  EurocCamera data(test_data.cam0_dir, 0);

  ASSERT_EQ(data.timestamps.size(), 1);
  ASSERT_EQ(data.timestamps[0], 1403636579763555584);
  ASSERT_TRUE((data.T_BS - eye(4)).norm() < 1e-8);
  ASSERT_TRUE(data.sensor_type == "camera");
  ASSERT_TRUE(data.comment == "VI-Sensor cam0 (MT9M034)");
  ASSERT_FLOAT_EQ(data.rate_hz, 20.0);
  ASSERT_TRUE(data.camera_model == "pinhole");
  ASSERT_FLOAT_EQ(data.intrinsics[0], 1.0);
  ASSERT_FLOAT_EQ(data.intrinsics[1], 2.0);
  ASSERT_FLOAT_EQ(data.intrinsics[2], 3.0);
  ASSERT_FLOAT_EQ(data.intrinsics[3], 4.0);
  ASSERT_TRUE(data.distortion_model == "radial-tangential");
  ASSERT_FLOAT_EQ(data.distortion_coefficients[0], 1.0);
  ASSERT_FLOAT_EQ(data.distortion_coefficients[1], 2.0);
  ASSERT_FLOAT_EQ(data.distortion_coefficients[2], 3.0);
  ASSERT_FLOAT_EQ(data.distortion_coefficients[3], 4.0);
}

TEST(Euroc, load_ground_truth) {
  EurocTestData test_data;
  EurocGroundTruth data{test_data.gnd_dir};

  ASSERT_TRUE(data.timestamps.size() == 1);
  ASSERT_TRUE(data.timestamps[0] == 1);
  ASSERT_FLOAT_EQ(data.p_RS_R[0].x(), 2);
  ASSERT_FLOAT_EQ(data.p_RS_R[0].y(), 3);
  ASSERT_FLOAT_EQ(data.p_RS_R[0].z(), 4);
  ASSERT_FLOAT_EQ(data.q_RS[0].w(), 5);
  ASSERT_FLOAT_EQ(data.q_RS[0].x(), 6);
  ASSERT_FLOAT_EQ(data.q_RS[0].y(), 7);
  ASSERT_FLOAT_EQ(data.q_RS[0].z(), 8);
  ASSERT_FLOAT_EQ(data.v_RS_R[0].x(), 9);
  ASSERT_FLOAT_EQ(data.v_RS_R[0].y(), 10);
  ASSERT_FLOAT_EQ(data.v_RS_R[0].z(), 11);
  ASSERT_FLOAT_EQ(data.b_w_RS_S[0].x(), 12);
  ASSERT_FLOAT_EQ(data.b_w_RS_S[0].y(), 13);
  ASSERT_FLOAT_EQ(data.b_w_RS_S[0].z(), 14);
  ASSERT_FLOAT_EQ(data.b_a_RS_S[0].x(), 15);
  ASSERT_FLOAT_EQ(data.b_a_RS_S[0].y(), 16);
  ASSERT_FLOAT_EQ(data.b_a_RS_S[0].z(), 17);
}

TEST(Euroc, load_sequence) {
  EurocTestData test_data;
  EurocData data{test_data.data_dir};
}

TEST(Euroc, load_calib_target) {
  EurocTestData test_data;
  EurocTarget data{test_data.target_config_path};

  ASSERT_EQ(data.type, "aprilgrid");
  ASSERT_EQ(data.tag_rows, 6);
  ASSERT_EQ(data.tag_cols, 6);
  ASSERT_FLOAT_EQ(data.tag_size, 0.088);
  ASSERT_FLOAT_EQ(data.tag_spacing, 0.3);
}

TEST(Euroc, load_calib) {
  EurocTestData test_data;
  EurocCalib data{test_data.data_dir};
}

} // namespace cartesian
