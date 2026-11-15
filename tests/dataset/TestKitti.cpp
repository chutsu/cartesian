#include <gtest/gtest.h>

#include "core/Core.hpp"
#include "dataset/Kitti.hpp"

namespace cartesian {

struct KittiTestData {
  const fs::path kitti_data_dir = "/tmp/kitti";
  const fs::path kitti_camera_dir = kitti_data_dir / "image_00";
  const fs::path kitti_oxts_dir = kitti_data_dir / "oxts";
  const fs::path kitti_velodyne_dir = kitti_data_dir / "velodyne_points";

  int setup_kitti_camera_test_data(const fs::path &camera_dir) {
    // Create data directory
    const fs::path data_dir = camera_dir / "data";
    fs::create_directories(data_dir);

    // Image file
    {
      const fs::path image_path = data_dir / "0000000000.png";
      FILE *fp = fopen(image_path.string().c_str(), "w");
      fclose(fp);
    }

    // Timestamps file
    {
      const fs::path timestamps_path = camera_dir / "timestamps.txt";
      FILE *fp = fopen(timestamps_path.string().c_str(), "w");
      fprintf(fp, "2011-09-26 13:02:25.967790592\n");
      fclose(fp);
    }

    return 0;
  }

  int setup_kitti_oxts_test_data(const fs::path &oxts_dir) {
    // Create data directory
    const fs::path data_dir = oxts_dir / "data";
    fs::create_directories(data_dir);

    // Oxts entry
    {
      const fs::path &oxts_path = data_dir / "0000000000.txt";
      FILE *fp = fopen(oxts_path.string().c_str(), "w");
      for (int i = 0; i < 25; ++i) {
        fprintf(fp, "%f ", (double) i);
      }
      fprintf(fp, "25 ");
      fprintf(fp, "26 ");
      fprintf(fp, "27 ");
      fprintf(fp, "28 ");
      fprintf(fp, "29\n");
      fclose(fp);
    }

    // Timestamps
    {
      const fs::path timestamps_path = oxts_dir / "timestamps.txt";
      FILE *fp = fopen(timestamps_path.string().c_str(), "w");
      fprintf(fp, "2011-09-26 13:02:25.964389445\n");
      fclose(fp);
    }

    return 0;
  }

  int setup_kitti_velodyne_test_data(const fs::path &velodyne_dir) {
    // Create data directory
    const fs::path data_dir = velodyne_dir / "data";
    fs::create_directories(data_dir);

    // Velodyne entry
    {
      const fs::path velodyne_path = data_dir / "0000000000.bin";
      FILE *fp = fopen(velodyne_path.string().c_str(), "w");
      fprintf(fp, "\n");
      fclose(fp);
    }

    // Timestamps
    {
      const fs::path timestamps_path = velodyne_dir / "timestamps.txt";
      FILE *fp = fopen(timestamps_path.string().c_str(), "w");
      fprintf(fp, "2011-09-26 13:02:25.951199337\n");
      fclose(fp);
    }

    // Timestamps - start
    {
      const fs::path timestamps_path = velodyne_dir / "timestamps_start.txt";
      FILE *fp = fopen(timestamps_path.string().c_str(), "w");
      fprintf(fp, "2011-09-26 13:02:25.899635528\n");
      fclose(fp);
    }

    // Timestamps - end
    {
      const fs::path timestamps_path = velodyne_dir / "timestamps_end.txt";
      FILE *fp = fopen(timestamps_path.string().c_str(), "w");
      fprintf(fp, "2011-09-26 13:02:26.002763147\n");
      fclose(fp);
    }

    return 0;
  }

  int setup_kitti_calib_test_data(const fs::path &data_dir) {
    // Create calib_cam_to_cam.txt
    // clang-format off
    const fs::path &cam_to_cam_path = data_dir / "calib_cam_to_cam.txt";
    FILE *f = fopen(cam_to_cam_path.string().c_str(), "w");
    fprintf(f, "calib_time: 09-Jan-2012 13:57:47\n");
    fprintf(f, "corner_dist: 1.0\n");
    fprintf(f, "S_00: 1.0 2.0\n");
    fprintf(f, "K_00: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "D_00: 1.0 2.0 3.0 4.0 5.0\n");
    fprintf(f, "R_00: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "T_00: 1.0 2.0 3.0\n");
    fprintf(f, "S_rect_00: 1.0 2.0\n");
    fprintf(f, "R_rect_00: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "P_rect_00: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0\n");
    fprintf(f, "S_01: 1.0 2.0\n");
    fprintf(f, "K_01: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "D_01: 1.0 2.0 3.0 4.0 5.0\n");
    fprintf(f, "R_01: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "T_01: 1.0 2.0 3.0\n");
    fprintf(f, "S_rect_01: 1.0 2.0\n");
    fprintf(f, "R_rect_01: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "P_rect_01: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0\n");
    fprintf(f, "S_02: 1.0 2.0\n");
    fprintf(f, "K_02: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "D_02: 1.0 2.0 3.0 4.0 5.0\n");
    fprintf(f, "R_02: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "T_02: 1.0 2.0 3.0\n");
    fprintf(f, "S_rect_02: 1.0 2.0\n");
    fprintf(f, "R_rect_02: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "P_rect_02: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0\n");
    fprintf(f, "S_03: 1.0 2.0\n");
    fprintf(f, "K_03: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "D_03: 1.0 2.0 3.0 4.0 5.0\n");
    fprintf(f, "R_03: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "T_03: 1.0 2.0 3.0\n");
    fprintf(f, "S_rect_03: 1.0 2.0\n");
    fprintf(f, "R_rect_03: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "P_rect_03: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0\n");
    fclose(f);
    // clang-format on

    // Create calib_imu_to_velo.txt
    const fs::path &imu_to_velo_path = data_dir / "calib_imu_to_velo.txt";
    f = fopen(imu_to_velo_path.string().c_str(), "w");
    fprintf(f, "calib_time: 25-May-2012 16:47:16\n");
    fprintf(f, "R: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "T: 1.0 2.0 3.0\n");
    fclose(f);

    // Create calib_velo_to_cam.txt
    const fs::path &velo_to_cam_path = data_dir / "calib_velo_to_cam.txt";
    f = fopen(velo_to_cam_path.string().c_str(), "w");
    fprintf(f, "calib_time: 15-Mar-2012 11:37:16\n");
    fprintf(f, "R: 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0\n");
    fprintf(f, "T: 1.0 2.0 3.0\n");
    fprintf(f, "delta_f: 1.0 2.0\n");
    fprintf(f, "delta_c: 1.0 2.0\n");
    fclose(f);

    return 0;
  }

  KittiTestData() {
    const auto cam0_dir = kitti_data_dir / "image_00";
    const auto cam1_dir = kitti_data_dir / "image_01";
    const auto cam2_dir = kitti_data_dir / "image_02";
    const auto cam3_dir = kitti_data_dir / "image_03";
    const auto oxts_dir = kitti_data_dir / "oxts";
    const auto velodyne_dir = kitti_data_dir / "velodyne_points";
    fs::create_directories(kitti_data_dir);

    setup_kitti_camera_test_data(cam0_dir);
    setup_kitti_camera_test_data(cam1_dir);
    setup_kitti_camera_test_data(cam2_dir);
    setup_kitti_camera_test_data(cam3_dir);
    setup_kitti_oxts_test_data(oxts_dir);
    setup_kitti_velodyne_test_data(velodyne_dir);
    setup_kitti_calib_test_data(kitti_data_dir);
  }

  ~KittiTestData() {
    // Remove test data
    fs::remove_all(kitti_data_dir);
  }
};

TEST(Kitti, load_camera) {
  KittiTestData test_data;
  KittiCamera data{test_data.kitti_camera_dir};
}

TEST(Kitti, load_oxts) {
  KittiTestData test_data;

  KittiOxts data{test_data.kitti_oxts_dir};
  ASSERT_FLOAT_EQ(data.lat[0], 0.0);
  ASSERT_FLOAT_EQ(data.lon[0], 1.0);
  ASSERT_FLOAT_EQ(data.alt[0], 2.0);
  ASSERT_FLOAT_EQ(data.roll[0], 3.0);
  ASSERT_FLOAT_EQ(data.pitch[0], 4.0);
  ASSERT_FLOAT_EQ(data.yaw[0], 5.0);
  ASSERT_FLOAT_EQ(data.vn[0], 6.0);
  ASSERT_FLOAT_EQ(data.ve[0], 7.0);
  ASSERT_FLOAT_EQ(data.vf[0], 8.0);
  ASSERT_FLOAT_EQ(data.vl[0], 9.0);
  ASSERT_FLOAT_EQ(data.vu[0], 10.0);
  ASSERT_FLOAT_EQ(data.ax[0], 11.0);
  ASSERT_FLOAT_EQ(data.ay[0], 12.0);
  ASSERT_FLOAT_EQ(data.az[0], 13.0);
  ASSERT_FLOAT_EQ(data.af[0], 14.0);
  ASSERT_FLOAT_EQ(data.al[0], 15.0);
  ASSERT_FLOAT_EQ(data.au[0], 16.0);
  ASSERT_FLOAT_EQ(data.wx[0], 17.0);
  ASSERT_FLOAT_EQ(data.wy[0], 18.0);
  ASSERT_FLOAT_EQ(data.wz[0], 19.0);
  ASSERT_FLOAT_EQ(data.wf[0], 20.0);
  ASSERT_FLOAT_EQ(data.wl[0], 21.0);
  ASSERT_FLOAT_EQ(data.wu[0], 22.0);
  ASSERT_FLOAT_EQ(data.pos_accuracy[0], 23.0);
  ASSERT_FLOAT_EQ(data.vel_accuracy[0], 24.0);
  ASSERT_EQ(data.navstat[0], 25);
  ASSERT_EQ(data.numsats[0], 26);
  ASSERT_EQ(data.posmode[0], 27);
  ASSERT_EQ(data.velmode[0], 28);
  ASSERT_EQ(data.orimode[0], 29);
}

TEST(Kitti, load_velodyne) {
  KittiTestData test_data;
  KittiVelodyne data{test_data.kitti_velodyne_dir};
}

TEST(Kitti, load_calib) {
  KittiTestData test_data;
  KittiCalib data{test_data.kitti_data_dir};
}

TEST(Kitti, load_raw) {
  KittiTestData test_data;
  KittiRaw data{test_data.kitti_data_dir, ""};
}

} // namespace cartesian
