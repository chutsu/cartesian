#include <gtest/gtest.h>

#include "calib/AprilGrid.hpp"
#include "calib/AprilGridConfig.hpp"

#define TEST_OUTPUT "/tmp/aprilgrid.csv"

namespace cartesian {

static AprilGrid setup_aprilgrid() {
  const timestamp_t ts = 0;
  const int camera_id = 0;

  AprilGridConfig config;
  config.target_id = 0;
  config.tag_rows = 6;
  config.tag_cols = 7;
  config.tag_size = 0.088;
  config.tag_spacing = 0.3;

  return AprilGrid{ts, camera_id, config};
}

TEST(AprilGrid, construct) {
  AprilGrid grid = setup_aprilgrid();

  ASSERT_EQ(grid.ts, 0);
  ASSERT_EQ(grid.camera_id, 0);
  ASSERT_EQ(grid.get_tag_rows(), 6);
  ASSERT_EQ(grid.get_tag_cols(), 7);
  ASSERT_EQ(grid.get_tag_size(), 0.088);
  ASSERT_EQ(grid.get_tag_spacing(), 0.3);
}

TEST(AprilGrid, addAndRemove) {
  AprilGrid grid = setup_aprilgrid();

  // Keypoints
  const Vec2 kp1_gnd{1.0, 2.0};
  const Vec2 kp2_gnd{3.0, 4.0};
  const Vec2 kp3_gnd{5.0, 6.0};
  const Vec2 kp4_gnd{7.0, 8.0};

  // Test add
  const int tag_id = 0;
  grid.add(tag_id, 0, kp1_gnd);
  grid.add(tag_id, 1, kp2_gnd);
  grid.add(tag_id, 2, kp3_gnd);
  grid.add(tag_id, 3, kp4_gnd);
  {
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    Vec2s keypoints;
    Vec3s object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);
    ASSERT_EQ(tag_ids.size(), 4);
    ASSERT_EQ(corner_indicies.size(), 4);
    ASSERT_EQ(keypoints.size(), 4);
    ASSERT_EQ(object_points.size(), 4);

    ASSERT_NEAR(0.0, (kp1_gnd - keypoints[0]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp2_gnd - keypoints[1]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp3_gnd - keypoints[2]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp4_gnd - keypoints[3]).norm(), 1e-8);
  }

  // Test remove
  grid.remove(tag_id, 0);
  grid.remove(tag_id, 2);
  {
    std::vector<int> tag_ids;
    std::vector<int> corner_indicies;
    Vec2s keypoints;
    Vec3s object_points;
    grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);
    ASSERT_EQ(tag_ids.size(), 2);
    ASSERT_EQ(corner_indicies.size(), 2);
    ASSERT_EQ(keypoints.size(), 2);
    ASSERT_EQ(object_points.size(), 2);

    ASSERT_NEAR(0.0, (kp2_gnd - keypoints[0]).norm(), 1e-8);
    ASSERT_NEAR(0.0, (kp4_gnd - keypoints[1]).norm(), 1e-8);
  }
}

TEST(AprilGrid, saveAndLoad) {
  AprilGrid grid = setup_aprilgrid();

  // Test save
  const int tag_id = 0;
  const Vec2 kp0{1.0, 2.0};
  const Vec2 kp1{3.0, 4.0};
  const Vec2 kp2{5.0, 6.0};
  const Vec2 kp3{7.0, 8.0};
  grid.add(tag_id, 0, kp0);
  grid.add(tag_id, 1, kp1);
  grid.add(tag_id, 2, kp2);
  grid.add(tag_id, 3, kp3);
  ASSERT_TRUE(grid.save(TEST_OUTPUT) == 0);

  // Test load
  auto grid2 = AprilGrid::load(TEST_OUTPUT);

  std::vector<int> tag_ids;
  std::vector<int> corner_indicies;
  Vec2s keypoints;
  Vec3s object_points;
  grid.get_measurements(tag_ids, corner_indicies, keypoints, object_points);

  ASSERT_EQ(grid.ts, grid2->ts);
  ASSERT_FLOAT_EQ(0.0, (keypoints[0] - kp0).norm());
  ASSERT_FLOAT_EQ(0.0, (keypoints[1] - kp1).norm());
  ASSERT_FLOAT_EQ(0.0, (keypoints[2] - kp2).norm());
  ASSERT_FLOAT_EQ(0.0, (keypoints[3] - kp3).norm());
}

} // namespace cartesian
