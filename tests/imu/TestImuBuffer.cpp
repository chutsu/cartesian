#include <gtest/gtest.h>

#include "imu/ImuBuffer.hpp"

namespace cartesian {

TEST(ImuBuffer, trim) {
  ImuBuffer imu_buffer;
  for (int i = 0; i < 10; ++i) {
    const timestamp_t ts = i;
    const Vec3 acc(i, i, i);
    const Vec3 gyr(i, i, i);
    imu_buffer.add(ts, acc, gyr);
  }

  imu_buffer.trim(5);
  ASSERT_EQ(imu_buffer.getTimestamp(0), 5);
  ASSERT_EQ(imu_buffer.getTimestamp(1), 6);
  ASSERT_EQ(imu_buffer.getTimestamp(2), 7);
  ASSERT_EQ(imu_buffer.getTimestamp(3), 8);
  ASSERT_EQ(imu_buffer.getTimestamp(4), 9);
}

TEST(ImuBuffer, extract) {
  ImuBuffer imu_buffer;
  for (int i = 0; i < 10; i += 2) {
    const timestamp_t ts = i;
    const Vec3 acc(i, i, i);
    const Vec3 gyr(i, i, i);
    imu_buffer.add(ts, acc, gyr);
  }

  auto imu_buffer2 = imu_buffer.extract(1, 7);
  ASSERT_EQ(imu_buffer2.getTimestamp(0), 1);
  ASSERT_EQ(imu_buffer2.getTimestamp(4), 7);
}

} // namespace cartesian
