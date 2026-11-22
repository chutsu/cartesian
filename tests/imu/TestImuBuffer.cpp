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

TEST(ImuBuffer, trim_before) {
  ImuBuffer imu_buffer;
  for (int i = 0; i < 10; i += 2) {
    const timestamp_t ts = i;
    imu_buffer.add(ts, Vec3(i, i, i), Vec3(i, i, i));
  }

  // Keep the last measurement at or before ts, so extract can interpolate
  imu_buffer.trim_before(5);
  ASSERT_EQ(imu_buffer.size(), 3);
  ASSERT_EQ(imu_buffer.getTimestamp(0), 4);

  // Measurement exactly at ts is kept as the first
  imu_buffer.trim_before(6);
  ASSERT_EQ(imu_buffer.getTimestamp(0), 6);

  // Never empties the buffer
  imu_buffer.trim_before(100);
  ASSERT_EQ(imu_buffer.size(), 1);
  ASSERT_EQ(imu_buffer.getTimestamp(0), 8);

  // The window after the trim can still be interpolated at its start
  ImuBuffer buf;
  for (int i = 0; i < 10; i += 2) {
    buf.add(i, Vec3(i, i, i), Vec3(i, i, i));
  }
  buf.trim_before(5);
  const auto window = buf.extract(5, 8);
  ASSERT_EQ(window.getTimestamp(0), 5);
  ASSERT_NEAR(window.getAcc(0).x(), 5.0, 1e-9);
  ASSERT_EQ(window.getTimestamp(window.size() - 1), 8);
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
