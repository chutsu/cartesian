#include <gtest/gtest.h>

#include "camera/Pinhole.hpp"

namespace cartesian {

TEST(Pinhole, pinhole_focal) {
  const double focal = pinhole_focal(640, 90.0);
  ASSERT_TRUE(fltcmp(focal, 320) == 0);
}

TEST(Pinhole, pinhole_K) {
  const Vec2i cam_res = {640, 480};
  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const Mat3 K = pinhole_K(fx, fy, cx, cy);
  ASSERT_FLOAT_EQ(K(0, 0), fx);
  ASSERT_FLOAT_EQ(K(1, 1), fy);
  ASSERT_FLOAT_EQ(K(0, 2), cx);
  ASSERT_FLOAT_EQ(K(1, 2), cy);
  ASSERT_FLOAT_EQ(K(2, 2), 1.0);
}

TEST(Pinhole, pinhole_project) {
  const Vec2i cam_res = {640, 480};
  const double fx = pinhole_focal(cam_res[0], 90.0);
  const double fy = pinhole_focal(cam_res[0], 90.0);
  const double cx = cam_res[0] / 2.0;
  const double cy = cam_res[1] / 2.0;
  const Vec4 proj_params{fx, fy, cx, cy};

  const Vec3 p_C{0.0, 0.0, 1.0};
  Vec2 z_hat;
  int retval = pinhole_project(cam_res, proj_params, p_C, z_hat);
  ASSERT_EQ(retval, 0);
  ASSERT_FLOAT_EQ(z_hat(0), 320.0);
  ASSERT_FLOAT_EQ(z_hat(1), 240.0);
}

} // namespace cartesian
