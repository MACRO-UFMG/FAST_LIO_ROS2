#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "imu_frame_adapter/transform.hpp"

namespace imu_frame_adapter
{
namespace
{

TEST(AxisMapping, AngularVelocityOpticalX)
{
  double ox, oy, oz;
  rotate_vector(1.0, 0.0, 0.0, ox, oy, oz);
  EXPECT_NEAR(ox, 0.0, 1e-12);
  EXPECT_NEAR(oy, -1.0, 1e-12);
  EXPECT_NEAR(oz, 0.0, 1e-12);
}

TEST(AxisMapping, AngularVelocityOpticalY)
{
  double ox, oy, oz;
  rotate_vector(0.0, 1.0, 0.0, ox, oy, oz);
  EXPECT_NEAR(ox, 0.0, 1e-12);
  EXPECT_NEAR(oy, 0.0, 1e-12);
  EXPECT_NEAR(oz, -1.0, 1e-12);
}

TEST(AxisMapping, AngularVelocityOpticalZ)
{
  double ox, oy, oz;
  rotate_vector(0.0, 0.0, 1.0, ox, oy, oz);
  EXPECT_NEAR(ox, 1.0, 1e-12);
  EXPECT_NEAR(oy, 0.0, 1e-12);
  EXPECT_NEAR(oz, 0.0, 1e-12);
}

TEST(AxisMapping, LinearAccelerationAxes)
{
  double bx, by, bz;
  rotate_vector(1.0, 0.0, 0.0, bx, by, bz);
  EXPECT_NEAR(by, -1.0, 1e-12);
  rotate_vector(0.0, 1.0, 0.0, bx, by, bz);
  EXPECT_NEAR(bz, -1.0, 1e-12);
  rotate_vector(0.0, 0.0, 1.0, bx, by, bz);
  EXPECT_NEAR(bx, 1.0, 1e-12);
}

TEST(CovarianceRotation, NonIsotropicDiagonal)
{
  std::array<double, 9> c_opt = {4.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 16.0};
  std::array<double, 9> c_body{};
  rotate_covariance(c_opt, c_body);
  // R_BO maps optical Z->body X, optical X->body -Y, optical Y->body -Z
  EXPECT_NEAR(c_body[0], 16.0, 1e-9);  // body xx gets optical zz variance
  EXPECT_NEAR(c_body[4], 4.0, 1e-9);    // body yy gets optical xx variance
  EXPECT_NEAR(c_body[8], 9.0, 1e-9);    // body zz gets optical yy variance
}

}  // namespace
}  // namespace imu_frame_adapter

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
