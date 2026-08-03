#ifndef IMU_FRAME_ADAPTER__TRANSFORM_HPP_
#define IMU_FRAME_ADAPTER__TRANSFORM_HPP_

#include <array>
#include <cmath>
#include <cstddef>

namespace imu_frame_adapter
{

// RealSense optical -> Espeleo body (camera_imu_body_frame):
//   x_body =  z_optical
//   y_body = -x_optical
//   z_body = -y_optical
inline constexpr std::array<double, 9> kR_BO = {
  0.0, 0.0, 1.0,
  -1.0, 0.0, 0.0,
  0.0, -1.0, 0.0,
};

inline void rotate_vector(
  double ix, double iy, double iz,
  double & ox, double & oy, double & oz)
{
  ox = kR_BO[0] * ix + kR_BO[1] * iy + kR_BO[2] * iz;
  oy = kR_BO[3] * ix + kR_BO[4] * iy + kR_BO[5] * iz;
  oz = kR_BO[6] * ix + kR_BO[7] * iy + kR_BO[8] * iz;
}

inline void rotate_covariance(const std::array<double, 9> & in, std::array<double, 9> & out)
{
  // C_body = R * C_opt * R^T  (row-major 3x3)
  double R[3][3];
  double C[3][3];
  double RC[3][3];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R[r][c] = kR_BO[r * 3 + c];
      C[r][c] = in[r * 3 + c];
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      RC[r][c] = 0.0;
      for (int k = 0; k < 3; ++k) {
        RC[r][c] += R[r][k] * C[k][c];
      }
    }
  }
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[r * 3 + c] = 0.0;
      for (int k = 0; k < 3; ++k) {
        out[r * 3 + c] += RC[r][k] * R[c][k];  // multiply by R^T
      }
    }
  }
}

inline bool is_finite3(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

}  // namespace imu_frame_adapter

#endif  // IMU_FRAME_ADAPTER__TRANSFORM_HPP_
