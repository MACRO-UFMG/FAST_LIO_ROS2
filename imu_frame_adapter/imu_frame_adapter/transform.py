"""Optical-to-body IMU frame transformation for RealSense D456 on Espeleo.

RealSense optical frame (camera_imu_optical_frame):
    X: right, Y: down, Z: forward

Body-aligned frame (camera_imu_body_frame):
    X: forward, Y: left, Z: up

Fixed rotation R_BO (optical -> body):
    x_body =  z_optical
    y_body = -x_optical
    z_body = -y_optical

    R_BO = [[ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]]
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

# Row-major 3x3 rotation: v_body = R_BO @ v_optical
R_BO = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)

ORIENTATION_UNAVAILABLE = -1.0


def rotate_vector(vec: Sequence[float]) -> Tuple[float, float, float]:
    """Rotate a 3-vector from optical to body frame."""
    out = R_BO @ np.asarray(vec, dtype=np.float64)
    return float(out[0]), float(out[1]), float(out[2])


def rotate_covariance(cov: Sequence[float]) -> list[float]:
    """Rotate a row-major 3x3 covariance: C_body = R @ C_opt @ R.T."""
    c = np.asarray(cov, dtype=np.float64).reshape(3, 3)
    out = R_BO @ c @ R_BO.T
    return out.reshape(-1).tolist()


def is_finite_vec3(x: float, y: float, z: float) -> bool:
    return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)


def orientation_is_available(orientation_covariance: Sequence[float]) -> bool:
    if len(orientation_covariance) == 0:
        return False
    return float(orientation_covariance[0]) != ORIENTATION_UNAVAILABLE
