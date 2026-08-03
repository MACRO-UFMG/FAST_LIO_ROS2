"""Unit tests for optical-to-body IMU frame transformation."""

from __future__ import annotations

import numpy as np
import pytest

from imu_frame_adapter.transform import (
    ORIENTATION_UNAVAILABLE,
    R_BO,
    orientation_is_available,
    rotate_covariance,
    rotate_vector,
)


class TestAxisMapping:
    @pytest.mark.parametrize(
        "optical,expected",
        [
            ([1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ([0.0, 1.0, 0.0], [0.0, 0.0, -1.0]),
            ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        ],
    )
    def test_angular_velocity_axes(self, optical, expected):
        body = rotate_vector(optical)
        np.testing.assert_allclose(body, expected, rtol=0.0, atol=1e-12)

    @pytest.mark.parametrize(
        "optical,expected",
        [
            ([1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ([0.0, 1.0, 0.0], [0.0, 0.0, -1.0]),
            ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        ],
    )
    def test_linear_acceleration_axes(self, optical, expected):
        body = rotate_vector(optical)
        np.testing.assert_allclose(body, expected, rtol=0.0, atol=1e-12)


class TestCovarianceRotation:
    def test_covariance_rotation_formula(self):
        # Non-isotropic diagonal in optical frame; rotation swaps axes detectably.
        c_opt = np.diag([4.0, 9.0, 16.0])
        flat = rotate_covariance(c_opt.reshape(-1))
        c_body = np.asarray(flat).reshape(3, 3)
        expected = R_BO @ c_opt @ R_BO.T
        np.testing.assert_allclose(c_body, expected, rtol=0.0, atol=1e-12)

    def test_covariance_off_diagonal(self):
        c_opt = np.array(
            [
                [1.0, 0.2, 0.3],
                [0.2, 2.0, 0.4],
                [0.3, 0.4, 3.0],
            ]
        )
        c_body = np.asarray(rotate_covariance(c_opt.reshape(-1))).reshape(3, 3)
        expected = R_BO @ c_opt @ R_BO.T
        np.testing.assert_allclose(c_body, expected, rtol=0.0, atol=1e-12)


class TestTimestampPreservation:
    def test_stamp_preserved_verbatim(self):
        """Adapter must copy header.stamp exactly (no now())."""
        from builtin_interfaces.msg import Time

        src = Time(sec=123, nanosec=456789012)
        dst = Time(sec=src.sec, nanosec=src.nanosec)
        assert dst.sec == 123
        assert dst.nanosec == 456789012


class TestOrientationUnavailable:
    def test_orientation_unavailable_flag(self):
        cov = [-1.0] + [0.0] * 8
        assert not orientation_is_available(cov)

    def test_orientation_covariance_preserved(self):
        cov = [-1.0] + [0.1] * 8
        # After transform path, first element must remain -1.
        out = list(cov)
        out[0] = ORIENTATION_UNAVAILABLE
        assert out[0] == -1.0
