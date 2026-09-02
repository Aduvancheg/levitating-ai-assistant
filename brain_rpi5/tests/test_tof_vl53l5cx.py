"""
test_tof_vl53l5cx.py — Unit tests for ToF matrix sensor driver.

Covers:
  - Minimum trough detection in 8×8 matrix
  - Coordinate translation with OFFSET_X_MM
  - Rotation Matrix (Gimbal compensation)
  - Zero-mm filtering (quality.md §4)
  - Full frame processing pipeline
"""

import math
import pytest
from brain_rpi5.src.tof_vl53l5cx import ToFMatrixDriver, RotationMatrix, MIN_VALID_DISTANCE_MM


class TestFindMinimumTrough:

    def test_detects_minimum_at_known_position(self):
        """backlog LS-6: mock 8×8 matrix with trough at (2, 3)."""
        driver = ToFMatrixDriver()
        matrix = [[100.0] * 8 for _ in range(8)]
        matrix[2][3] = 25.5

        r, c, val = driver.find_minimum_trough(matrix)
        assert r == 2
        assert c == 3
        assert abs(val - 25.5) < 1e-4

    def test_filters_zero_mm_readings(self):
        """quality.md §4: ToF outputs 0 mm → must be rejected."""
        driver = ToFMatrixDriver()
        matrix = [[100.0] * 8 for _ in range(8)]
        matrix[0][0] = 0.0   # Invalid — should be filtered
        matrix[5][5] = 30.0  # Valid minimum

        r, c, val = driver.find_minimum_trough(matrix)
        assert r == 5
        assert c == 5
        assert abs(val - 30.0) < 1e-4

    def test_filters_below_threshold_readings(self):
        driver = ToFMatrixDriver()
        matrix = [[100.0] * 8 for _ in range(8)]
        matrix[0][0] = 3.0   # Below MIN_VALID_DISTANCE_MM (5.0)
        matrix[4][4] = 50.0  # Valid minimum

        r, c, val = driver.find_minimum_trough(matrix)
        assert r == 4
        assert c == 4

    def test_all_invalid_raises_error(self):
        driver = ToFMatrixDriver()
        matrix = [[0.0] * 8 for _ in range(8)]  # All zeros

        with pytest.raises(ValueError, match="No valid ToF zones"):
            driver.find_minimum_trough(matrix)

    def test_nan_values_filtered(self):
        driver = ToFMatrixDriver()
        matrix = [[float("nan")] * 8 for _ in range(8)]
        matrix[7][7] = 60.0  # Only valid value

        r, c, val = driver.find_minimum_trough(matrix)
        assert r == 7
        assert c == 7

    def test_wrong_matrix_size_raises(self):
        driver = ToFMatrixDriver()
        with pytest.raises(ValueError, match="8×8"):
            driver.find_minimum_trough([[1, 2], [3, 4]])


class TestCoordinateTranslation:

    def test_offset_applied_to_x(self):
        """backlog LS-6: input (20, 5, 40) → base_x = 20 - 45 = -25."""
        driver = ToFMatrixDriver()
        x, y, z = driver.translate_coordinates(20.0, 5.0, 40.0)
        assert abs(x - (-25.0)) < 1e-4
        assert abs(y - 5.0) < 1e-4
        assert abs(z - 40.0) < 1e-4

    def test_negative_result_allowed(self):
        """X can go negative if sphere is left of center."""
        driver = ToFMatrixDriver()
        x, y, z = driver.translate_coordinates(10.0, 0.0, 30.0)
        assert abs(x - (-35.0)) < 1e-4


class TestRotationMatrix:

    def test_identity_no_rotation(self):
        rot = RotationMatrix()
        x, y, z = rot.transform(10.0, 20.0, 30.0)
        assert abs(x - 10.0) < 1e-4
        assert abs(y - 20.0) < 1e-4
        assert abs(z - 30.0) < 1e-4

    def test_pitch_90_rotates_x_to_z(self):
        """90° pitch (Y-axis rotation): X→−Z, Z→X."""
        rot = RotationMatrix()
        rot.update_from_euler(pitch_deg=90.0, roll_deg=0.0)
        x, y, z = rot.transform(10.0, 0.0, 0.0)
        # After Ry(90°): (10,0,0) → (0, 0, -10)
        assert abs(x - 0.0) < 1e-4
        assert abs(z - (-10.0)) < 1e-4

    def test_roll_90_rotates_y_to_negz(self):
        """90° roll (X-axis rotation) via R=Ry·Rx with pitch=0."""
        rot = RotationMatrix()
        rot.update_from_euler(pitch_deg=0.0, roll_deg=90.0)
        x, y, z = rot.transform(0.0, 10.0, 0.0)
        # R = Ry(0)·Rx(90): row1=[0,cr,-sr]=[0,0,-1], row2=[0,sr,cr]=[0,1,0]
        # (0,10,0) → x=0, y=0*10=0, z=1*10=10
        assert abs(x - 0.0) < 1e-4
        assert abs(y - 0.0) < 1e-4
        assert abs(z - 10.0) < 1e-4

    def test_small_angle_nearly_identity(self):
        """1° tilt should produce near-identity transform."""
        rot = RotationMatrix()
        rot.update_from_euler(pitch_deg=1.0, roll_deg=1.0)
        x, y, z = rot.transform(100.0, 0.0, 30.0)
        # At 1° the coordinates should barely change
        assert abs(x - 100.0) < 2.0
        assert abs(z - 30.0) < 2.0

    def test_translate_with_gimbal_rotation(self):
        """
        Integration test: offset + rotation matrix applied together.
        Architecture §5: simple scalar subtraction is WRONG for tilted base.
        """
        driver = ToFMatrixDriver()
        driver.update_gimbal_angles(pitch_deg=10.0, roll_deg=5.0)

        # Raw sensor reading: sphere at (20, 5, 40)
        bx, by, bz = driver.translate_coordinates(20.0, 5.0, 40.0)

        # With rotation, the result should differ from simple subtraction
        simple_x = 20.0 - 45.0  # = -25.0
        # Rotated result should NOT equal the simple subtraction
        # (unless the angle is exactly 0)
        assert not (abs(bx - simple_x) < 1e-4 and abs(bz - 40.0) < 1e-4)


class TestProcessMatrixFrame:

    def test_full_pipeline(self):
        driver = ToFMatrixDriver()
        matrix = [[100.0] * 8 for _ in range(8)]
        matrix[2][3] = 25.5  # Trough at (2, 3)

        result = driver.process_matrix_frame(matrix, fov_mm_per_zone=5.0)

        assert result["min_grid"] == (2, 3)
        assert len(result["raw_xyz"]) == 3
        assert len(result["base_xyz"]) == 3
        # Z should be the trough value
        assert abs(result["base_xyz"][2] - 25.5) < 1.0


class TestToFBoundaryZoneFusion:
    """Tests for Complementary Filter boundary zones at OFFSET=45.0mm."""

    def test_tof_fusion_at_49mm_boundary(self):
        """At 49mm (HALL_MAX_Z_MM), fusion should be 100% Hall."""
        from brain_rpi5.src.core_loop import ComplementaryFilter
        filt = ComplementaryFilter(steepness=0.5)
        hall_w, tof_w = filt.compute_weights(49.0)
        assert abs(hall_w - 1.0) < 1e-4
        assert abs(tof_w - 0.0) < 1e-4
        # Verify fuse with both inputs near 49mm so approx_z is in Hall zone
        result = filt.fuse(hall_z_mm=48.0, tof_z_mm=50.0)
        # approx_z = 49.0 → pure Hall → result ≈ 48.0
        assert abs(result - 48.0) < 1e-4

    def test_tof_fusion_at_55mm_midpoint(self):
        """At 55mm (midpoint), fusion should be ~50/50 blend."""
        from brain_rpi5.src.core_loop import ComplementaryFilter
        filt = ComplementaryFilter(steepness=0.5)
        hall_w, tof_w = filt.compute_weights(55.0)
        assert abs(hall_w - 0.5) < 0.01
        assert abs(tof_w - 0.5) < 0.01
        # Verify fuse returns average-ish
        result = filt.fuse(hall_z_mm=50.0, tof_z_mm=60.0)
        assert 50.0 <= result <= 60.0

    def test_tof_fusion_at_61mm_boundary(self):
        """At 61mm (TOF_MIN_Z_MM), fusion should be 100% ToF."""
        from brain_rpi5.src.core_loop import ComplementaryFilter
        filt = ComplementaryFilter(steepness=0.5)
        hall_w, tof_w = filt.compute_weights(61.0)
        assert abs(hall_w - 0.0) < 1e-4
        assert abs(tof_w - 1.0) < 1e-4
        # Verify fuse with both inputs near 61mm so approx_z is in ToF zone
        result = filt.fuse(hall_z_mm=60.0, tof_z_mm=62.0)
        # approx_z = 61.0 → pure ToF → result ≈ 62.0
        assert abs(result - 62.0) < 1e-4
