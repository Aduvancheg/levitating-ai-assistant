"""
tof_vl53l5cx.py — VL53L5CX 8×8 ToF Matrix Sensor Driver

Parses 64-zone distance data, identifies the sphere position (minimum
trough), applies base-offset translation, and performs Gimbal Rotation
Matrix compensation for macro-levitation.

Conforms to:
  - architecture.md §5 (Rotation Matrix for Gimbal, OFFSET_X_MM = 15.0)
  - quality.md §4 (boundary testing: 0 mm rejection)
  - backlog LS-6
"""

from __future__ import annotations

import math
from typing import List, Tuple, Dict, Any, Optional

# Minimum valid ToF reading in mm.  Values at or below this threshold
# are treated as sensor errors (out-of-range or blocked FoV zone).
MIN_VALID_DISTANCE_MM: float = 5.0


class RotationMatrix:
    """
    3D rotation matrix for Gimbal compensation.

    When the Base physically tilts (Pitch/Roll servos), the ToF sensor's
    coordinate frame rotates relative to the world frame.  This class
    computes the inverse rotation to transform ToF readings back into
    absolute Base coordinates.

    Architecture §5:  "Code MUST use Rotation Matrices (or quaternions)
    for translating ToF coordinates depending on the current servo tilt.
    Simple scalar subtraction will cause a target miss."
    """

    def __init__(self) -> None:
        # Identity matrix (no rotation)
        self._matrix: List[List[float]] = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    def update_from_euler(self, pitch_deg: float, roll_deg: float) -> None:
        """
        Recompute the rotation matrix from current Gimbal servo angles.

        Uses ZYX (Tait-Bryan) convention:
          R = Ry(pitch) · Rx(roll)

        Args:
            pitch_deg: Gimbal pitch angle in degrees (Y-axis rotation).
            roll_deg:  Gimbal roll angle in degrees (X-axis rotation).
        """
        pitch_rad = math.radians(pitch_deg)
        roll_rad = math.radians(roll_deg)

        cp = math.cos(pitch_rad)
        sp = math.sin(pitch_rad)
        cr = math.cos(roll_rad)
        sr = math.sin(roll_rad)

        # R = Ry(pitch) · Rx(roll)
        self._matrix = [
            [ cp,       sp * sr,      sp * cr],
            [ 0.0,      cr,          -sr     ],
            [-sp,       cp * sr,      cp * cr],
        ]

    def transform(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Apply the rotation matrix to a 3D point.

        Returns:
            Rotated (x', y', z') in the world / Base frame.
        """
        m = self._matrix
        rx = m[0][0] * x + m[0][1] * y + m[0][2] * z
        ry = m[1][0] * x + m[1][1] * y + m[1][2] * z
        rz = m[2][0] * x + m[2][1] * y + m[2][2] * z
        return rx, ry, rz


class ToFMatrixDriver:
    """
    Driver for VL53L5CX 8×8 optical ToF matrix sensor.

    Pipeline:
      1. Filter invalid zones (0 mm, below threshold).
      2. Find the trough (minimum distance = sphere position).
      3. Convert grid indices → mm coordinates.
      4. Apply sensor offset (OFFSET_X_MM).
      5. Apply Gimbal Rotation Matrix to get absolute Base coordinates.
    """

    OFFSET_X_MM: float = 15.0

    def __init__(self, sensor_dev: Optional[object] = None) -> None:
        self.sensor_dev = sensor_dev
        self.rotation = RotationMatrix()

    # -----------------------------------------------------------------
    # Gimbal angle update
    # -----------------------------------------------------------------

    def update_gimbal_angles(self, pitch_deg: float, roll_deg: float) -> None:
        """
        Call this each tick with the current Gimbal servo angles so that
        subsequent coordinate transformations are correct.
        """
        self.rotation.update_from_euler(pitch_deg, roll_deg)

    # -----------------------------------------------------------------
    # Matrix analysis
    # -----------------------------------------------------------------

    def find_minimum_trough(
        self, matrix_8x8: List[List[float]]
    ) -> Tuple[int, int, float]:
        """
        Find (row, col) of the minimum valid distance in an 8×8 matrix.

        Zones with distance ≤ MIN_VALID_DISTANCE_MM are excluded as
        sensor errors (quality.md §4: "ToF outputs 0 mm → reject").

        Returns:
            (row, col, min_distance_mm).

        Raises:
            ValueError: If matrix is not 8×8 or no valid zones found.
        """
        if (
            not matrix_8x8
            or len(matrix_8x8) != 8
            or any(len(row) != 8 for row in matrix_8x8)
        ):
            raise ValueError("Input matrix must be exactly 8×8")

        min_val: float = float("inf")
        min_pos: Tuple[int, int] = (0, 0)

        for r in range(8):
            for c in range(8):
                val = matrix_8x8[r][c]

                # Filter invalid readings
                if val <= MIN_VALID_DISTANCE_MM:
                    continue

                # Filter non-finite values
                if math.isnan(val) or math.isinf(val):
                    continue

                if val < min_val:
                    min_val = val
                    min_pos = (r, c)

        if min_val == float("inf"):
            raise ValueError(
                "No valid ToF zones found (all readings ≤ "
                f"{MIN_VALID_DISTANCE_MM} mm or non-finite)"
            )

        return min_pos[0], min_pos[1], float(min_val)

    # -----------------------------------------------------------------
    # Coordinate translation (with Rotation Matrix)
    # -----------------------------------------------------------------

    def translate_coordinates(
        self,
        raw_x_mm: float,
        raw_y_mm: float,
        raw_z_mm: float,
    ) -> Tuple[float, float, float]:
        """
        Transform raw ToF coordinates to absolute Base coordinates:
          1. Apply OFFSET_X_MM.
          2. Apply Gimbal Rotation Matrix.

        Without the rotation step, macro-levitation (tilted base) would
        produce systematic targeting errors.
        """
        # Step 1: Offset correction (sensor is shifted from coil center)
        offset_x = float(raw_x_mm) - self.OFFSET_X_MM
        offset_y = float(raw_y_mm)
        offset_z = float(raw_z_mm)

        # Step 2: Gimbal rotation compensation
        base_x, base_y, base_z = self.rotation.transform(
            offset_x, offset_y, offset_z
        )

        return base_x, base_y, base_z

    # -----------------------------------------------------------------
    # Full frame processing pipeline
    # -----------------------------------------------------------------

    def process_matrix_frame(
        self,
        matrix_8x8: List[List[float]],
        fov_mm_per_zone: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Process a full 8×8 ToF frame end-to-end.

        Args:
            matrix_8x8:      8×8 distance matrix in mm.
            fov_mm_per_zone: Physical width of each zone in mm.

        Returns:
            Dict with keys:
              - "min_grid":  (row, col) of the detected sphere.
              - "raw_xyz":   Raw sensor XYZ before rotation.
              - "base_xyz":  Absolute Base XYZ after rotation + offset.
        """
        min_row, min_col, min_z = self.find_minimum_trough(matrix_8x8)

        # Convert grid indices to mm relative to matrix centre (3.5, 3.5)
        raw_x = (min_col - 3.5) * fov_mm_per_zone + self.OFFSET_X_MM
        raw_y = (min_row - 3.5) * fov_mm_per_zone

        abs_x, abs_y, abs_z = self.translate_coordinates(raw_x, raw_y, min_z)

        return {
            "min_grid": (min_row, min_col),
            "raw_xyz": (raw_x, raw_y, min_z),
            "base_xyz": (abs_x, abs_y, abs_z),
        }
