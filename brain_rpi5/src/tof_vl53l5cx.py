"""
tof_vl53l5cx.py — VL53L5CX 8×8 ToF Matrix Sensor Driver

Parses 64-zone distance data, identifies the sphere position (minimum
trough), converts the zone to sensor-frame XYZ using the zone's angular
position, then transforms it into the Base frame (sensor offset + Gimbal
Rotation Matrix).

Frames:
  - Sensor frame: origin at the ToF optical centre, Z along its axis.
  - Base frame:   origin at the centre coil. The sensor sits at
                  (OFFSET_X_MM, OFFSET_Y_MM, 0) in this frame.
  p_base = R · (p_sensor + t_sensor)      (offset applied exactly once)

Conforms to:
  - architecture.md §5 (Rotation Matrix for Gimbal, OFFSET_X_MM = 45.0)
  - enclosure_architecture_v7.md (tof_on_diagonal=false, axial compact alignment)
  - quality.md §4 (boundary testing: 0 mm rejection)
  - backlog LS-6
"""

from __future__ import annotations

import math
from typing import List, Tuple, Dict, Any, Optional

# Minimum valid ToF reading in mm.  Values at or below this threshold
# are treated as sensor errors (out-of-range or blocked FoV zone).
MIN_VALID_DISTANCE_MM: float = 5.0

# VL53L5CX optics: 45° × 45° square field of view split into 8 × 8 zones.
FOV_DEG: float = 45.0
GRID_SIZE: int = 8
ZONE_ANGLE_DEG: float = FOV_DEG / GRID_SIZE  # ≈ 5.625° per zone


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
      3. Convert zone angle + distance → sensor-frame XYZ.
      4. Add the sensor position in the Base frame (OFFSET_X_MM, once).
      5. Apply Gimbal Rotation Matrix to get absolute Base coordinates.
    """

    # Position of the ToF optical centre in the Base frame
    # (enclosure_architecture_v7.md: ToF at X=+45.0 mm, Y=0.0 mm).
    OFFSET_X_MM: float = 45.0
    OFFSET_Y_MM: float = 0.0

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

    @staticmethod
    def zone_to_sensor_xyz(
        row: int, col: int, distance_mm: float
    ) -> Tuple[float, float, float]:
        """
        Convert a zone index and its distance to sensor-frame XYZ.

        Each zone covers ZONE_ANGLE_DEG; its centre is at
        θ = (index − 3.5) · ZONE_ANGLE_DEG off the optical axis, so the
        lateral offset grows with distance: x = z · tan(θx) (review S6).

        The reported zone distance is treated as the distance along the
        sensor axis (Z). If bench tests show the sensor reports radial
        distance instead, use z = d·cos(θ) here.
        """
        centre = (GRID_SIZE - 1) / 2.0
        theta_x = math.radians((col - centre) * ZONE_ANGLE_DEG)
        theta_y = math.radians((row - centre) * ZONE_ANGLE_DEG)
        z = float(distance_mm)
        return z * math.tan(theta_x), z * math.tan(theta_y), z

    def translate_coordinates(
        self,
        sensor_x_mm: float,
        sensor_y_mm: float,
        sensor_z_mm: float,
    ) -> Tuple[float, float, float]:
        """
        Transform sensor-frame coordinates to absolute Base coordinates:
          p_base = R · (p_sensor + t_sensor)
        where t_sensor = (OFFSET_X_MM, OFFSET_Y_MM, 0) is the sensor
        position in the Base frame. The offset is applied exactly once
        (review S5).

        Without the rotation step, macro-levitation (tilted base) would
        produce systematic targeting errors.
        """
        # Step 1: shift from the sensor origin to the Base (coil) origin
        bx = float(sensor_x_mm) + self.OFFSET_X_MM
        by = float(sensor_y_mm) + self.OFFSET_Y_MM
        bz = float(sensor_z_mm)

        # Step 2: Gimbal rotation compensation
        return self.rotation.transform(bx, by, bz)

    # -----------------------------------------------------------------
    # Full frame processing pipeline
    # -----------------------------------------------------------------

    def process_matrix_frame(
        self,
        matrix_8x8: List[List[float]],
    ) -> Dict[str, Any]:
        """
        Process a full 8×8 ToF frame end-to-end.

        Args:
            matrix_8x8: 8×8 distance matrix in mm.

        Returns:
            Dict with keys:
              - "min_grid":  (row, col) of the detected sphere.
              - "raw_xyz":   Sphere XYZ in the sensor frame.
              - "base_xyz":  Sphere XYZ in the Base frame (offset + rotation).
        """
        min_row, min_col, min_z = self.find_minimum_trough(matrix_8x8)

        raw_x, raw_y, raw_z = self.zone_to_sensor_xyz(min_row, min_col, min_z)
        abs_x, abs_y, abs_z = self.translate_coordinates(raw_x, raw_y, raw_z)

        return {
            "min_grid": (min_row, min_col),
            "raw_xyz": (raw_x, raw_y, raw_z),
            "base_xyz": (abs_x, abs_y, abs_z),
        }
