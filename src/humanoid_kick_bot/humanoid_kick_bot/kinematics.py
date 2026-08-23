"""
kinematics.py
-------------
2-DOF planar leg Forward and Inverse Kinematics.

Frame convention (per-leg, sagittal / x-y plane):
    - Origin (0, 0) is the hip joint (top of thigh, "Thigh" servo axis).
    - theta1 (hip / thigh angle) is measured from the +x axis (forward),
      positive = leg swinging forward, sweeping toward -y (down) as it
      approaches the straight-down "stance" pose.
    - theta2 (knee / shin angle) is measured RELATIVE to the thigh link
      (standard 2-link planar convention from the assignment brief).
    - L1 = thigh length (Thigh servo -> Shin servo).
    - L2 = shin length  (Shin servo -> Foot / ankle servo).

FK:
    x = L1*cos(theta1) + L2*cos(theta1 + theta2)
    y = L1*sin(theta1) + L2*sin(theta1 + theta2)

IK (geometric / law of cosines):
    r^2 = x^2 + y^2
    cos(theta2) = (r^2 - L1^2 - L2^2) / (2*L1*L2)
    theta2 = +/- acos(cos(theta2))          -> two elbow/knee configurations
    theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))

This module is pure math (no ROS dependency) so it can be unit tested and
reused by both the planner node and, if desired, the ESP32 firmware logic.
"""

import math
from dataclasses import dataclass


@dataclass
class LegGeometry:
    L1: float  # thigh length (m)
    L2: float  # shin length (m)


class KinematicsError(ValueError):
    """Raised when a requested foot position is unreachable."""


def forward_kinematics(theta1: float, theta2: float, geom: LegGeometry):
    """Given hip angle theta1 and knee angle theta2 (radians), return (x, y)
    of the foot/ankle point in the hip-centered leg frame."""
    x = geom.L1 * math.cos(theta1) + geom.L2 * math.cos(theta1 + theta2)
    y = geom.L1 * math.sin(theta1) + geom.L2 * math.sin(theta1 + theta2)
    return x, y


def inverse_kinematics(x: float, y: float, geom: LegGeometry, knee_forward: bool = True):
    """Geometric IK via the law of cosines.

    Args:
        x, y: desired foot position in the hip-centered leg frame (m).
        geom: LegGeometry(L1, L2).
        knee_forward: selects which of the two elbow/knee solutions to use.
            True  -> theta2 >= 0 (knee bends the "kicking" way)
            False -> theta2 <= 0 (knee bends the opposite way)

    Returns:
        (theta1, theta2) in radians.

    Raises:
        KinematicsError if the target is outside the leg's reachable
        workspace (too far, or inside the "dead zone" closer than |L1-L2|).
    """
    r2 = x * x + y * y
    r = math.sqrt(r2)
    reach_max = geom.L1 + geom.L2
    reach_min = abs(geom.L1 - geom.L2)

    if r > reach_max + 1e-9 or r < reach_min - 1e-9:
        raise KinematicsError(
            f"Target ({x:.4f}, {y:.4f}) with r={r:.4f} is outside reachable "
            f"workspace [{reach_min:.4f}, {reach_max:.4f}]"
        )

    cos_theta2 = (r2 - geom.L1 ** 2 - geom.L2 ** 2) / (2 * geom.L1 * geom.L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # clamp for numerical safety

    theta2_mag = math.acos(cos_theta2)
    theta2 = theta2_mag if knee_forward else -theta2_mag

    k1 = geom.L1 + geom.L2 * math.cos(theta2)
    k2 = geom.L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    return theta1, theta2


def verify_ik(theta1: float, theta2: float, x_target: float, y_target: float,
              geom: LegGeometry, tol: float = 1e-3) -> bool:
    """Re-run FK on an IK solution and confirm it reproduces the target.
    Used as a runtime safety check before commanding servos."""
    x_fk, y_fk = forward_kinematics(theta1, theta2, geom)
    err = math.hypot(x_fk - x_target, y_fk - y_target)
    return err <= tol


def clamp_angle(angle_rad: float, lo_rad: float, hi_rad: float) -> float:
    return max(lo_rad, min(hi_rad, angle_rad))
