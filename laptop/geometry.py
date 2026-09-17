"""Vergence geometry — the laptop half of the scanner.

The firmware is deliberately a dumb sensor pipe: it aims two servos, reads two
ToF sensors, and streams the raw numbers. Everything here inverts that.

Frame convention (shared with the firmware):

    +Y is forward, +X is to the right, origin is the midpoint of the baseline.
    Bearing theta is measured from +Y, positive toward +X.
    Left sensor sits at (-B/2, 0), right sensor at (+B/2, 0).
    A servo command of 90 aims its sensor straight down +Y.

        L                 R
        o-------+-------o        <- baseline, length B
       -B/2     0     +B/2
                |
                | theta = 0
                v  +Y (forward)

Keep BASELINE_M and VERGE_RANGE_M in step with the firmware. If you change the
rig, change both — nothing here can detect a mismatch for you.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# --- must match lidar_2_eye_scaer.ino -------------------------------------
BASELINE_M = 0.10
VERGE_RANGE_M = 1.5
SERVO_L_OFFSET_DEG = 0
SERVO_R_OFFSET_DEG = 0
# --------------------------------------------------------------------------

# The VL53L0X reports a distance even when nothing is in range — the number is
# simply meaningless past roughly 2m (1.2m in its default profile, ~2m in long
# range mode). The firmware only filters RangeStatus 4, which does not catch
# every case, so anything beyond this is treated here as "no return". Without
# the gate a handful of absurd readings dominate the map and hide the scene.
MAX_RANGE_M = 2.0


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    @property
    def range_m(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing_deg(self) -> float:
        return math.degrees(math.atan2(self.x, self.y))


def verge_angles(theta_deg: float) -> tuple[int, int]:
    """Servo commands that verge both sensors on a point at `theta_deg`.

    This is a direct port of `vergeAngles()` in the firmware. It exists here so
    the round-trip can be tested without hardware in the loop — if this and the
    firmware ever disagree, every point in every sweep is quietly wrong.
    """
    theta = math.radians(theta_deg)
    xt = VERGE_RANGE_M * math.sin(theta)
    yt = VERGE_RANGE_M * math.cos(theta)

    sl = math.degrees(math.atan2(xt + BASELINE_M / 2.0, yt))
    sr = math.degrees(math.atan2(xt - BASELINE_M / 2.0, yt))

    cmd_l = int(round(90.0 + sl)) + SERVO_L_OFFSET_DEG
    cmd_r = int(round(90.0 + sr)) + SERVO_R_OFFSET_DEG
    return max(0, min(180, cmd_l)), max(0, min(180, cmd_r))


def aim_deg(servo_cmd: int, offset_deg: int) -> float:
    """Where a sensor is actually pointing, in degrees from forward."""
    return float(servo_cmd - offset_deg - 90)


def project(sensor_x: float, aim_degrees: float, dist_mm: int) -> Point | None:
    """Put one sensor's reading into the shared frame.

    Returns None for the firmware's out-of-range sentinel (-1). Those are not
    zeros and must never be averaged as if they were.
    """
    if dist_mm is None or dist_mm < 0:
        return None
    d = dist_mm / 1000.0
    if d > MAX_RANGE_M:
        return None  # past the sensor's useful range; the number is noise
    a = math.radians(aim_degrees)
    return Point(sensor_x + d * math.sin(a), d * math.cos(a))


def triangulate(aim_l_deg: float, aim_r_deg: float) -> Point | None:
    """Range from the two servo angles alone, ignoring the ToF readings.

    Classic stereo: intersect the two aim rays. This is an independent estimate
    of where the sensors are *looking*, which is what makes the agreement check
    below meaningful — it is derived from the servos, not from the rangefinders.

    Returns None when the rays are near-parallel (looking far away, where the
    10cm baseline carries no information) or when they meet behind the rig.
    """
    ol_x, or_x = -BASELINE_M / 2.0, BASELINE_M / 2.0
    al, ar = math.radians(aim_l_deg), math.radians(aim_r_deg)
    dlx, dly = math.sin(al), math.cos(al)
    drx, dry = math.sin(ar), math.cos(ar)

    denom = dlx * (-dry) - dly * (-drx)
    if abs(denom) < 1e-9:
        return None  # parallel: baseline too short to resolve at this range

    bx, by = or_x - ol_x, 0.0
    t = (bx * (-dry) - by * (-drx)) / denom
    if t <= 0:
        return None  # intersection is behind the sensors

    return Point(ol_x + t * dlx, t * dly)


@dataclass(frozen=True)
class Fused:
    theta_deg: float
    point: Point | None
    disagreement_m: float | None
    n_valid: int
    flag: str

    @property
    def is_confident(self) -> bool:
        return self.flag == "ok"


# Servo commands are whole degrees, so each aim angle carries up to half a
# degree of quantisation error, which becomes a lateral error proportional to
# range. Two independent sensors double it. The VL53L0X adds a few percent of
# ranging noise on top. Both terms scale with distance, which is why the
# agreement tolerance has to as well — a constant threshold rejects a flat wall
# at 3m while accepting a genuine step at 0.3m.
_QUANT_DEG = 1.0
_TOL_SLOPE = 2 * math.tan(math.radians(_QUANT_DEG)) + 0.04  # ~0.075 per metre
_TOL_FLOOR_M = 0.02  # sensor noise floor at point-blank range


def agreement_tolerance(range_m: float) -> float:
    """How far apart two readings may land before it means something."""
    return _TOL_FLOOR_M + _TOL_SLOPE * range_m


def expected_separation(range_m: float) -> float:
    """Where the two beams land relative to each other, on a smooth surface.

    This is the part that is easy to get wrong. The sensors are verged to cross
    at VERGE_RANGE_M, so their beams coincide at exactly one distance. Nearer
    or further than that they strike the surface a predictable distance apart:

        separation = B · |1 − r / R|

    On a wall at 0.5m with B=10cm and R=1.5m that is 6.7cm of pure geometry —
    an order of magnitude larger than the sensor noise, and present even on a
    perfectly flat surface. Comparing the observed gap against zero therefore
    flags most of a close wall as broken. The gap is only meaningful as a
    residual against this expectation.
    """
    return BASELINE_M * abs(1.0 - range_m / VERGE_RANGE_M)


def fuse(
    theta_deg: float,
    servo_l: int,
    servo_r: int,
    dist_l_mm: int,
    dist_r_mm: int,
    agreement_tol_m: float | None = None,
) -> Fused:
    """Combine one bearing's two readings into a single point, or refuse to.

    Both sensors are verged on the same target, so in open space they should
    land on the same surface patch. When they disagree by more than the
    tolerance the beams have straddled an edge or one of them is seeing
    through a gap — averaging there invents a surface that is not present, so
    the point is flagged rather than smoothed. Flagged points are the useful
    ones: disagreement localises depth discontinuities to within a degree.

    `agreement_tol_m` defaults to a range-proportional tolerance; pass a float
    to pin it to a constant.
    """
    al = aim_deg(servo_l, SERVO_L_OFFSET_DEG)
    ar = aim_deg(servo_r, SERVO_R_OFFSET_DEG)

    pl = project(-BASELINE_M / 2.0, al, dist_l_mm)
    pr = project(BASELINE_M / 2.0, ar, dist_r_mm)
    n_valid = sum(p is not None for p in (pl, pr))

    if n_valid == 0:
        return Fused(theta_deg, None, None, 0, "no_return")

    if n_valid == 1:
        only = pl or pr
        return Fused(theta_deg, only, None, 1, "single")

    gap = math.hypot(pl.x - pr.x, pl.y - pr.y)
    mean_range = (pl.range_m + pr.range_m) / 2.0
    # What the gap should be here if the surface is smooth, then how far the
    # observed gap strays from it. The residual is the actual signal.
    residual = abs(gap - expected_separation(mean_range))
    tol = (agreement_tolerance(mean_range)
           if agreement_tol_m is None else agreement_tol_m)
    if residual > tol:
        # Keep the nearer of the two: on an edge, the near surface is the real
        # one and the far reading is the beam that slipped past it.
        nearer = pl if pl.range_m <= pr.range_m else pr
        return Fused(theta_deg, nearer, residual, 2, "edge")

    mid = Point((pl.x + pr.x) / 2.0, (pl.y + pr.y) / 2.0)
    return Fused(theta_deg, mid, residual, 2, "ok")
