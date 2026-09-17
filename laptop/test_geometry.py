"""Geometry round-trip tests.

The firmware computes servo angles from a bearing; this package computes a
bearing back from servo angles. Those two conversions live in different
languages on different machines, and nothing at runtime can tell you they have
drifted apart — a sign error would simply produce a plausible, wrong map.

These tests pin the round trip against synthetic readings, so the geometry can
be checked without the rig on the desk.

    python test_geometry.py      # plain asserts, no pytest needed
    pytest test_geometry.py      # also works
"""

from __future__ import annotations

import math

from geometry import (
    BASELINE_M,
    SERVO_L_OFFSET_DEG,
    SERVO_R_OFFSET_DEG,
    aim_deg,
    fuse,
    triangulate,
    verge_angles,
)


def _wall_reading_mm(sensor_x: float, aim_degrees: float, wall_y: float) -> int:
    """What a sensor at `sensor_x` reads against a flat wall at y = wall_y."""
    a = math.radians(aim_degrees)
    if math.cos(a) <= 0:
        return -1  # aimed away from the wall
    return int(round((wall_y / math.cos(a)) * 1000))


def test_sensors_toe_inward_at_boresight():
    """At theta=0 both sensors must converge, not point parallel."""
    cmd_l, cmd_r = verge_angles(0.0)
    assert cmd_l > 90, f"left should toe right, got {cmd_l}"
    assert cmd_r < 90, f"right should toe left, got {cmd_r}"
    assert (cmd_l - 90) == (90 - cmd_r), "vergence should be symmetric at boresight"


def test_flat_wall_is_recovered_flat():
    """A wall at 1.2m must come back as a wall at 1.2m across the sweep.

    Bounded at +-45deg: slant range is wall_y/cos(theta), so a 1.2m wall is
    already 2.4m away at 60deg — past what the sensor can see at all.
    """
    wall_y = 1.2
    for theta in range(-45, 46, 5):
        cmd_l, cmd_r = verge_angles(float(theta))
        d_l = _wall_reading_mm(-BASELINE_M / 2, aim_deg(cmd_l, SERVO_L_OFFSET_DEG), wall_y)
        d_r = _wall_reading_mm(BASELINE_M / 2, aim_deg(cmd_r, SERVO_R_OFFSET_DEG), wall_y)

        f = fuse(float(theta), cmd_l, cmd_r, d_l, d_r)
        assert f.point is not None, f"lost the wall at theta={theta}"
        assert abs(f.point.y - wall_y) < 0.02, (
            f"theta={theta}: wall recovered at y={f.point.y:.3f}, expected {wall_y}"
        )


def test_a_flat_wall_is_not_read_as_a_field_of_edges():
    """Regression: a constant tolerance flagged most of a flat wall as edges.

    Servo quantisation alone puts the two readings several centimetres apart at
    3m. With a fixed 5cm threshold, 103 of 179 bearings on a flat wall came
    back flagged — the edge signal was worthless because it fired everywhere.
    The tolerance has to scale with range.
    """
    for wall_y in (0.5, 1.2, 1.8):
        flagged = 0
        for theta in range(-70, 71, 2):
            cmd_l, cmd_r = verge_angles(float(theta))
            d_l = _wall_reading_mm(-BASELINE_M / 2, aim_deg(cmd_l, SERVO_L_OFFSET_DEG), wall_y)
            d_r = _wall_reading_mm(BASELINE_M / 2, aim_deg(cmd_r, SERVO_R_OFFSET_DEG), wall_y)
            if fuse(float(theta), cmd_l, cmd_r, d_l, d_r).flag == "edge":
                flagged += 1
        assert flagged == 0, f"wall at {wall_y}m: {flagged} phantom edges"


def test_a_real_step_still_registers_at_close_range():
    """The looser tolerance must not blind us to genuine discontinuities."""
    cmd_l, cmd_r = verge_angles(0.0)
    near = _wall_reading_mm(-BASELINE_M / 2, aim_deg(cmd_l, SERVO_L_OFFSET_DEG), 0.30)
    far = _wall_reading_mm(BASELINE_M / 2, aim_deg(cmd_r, SERVO_R_OFFSET_DEG), 0.45)
    assert fuse(0.0, cmd_l, cmd_r, near, far).flag == "edge"


def test_beyond_sensor_range_is_discarded():
    """The VL53L0X returns a number past its range; the number is not a surface."""
    from geometry import MAX_RANGE_M, project
    assert project(0.0, 0.0, int((MAX_RANGE_M + 0.5) * 1000)) is None
    assert project(0.0, 0.0, int((MAX_RANGE_M - 0.5) * 1000)) is not None


def test_out_of_range_is_not_a_zero():
    """-1 is a sentinel. Treating it as 0mm would put a phantom at the origin."""
    f = fuse(0.0, 92, 88, -1, -1)
    assert f.flag == "no_return"
    assert f.point is None
    assert f.n_valid == 0


def test_one_eye_still_reports():
    """One dead sensor degrades the scan, it does not end it."""
    cmd_l, cmd_r = verge_angles(0.0)
    d_l = _wall_reading_mm(-BASELINE_M / 2, aim_deg(cmd_l, SERVO_L_OFFSET_DEG), 1.0)
    f = fuse(0.0, cmd_l, cmd_r, d_l, -1)
    assert f.flag == "single"
    assert f.n_valid == 1
    assert f.point is not None


def test_disagreement_flags_an_edge_and_keeps_the_near_surface():
    """Straddling a depth discontinuity must not average into empty space."""
    cmd_l, cmd_r = verge_angles(0.0)
    near_mm = _wall_reading_mm(-BASELINE_M / 2, aim_deg(cmd_l, SERVO_L_OFFSET_DEG), 0.5)
    # 1.5m, not 2.0m: the far surface has to stay inside MAX_RANGE_M or it is
    # discarded as noise and the bearing degrades to "single" instead.
    far_mm = _wall_reading_mm(BASELINE_M / 2, aim_deg(cmd_r, SERVO_R_OFFSET_DEG), 1.5)

    f = fuse(0.0, cmd_l, cmd_r, near_mm, far_mm)
    assert f.flag == "edge"
    assert f.disagreement_m > 0.05
    # The near surface is the real one; the far reading slipped past it.
    assert f.point.y < 0.6, f"kept the far surface at y={f.point.y:.3f}"


def test_triangulation_agrees_with_the_verge_distance():
    """Servo angles alone should place the crossing at VERGE_RANGE_M."""
    from geometry import VERGE_RANGE_M

    for theta in (-45.0, 0.0, 45.0):
        cmd_l, cmd_r = verge_angles(theta)
        p = triangulate(aim_deg(cmd_l, SERVO_L_OFFSET_DEG),
                        aim_deg(cmd_r, SERVO_R_OFFSET_DEG))
        assert p is not None, f"no crossing at theta={theta}"
        # Servo commands are rounded to whole degrees, so the crossing lands
        # near the verge range rather than exactly on it. At 1.5m with a 10cm
        # baseline one degree of rounding is worth roughly half a metre.
        assert 0.8 < p.range_m < 4.0, (
            f"theta={theta}: crossing at {p.range_m:.2f}m, expected near {VERGE_RANGE_M}m"
        )


def test_parallel_rays_have_no_crossing():
    """Looking straight ahead with no vergence, the baseline tells you nothing."""
    assert triangulate(0.0, 0.0) is None


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print(f"  ok    {name}")
        except AssertionError as exc:
            failures += 1
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{'all passed' if not failures else f'{failures} failed'}")
    raise SystemExit(1 if failures else 0)
