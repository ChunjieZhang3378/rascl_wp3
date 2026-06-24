import math

import pytest

from rascl_wp3_ss26_group16.wp3_tsk2 import (
    cartesian_to_cylindrical,
    cylindrical_to_cartesian,
    solve_planar_ik,
)


def test_cylindrical_cartesian_round_trip():
    point = cylindrical_to_cartesian(0.25, 0.4, 0.03)
    radius, theta, height = cartesian_to_cylindrical(point)

    assert math.isclose(radius, 0.25)
    assert math.isclose(theta, 0.4)
    assert math.isclose(height, 0.03)


def test_planar_ik_reaches_requested_point():
    upper_length = 0.1878829423
    lower_length = 0.12909
    shoulder_height = 0.123001
    radius = 0.22
    height = 0.04

    upper, lower = solve_planar_ik(
        radius, height, shoulder_height, upper_length, lower_length
    )
    reconstructed_radius = (
        upper_length * math.cos(upper)
        + lower_length * math.cos(upper + lower)
    )
    reconstructed_height = shoulder_height + (
        upper_length * math.sin(upper)
        + lower_length * math.sin(upper + lower)
    )

    assert math.isclose(reconstructed_radius, radius, abs_tol=1e-9)
    assert math.isclose(reconstructed_height, height, abs_tol=1e-9)


def test_planar_ik_rejects_unreachable_pose():
    with pytest.raises(ValueError, match="outside the IK workspace"):
        solve_planar_ik(1.0, 0.0, 0.123001, 0.1878829423, 0.12909)
