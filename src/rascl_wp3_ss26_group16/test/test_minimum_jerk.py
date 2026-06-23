"""Tests for the Task 1 minimum-jerk trajectory generator."""

import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).parents[1] / "src"))

from rascl_wp3_ss26_group16.wp3_tsk1 import minimum_jerk_segment  # noqa: E402


def test_minimum_jerk_reaches_goal_with_zero_endpoint_derivatives():
    start = [0.0, -0.2, 0.4, 0.1]
    goal = [0.5, 0.3, -0.1, 0.6]

    samples = minimum_jerk_segment(start, goal, 1.0, 0.02)
    final_time, positions, velocities, accelerations = samples[-1]

    assert math.isclose(final_time, 1.0)
    assert positions == pytest.approx(goal)
    assert velocities == pytest.approx([0.0] * 4, abs=1e-12)
    assert accelerations == pytest.approx([0.0] * 4, abs=1e-12)


def test_minimum_jerk_sample_times_are_increasing_and_include_duration():
    samples = minimum_jerk_segment([0.0], [1.0], 0.11, 0.05)
    sample_times = [sample[0] for sample in samples]

    assert sample_times == pytest.approx([0.05, 0.10, 0.11])
    assert all(
        earlier < later for earlier, later in zip(sample_times, sample_times[1:])
    )


@pytest.mark.parametrize("duration", [0.0, -1.0])
def test_minimum_jerk_rejects_non_positive_duration(duration):
    with pytest.raises(ValueError, match="duration must be positive"):
        minimum_jerk_segment([0.0], [1.0], duration, 0.02)
