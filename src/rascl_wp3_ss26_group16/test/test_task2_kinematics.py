import math
import os
from pathlib import Path

import pytest

from rascl_wp3_ss26_group16.wp3_tsk2 import (
    cartesian_to_cylindrical,
    cylindrical_to_cartesian,
    solve_planar_ik,
)


def project_urdf_path():
    return (
        Path(__file__).resolve().parents[2]
        / "rascl_description"
        / "urdf"
        / "rascl.urdf"
    )


def end_effector_pose_for_tcp(tcp_pose, tool_to_tcp):
    """Convert a desired TCP pose into the required end-effector-link pose."""
    return tcp_pose * tool_to_tcp.inv()


def solve_robotics_toolbox_tcp_ik(
    robot,
    tcp_pose,
    tool_to_tcp,
    spatialmath,
    q0=None,
    iterations=20,
    tolerance=1e-4,
):
    """Solve position-only IK while correcting for the current TCP orientation."""
    end_effector_target = end_effector_pose_for_tcp(tcp_pose, tool_to_tcp)
    seed = q0
    last_solution = None
    last_tcp_pose = None

    for _ in range(iterations):
        solution = robot.ikine_LM(
            end_effector_target,
            q0=seed,
            mask=[1, 1, 1, 0, 0, 0],
        )
        if not solution.success:
            return solution, robot.fkine(seed) * tool_to_tcp if seed is not None else None

        actual_tcp_pose = robot.fkine(solution.q) * tool_to_tcp
        position_error = tcp_pose.t - actual_tcp_pose.t
        error_norm = math.sqrt(sum(float(value) ** 2 for value in position_error))
        if error_norm <= tolerance:
            return solution, actual_tcp_pose

        corrected_target = end_effector_target.t + position_error
        end_effector_target = spatialmath.SE3(
            float(corrected_target[0]),
            float(corrected_target[1]),
            float(corrected_target[2]),
        )
        seed = solution.q
        last_solution = solution
        last_tcp_pose = actual_tcp_pose

    return last_solution, last_tcp_pose


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


def test_robotics_toolbox_loads_project_urdf():
    rtb = pytest.importorskip("roboticstoolbox")

    robot = rtb.ERobot.URDF(str(project_urdf_path()))

    assert robot.n >= 4


def test_robotics_toolbox_position_only_ik_for_task2_target():
    rtb = pytest.importorskip("roboticstoolbox")
    spatialmath = pytest.importorskip("spatialmath")
    robot = rtb.ERobot.URDF(str(project_urdf_path()))

    target = spatialmath.SE3(0.25, 0.03, 0.05)
    tool_to_tcp = spatialmath.SE3(0.0, 0.04, 0.0)
    solution, actual_tcp_pose = solve_robotics_toolbox_tcp_ik(
        robot,
        target,
        tool_to_tcp,
        spatialmath,
    )

    assert solution.success
    assert actual_tcp_pose.t == pytest.approx(target.t, abs=1e-4)


def test_robotics_toolbox_visualize_task2_motion():
    if os.environ.get("SHOW_RTB_VISUALIZATION") != "1":
        pytest.skip("set SHOW_RTB_VISUALIZATION=1 to open the Robotics Toolbox plot")

    rtb = pytest.importorskip("roboticstoolbox")
    spatialmath = pytest.importorskip("spatialmath")
    pytest.importorskip("matplotlib")
    robot = rtb.ERobot.URDF(str(project_urdf_path()))

    task2_targets = [
        spatialmath.SE3(0.27, 0.03, 0.05),
        spatialmath.SE3(0.25, 0.03, 0.03),
        spatialmath.SE3(0.27, 0.03, 0.05),
    ]
    tool_to_tcp = spatialmath.SE3(0.0, 0.04, 0.0)
    joint_waypoints = [[0.0] * robot.n]
    seed = joint_waypoints[0]
    print("\nTask 2 Robotics Toolbox waypoint diagnostics:")
    for index, target in enumerate(task2_targets, start=1):
        solution, solved_tcp_pose = solve_robotics_toolbox_tcp_ik(
            robot,
            target,
            tool_to_tcp,
            spatialmath,
            q0=seed,
        )
        assert solution.success
        solved_end_effector_pose = robot.fkine(solution.q)
        print(
            f"waypoint {index}: "
            f"desired tcp={target.t.round(4).tolist()}, "
            f"end effector={solved_end_effector_pose.t.round(4).tolist()}, "
            f"actual tcp={solved_tcp_pose.t.round(4).tolist()}"
        )
        joint_waypoints.append(list(solution.q))
        seed = solution.q
    joint_waypoints.append([0.0] * robot.n)

    env = robot.plot(joint_waypoints[0], backend="pyplot", block=False)
    steps_per_segment = 60
    for start, goal in zip(joint_waypoints, joint_waypoints[1:]):
        for step in range(steps_per_segment + 1):
            blend = step / steps_per_segment
            q = [
                start_value + blend * (goal_value - start_value)
                for start_value, goal_value in zip(start, goal)
            ]
            robot.q = q
            env.step(0.03)
    robot.plot(joint_waypoints[-1], backend="pyplot", block=True)
