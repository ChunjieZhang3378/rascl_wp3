import math
import os
from pathlib import Path

import pytest

from rascl_wp3_ss26_group16.wp3_tsk2 import (
    cartesian_to_cylindrical,
    solve_robotics_toolbox_tcp_ik,
)


def project_urdf_path():
    return (
        Path(__file__).resolve().parents[2]
        / "rascl_description"
        / "urdf"
        / "rascl.urdf"
    )


def test_cartesian_to_cylindrical():
    class Point:
        x = 0.25 * math.cos(0.4)
        y = 0.25 * math.sin(0.4)
        z = 0.03

    point = Point()
    radius, theta, height = cartesian_to_cylindrical(point)

    assert math.isclose(radius, 0.25)
    assert math.isclose(theta, 0.4)
    assert math.isclose(height, 0.03)


def test_robotics_toolbox_loads_project_urdf():
    rtb = pytest.importorskip("roboticstoolbox")

    robot = rtb.ERobot.URDF(str(project_urdf_path()))

    assert robot.n >= 4


def test_robotics_toolbox_position_only_ik_for_task2_target():
    rtb = pytest.importorskip("roboticstoolbox")
    spatialmath = pytest.importorskip("spatialmath")
    robot = rtb.ERobot.URDF(str(project_urdf_path()))

    # Configured task-frame placement (0.165, 0.050, 0.038), transformed to
    # the URDF frame by the configured -pi/2 angular offset.
    target = spatialmath.SE3(0.050, -0.165, 0.038)
    tool_to_tcp = spatialmath.SE3(0.0, 0.038, 0.0)
    solution = solve_robotics_toolbox_tcp_ik(
        robot,
        target,
        tool_to_tcp,
        spatialmath,
    )
    actual_tcp_pose = robot.fkine(solution) * tool_to_tcp

    assert actual_tcp_pose.t == pytest.approx(target.t, abs=1e-4)


def test_robotics_toolbox_visualize_task2_motion():
    if os.environ.get("SHOW_RTB_VISUALIZATION") != "1":
        pytest.skip("set SHOW_RTB_VISUALIZATION=1 to open the Robotics Toolbox plot")

    rtb = pytest.importorskip("roboticstoolbox")
    spatialmath = pytest.importorskip("spatialmath")
    pytest.importorskip("matplotlib")
    robot = rtb.ERobot.URDF(str(project_urdf_path()))

    task2_targets = [
        spatialmath.SE3(0.050, -0.215, 0.045),
        spatialmath.SE3(0.050, -0.165, 0.038),
        spatialmath.SE3(0.050, -0.215, 0.045),
    ]
    tool_to_tcp = spatialmath.SE3(0.0, 0.038, 0.0)
    joint_waypoints = [[0.0] * robot.n]
    seed = joint_waypoints[0]
    print("\nTask 2 Robotics Toolbox waypoint diagnostics:")
    for index, target in enumerate(task2_targets, start=1):
        solution = solve_robotics_toolbox_tcp_ik(
            robot,
            target,
            tool_to_tcp,
            spatialmath,
            q0=seed,
        )
        solved_end_effector_pose = robot.fkine(solution)
        solved_tcp_pose = solved_end_effector_pose * tool_to_tcp
        print(
            f"waypoint {index}: "
            f"desired tcp={target.t.round(4).tolist()}, "
            f"end effector={solved_end_effector_pose.t.round(4).tolist()}, "
            f"actual tcp={solved_tcp_pose.t.round(4).tolist()}"
        )
        joint_waypoints.append(list(solution))
        seed = solution
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
