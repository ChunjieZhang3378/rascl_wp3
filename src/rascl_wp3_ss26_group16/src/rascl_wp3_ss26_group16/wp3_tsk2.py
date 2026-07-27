"""Online Cartesian cube-pose pick-and-place planning for Task 2."""

import math
import site
import sys
import time
from collections import deque
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .wp3_tsk1 import JOINT_NAMES, minimum_jerk_segment


def cartesian_to_cylindrical(point):
    """Convert a Cartesian point to (radius, theta, height)."""
    return math.hypot(point.x, point.y), math.atan2(point.y, point.x), point.z


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
    """Solve position-only Robotics Toolbox IK for a TCP offset."""
    end_effector_target = end_effector_pose_for_tcp(tcp_pose, tool_to_tcp)
    seed_candidates = []
    if q0 is not None:
        seed_candidates.append(list(q0))
    seed_candidates.append([0.0] * robot.n)
    for partial_seed in ([0.0, -0.8, -0.2, 0.0], [0.0, -1.0, -0.2, 0.0]):
        seed_candidates.append(
            list(partial_seed[: robot.n]) + [0.0] * max(0, robot.n - len(partial_seed))
        )
    last_solution = None
    last_error = None
    best_error_norm = float("inf")
    best_tcp_pose = None

    for initial_seed in seed_candidates:
        seed = initial_seed
        target_for_seed = end_effector_target
        for _ in range(iterations):
            solution = robot.ikine_LM(
                target_for_seed,
                q0=seed,
                mask=[1, 1, 1, 0, 0, 0],
            )
            if not solution.success:
                last_error = getattr(solution, "reason", "unknown reason")
                break

            actual_tcp_pose = robot.fkine(solution.q) * tool_to_tcp
            position_error = tcp_pose.t - actual_tcp_pose.t
            error_norm = math.sqrt(sum(float(value) ** 2 for value in position_error))
            if error_norm < best_error_norm:
                best_error_norm = error_norm
                best_tcp_pose = actual_tcp_pose
            if error_norm <= tolerance:
                return list(solution.q)

            corrected_target = target_for_seed.t + position_error
            target_for_seed = spatialmath.SE3(
                float(corrected_target[0]),
                float(corrected_target[1]),
                float(corrected_target[2]),
            )
            seed = solution.q
            last_solution = solution

    requested = [round(float(value), 4) for value in tcp_pose.t]
    if last_solution is None:
        raise ValueError(
            "Robotics Toolbox IK failed for TCP "
            f"{requested}; last error: {last_error}"
        )
    actual = (
        [round(float(value), 4) for value in best_tcp_pose.t]
        if best_tcp_pose is not None
        else None
    )
    raise ValueError(
        "Robotics Toolbox TCP IK did not converge for TCP "
        f"{requested}; best actual TCP {actual}; "
        f"best error {best_error_norm * 1000.0:.2f} mm"
    )


def add_robotics_toolbox_site_packages():
    """Expose the container venv packages to ROS entry points using system Python."""
    venv_site_packages = Path(
        f"/opt/rascl_venv/lib/python{sys.version_info.major}."
        f"{sys.version_info.minor}/site-packages"
    )
    if venv_site_packages.exists():
        site.addsitedir(str(venv_site_packages))


class Task2Node(Node):
    """Receive cube poses, solve IK, and execute online pick-and-place motions."""

    def __init__(self):
        super().__init__("wp3_tsk2")
        self.declare_parameter("cartesian_cube_topic", "/goal_poses")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter(
            "trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("sample_period", 0.02)
        self.declare_parameter("segment_duration", 4.0)
        self.declare_parameter("minimum_x", -0.25)
        self.declare_parameter("maximum_x", 0.25)
        self.declare_parameter("minimum_y", 0.03)
        self.declare_parameter("maximum_y", 0.25)
        self.declare_parameter("target_x", 0.25)
        self.declare_parameter("target_y", 0.03)
        self.declare_parameter("target_z", 0.03)
        self.declare_parameter("pre_place_x", 0.26)
        self.declare_parameter("pre_place_y", 0.03)
        self.declare_parameter("pre_place_z", 0.05)
        self.declare_parameter("tcp_offset_from_cube_center_z", 0.0)
        self.declare_parameter("approach_height", 0.08)
        self.declare_parameter("robotics_toolbox_tcp_offset", [0.0, 0.04, 0.0])
        self.declare_parameter("robotics_toolbox_theta_offset", -1.570796327)
        self.declare_parameter("robotics_toolbox_gripper_seed_position", 0.0)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", 2.5)
        self.declare_parameter("home_positions", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("simulate_only", False)

        self.sample_period = float(self.get_parameter("sample_period").value)
        self.segment_duration = float(self.get_parameter("segment_duration").value)
        self.minimum_x = float(self.get_parameter("minimum_x").value)
        self.maximum_x = float(self.get_parameter("maximum_x").value)
        self.minimum_y = float(self.get_parameter("minimum_y").value)
        self.maximum_y = float(self.get_parameter("maximum_y").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self.pre_place_x = float(self.get_parameter("pre_place_x").value)
        self.pre_place_y = float(self.get_parameter("pre_place_y").value)
        self.pre_place_z = float(self.get_parameter("pre_place_z").value)
        self.tcp_offset_from_cube_center_z = float(
            self.get_parameter("tcp_offset_from_cube_center_z").value
        )
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.simulate_only = bool(self.get_parameter("simulate_only").value)
        self.robotics_toolbox_gripper_seed_position = float(
            self.get_parameter("robotics_toolbox_gripper_seed_position").value
        )
        self.robotics_toolbox_tcp_offset = [
            float(value)
            for value in self.get_parameter("robotics_toolbox_tcp_offset").value
        ]
        self.robotics_toolbox_theta_offset = float(
            self.get_parameter("robotics_toolbox_theta_offset").value
        )
        self.home_positions = [
            float(value) for value in self.get_parameter("home_positions").value
        ]
        if len(self.home_positions) != len(JOINT_NAMES):
            raise ValueError("home_positions must contain four joint positions")
        if len(self.robotics_toolbox_tcp_offset) != 3:
            raise ValueError("robotics_toolbox_tcp_offset must contain three values")
        if self.sample_period <= 0.0 or self.segment_duration <= 0.0:
            raise ValueError("sample_period and segment_duration must be positive")
        if not self.minimum_x < self.maximum_x:
            raise ValueError("minimum_x must be smaller than maximum_x")
        if not self.minimum_y < self.maximum_y:
            raise ValueError("minimum_y must be smaller than maximum_y")

        self.pending_cubes = deque()
        self.current_positions = None
        self.robotics_toolbox_robot = None
        self.robotics_toolbox_spatialmath = None
        self.robotics_toolbox_tool_to_tcp = None
        self.robotics_toolbox_seed = None
        self._configure_robotics_toolbox_ik()

        self.goal_pose_publisher = self.create_publisher(
            Point, str(self.get_parameter("goal_pose_topic").value), 10
        )
        self.create_subscription(
            Point,
            str(self.get_parameter("cartesian_cube_topic").value),
            self._receive_cartesian_cube,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._receive_joint_state,
            10,
        )

        self.joint_state_publisher = None
        self.action_client = None
        if self.simulate_only:
            self.joint_state_publisher = self.create_publisher(
                JointState, str(self.get_parameter("joint_state_topic").value), 10
            )
        else:
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("trajectory_action").value),
            )

    def _receive_cartesian_cube(self, message):
        cartesian_pose = (message.x, message.y, message.z)
        if not all(math.isfinite(value) for value in cartesian_pose):
            self.get_logger().error("Rejected cube pose containing a non-finite value")
            return
        try:
            self.validate_cube_pose(message.x, message.y)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        cylindrical_pose = cartesian_to_cylindrical(message)
        radius, theta, height = cylindrical_pose
        self.goal_pose_publisher.publish(message)
        self.pending_cubes.append(cylindrical_pose)
        self.get_logger().info(
            f"Queued Cartesian cube ({message.x:.3f}, {message.y:.3f}, "
            f"{message.z:.3f}) m; internal r={radius:.3f} m, "
            f"theta={theta:.3f} rad, z={height:.3f} m"
        )

    def _receive_joint_state(self, message):
        positions_by_name = dict(zip(message.name, message.position))
        if all(name in positions_by_name for name in JOINT_NAMES):
            self.current_positions = [
                float(positions_by_name[name]) for name in JOINT_NAMES
            ]

    def _configure_robotics_toolbox_ik(self):
        """Load Robotics Toolbox and the project URDF."""
        add_robotics_toolbox_site_packages()
        try:
            import roboticstoolbox as rtb
            import spatialmath
        except ImportError as error:
            raise ValueError(
                "roboticstoolbox-python or spatialmath-python is not installed"
            ) from error

        urdf_path = (
            get_package_share_directory("rascl_description") + "/urdf/rascl.urdf"
        )
        self.robotics_toolbox_robot = rtb.ERobot.URDF(urdf_path)
        if self.robotics_toolbox_robot.n < len(JOINT_NAMES):
            raise ValueError("Robotics Toolbox URDF has fewer joints than Task 2")
        self.robotics_toolbox_spatialmath = spatialmath
        self.robotics_toolbox_tool_to_tcp = spatialmath.SE3(
            self.robotics_toolbox_tcp_offset[0],
            self.robotics_toolbox_tcp_offset[1],
            self.robotics_toolbox_tcp_offset[2],
        )
        self.robotics_toolbox_seed = list(self.home_positions)
        self.robotics_toolbox_seed[3] = self.robotics_toolbox_gripper_seed_position
        self.get_logger().info("Task 2 using Robotics Toolbox IK backend")

    def validate_cube_pose(self, x, y):
        """Reject Cartesian cube positions outside the configured x/y bounds."""
        if not self.minimum_x <= x <= self.maximum_x:
            raise ValueError(
                f"cube x {x:.3f} m is outside "
                f"[{self.minimum_x:.3f}, {self.maximum_x:.3f}] m"
            )
        if not self.minimum_y <= y <= self.maximum_y:
            raise ValueError(
                f"cube y {y:.3f} m is outside "
                f"[{self.minimum_y:.3f}, {self.maximum_y:.3f}] m"
            )

    def inverse_kinematics(self, radius, theta, height, gripper_position):
        """Return joint commands from Robotics Toolbox for a cylindrical TCP pose."""
        # Task 2 Cartesian input uses the calibrated robot/task frame. The URDF
        # model used by Robotics Toolbox has its zero shoulder direction rotated
        # by 90 degrees, so convert the target angle before solving IK.
        toolbox_theta = theta + self.robotics_toolbox_theta_offset
        x = radius * math.cos(toolbox_theta)
        y = radius * math.sin(toolbox_theta)
        target = self.robotics_toolbox_spatialmath.SE3(x, y, height)
        seed = list(self.robotics_toolbox_seed)
        seed[3] = self.robotics_toolbox_gripper_seed_position
        joints = solve_robotics_toolbox_tcp_ik(
            self.robotics_toolbox_robot,
            target,
            self.robotics_toolbox_tool_to_tcp,
            self.robotics_toolbox_spatialmath,
            q0=seed,
        )
        joints = [float(value) for value in joints[: len(JOINT_NAMES)]]
        joints[3] = gripper_position
        for index, joint in enumerate(joints[:3]):
            if not -math.pi / 2.0 <= joint <= math.pi / 2.0:
                raise ValueError(
                    f"Robotics Toolbox IK solution for {JOINT_NAMES[index]} "
                    f"({joint:.3f} rad) exceeds the URDF joint limits"
                )
        self.robotics_toolbox_seed = list(joints)
        self.robotics_toolbox_seed[3] = self.robotics_toolbox_gripper_seed_position
        return joints

    def build_pick_and_place_waypoints(self, cube_pose):
        """Build the Task-1-style approach, grasp, lift, place, and home sequence."""
        self.robotics_toolbox_seed = (
            list(self.current_positions)
            if self.current_positions is not None
            else list(self.home_positions)
        )
        self.robotics_toolbox_seed[3] = self.robotics_toolbox_gripper_seed_position

        cube_radius, cube_theta, cube_z = cube_pose
        target_radius = math.hypot(self.target_x, self.target_y)
        target_theta = math.atan2(self.target_y, self.target_x)
        pre_place_radius = math.hypot(self.pre_place_x, self.pre_place_y)
        pre_place_theta = math.atan2(self.pre_place_y, self.pre_place_x)
        open_gripper = float(self.get_parameter("gripper_open_position").value)
        closed_gripper = float(self.get_parameter("gripper_closed_position").value)
        # Input z is the cube center. The controlled point is the TCP, defined
        # relative to the cube center with a configurable z offset. The fixed
        # placement and pre-place coordinates are already TCP coordinates.
        cube_grasp_z = cube_z + self.tcp_offset_from_cube_center_z
        target_grasp_z = self.target_z
        cube_approach_z = cube_grasp_z + self.approach_height

        cube_approach_open = self.inverse_kinematics(
            cube_radius, cube_theta, cube_approach_z, open_gripper
        )
        cube_grasp_open = self.inverse_kinematics(
            cube_radius, cube_theta, cube_grasp_z, open_gripper
        )
        cube_grasp_closed = self.inverse_kinematics(
            cube_radius, cube_theta, cube_grasp_z, closed_gripper
        )
        cube_approach_closed = self.inverse_kinematics(
            cube_radius, cube_theta, cube_approach_z, closed_gripper
        )
        pre_place_closed = self.inverse_kinematics(
            pre_place_radius, pre_place_theta, self.pre_place_z, closed_gripper
        )
        target_grasp_closed = self.inverse_kinematics(
            target_radius, target_theta, target_grasp_z, closed_gripper
        )
        target_grasp_open = self.inverse_kinematics(
            target_radius, target_theta, target_grasp_z, open_gripper
        )
        pre_place_open = self.inverse_kinematics(
            pre_place_radius, pre_place_theta, self.pre_place_z, open_gripper
        )

        # Fold the upper and lower arm before rotating the shoulder. Keep the
        # gripper closed and preserve the current shoulder angle while folding.
        folded_at_cube = [
            cube_approach_closed[0],
            self.home_positions[1],
            self.home_positions[2],
            closed_gripper,
        ]
        folded_at_target = [
            pre_place_closed[0],
            self.home_positions[1],
            self.home_positions[2],
            closed_gripper,
        ]

        return [
            self.home_positions,
            cube_approach_open,
            cube_grasp_open,
            cube_grasp_closed,
            cube_approach_closed,
            folded_at_cube,
            folded_at_target,
            pre_place_closed,
            target_grasp_closed,
            target_grasp_open,
            pre_place_open,
            self.home_positions,
        ]

    def generate_trajectory(self, start, waypoints):
        """Generate one continuous minimum-jerk trajectory online."""
        trajectory = [(0.0, list(start), [0.0] * 4, [0.0] * 4)]
        elapsed_time = 0.0
        previous = list(start)
        for waypoint in waypoints:
            samples = minimum_jerk_segment(
                previous, waypoint, self.segment_duration, self.sample_period
            )
            for local_time, positions, velocities, accelerations in samples:
                trajectory.append(
                    (elapsed_time + local_time, positions, velocities, accelerations)
                )
            elapsed_time += self.segment_duration
            previous = waypoint
        return trajectory

    def execute_trajectory(self, trajectory):
        """Execute through the same trajectory controller used by Task 1."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for time_from_start, positions, velocities, accelerations in trajectory:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            seconds = int(time_from_start)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = int(
                round((time_from_start - seconds) * 1_000_000_000)
            )
            if point.time_from_start.nanosec == 1_000_000_000:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec = 0
            goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("controller rejected the Task 2 trajectory")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("controller returned no Task 2 result")
        if wrapped_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Task 2 trajectory failed: {wrapped_result.result.error_string}"
            )

    def publish_joint_state(self, positions, velocities=None):
        """Publish one complete simulated joint state."""
        if self.joint_state_publisher is None:
            raise RuntimeError("simulation joint-state publisher is unavailable")

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        message.velocity = (
            list(velocities) if velocities is not None else [0.0] * len(JOINT_NAMES)
        )
        self.joint_state_publisher.publish(message)

    def preview_trajectory(self, trajectory):
        """Publish generated positions for robot_state_publisher and RViz."""
        previous_time = 0.0
        for time_from_start, positions, velocities, _accelerations in trajectory:
            delay = max(0.0, time_from_start - previous_time)
            if delay > 0.0:
                time.sleep(delay)
            self.publish_joint_state(positions, velocities)
            previous_time = time_from_start

    def run(self):
        """Wait for cube poses and process any number of cubes sequentially."""
        if not self.simulate_only:
            self.get_logger().info("Waiting for joint trajectory controller")
            if not self.action_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError("joint trajectory controller action is unavailable")
        else:
            self.current_positions = list(self.home_positions)

        self.get_logger().info(
            "Ready: publish Cartesian Point(x,y,z) on "
            f"{self.get_parameter('cartesian_cube_topic').value}"
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.pending_cubes:
                if self.simulate_only:
                    self.publish_joint_state(self.current_positions)
                continue
            if self.current_positions is None:
                if self.simulate_only:
                    self.current_positions = list(self.home_positions)
                else:
                    self.get_logger().warning(
                        "Cube queued, but no complete joint state has been received"
                    )
                    continue

            cube_pose = self.pending_cubes.popleft()
            try:
                waypoints = self.build_pick_and_place_waypoints(cube_pose)
                trajectory = self.generate_trajectory(
                    self.current_positions, waypoints
                )
                self.get_logger().info(
                    f"Executing online trajectory with {len(trajectory)} samples"
                )
                if self.simulate_only:
                    self.preview_trajectory(trajectory)
                else:
                    self.execute_trajectory(trajectory)
                self.current_positions = list(waypoints[-1])
                self.get_logger().info("Cube pick-and-place completed")
            except (RuntimeError, ValueError) as error:
                self.get_logger().error(str(error))


def main(args=None):
    """Run the Task 2 online planner until ROS shuts down."""
    rclpy.init(args=args)
    node = None
    try:
        node = Task2Node()
        node.run()
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            raise
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
