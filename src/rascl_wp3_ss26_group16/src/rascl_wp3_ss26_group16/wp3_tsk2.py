"""Online cylindrical-pose pick-and-place planning for Task 2."""

import math
import time
from collections import deque

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .wp3_tsk1 import JOINT_NAMES, minimum_jerk_segment


def cylindrical_to_cartesian(radius, theta, height):
    """Convert a robot-base cylindrical pose to Cartesian coordinates."""
    return Point(
        x=radius * math.cos(theta),
        y=radius * math.sin(theta),
        z=height,
    )


def cartesian_to_cylindrical(point):
    """Convert a Cartesian point to (radius, theta, height)."""
    return math.hypot(point.x, point.y), math.atan2(point.y, point.x), point.z


def solve_planar_ik(
    radius,
    height,
    shoulder_height,
    upperarm_length,
    lowerarm_length,
    elbow_up=False,
):
    """Solve the two-link vertical-plane IK for the arm joints."""
    radial_distance = radius
    vertical_distance = height - shoulder_height
    distance_squared = radial_distance ** 2 + vertical_distance ** 2
    cosine_lowerarm = (
        distance_squared - upperarm_length ** 2 - lowerarm_length ** 2
    ) / (2.0 * upperarm_length * lowerarm_length)
    if cosine_lowerarm < -1.0 - 1e-9 or cosine_lowerarm > 1.0 + 1e-9:
        raise ValueError(
            f"pose r={radius:.3f}, z={height:.3f} is outside the IK workspace"
        )

    cosine_lowerarm = max(-1.0, min(1.0, cosine_lowerarm))
    lowerarm = math.acos(cosine_lowerarm)
    if elbow_up:
        lowerarm = -lowerarm
    upperarm = math.atan2(vertical_distance, radial_distance) - math.atan2(
        lowerarm_length * math.sin(lowerarm),
        upperarm_length + lowerarm_length * math.cos(lowerarm),
    )
    return upperarm, lowerarm


class Task2Node(Node):
    """Receive cube poses, solve IK, and execute online pick-and-place motions."""

    def __init__(self):
        super().__init__("wp3_tsk2")
        self.declare_parameter("cartesian_cube_topic", "/goal_poses")
        self.declare_parameter("cylindrical_cube_topic", "/cube_pose_cylindrical")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter(
            "trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("sample_period", 0.02)
        self.declare_parameter("segment_duration", 4.0)
        self.declare_parameter("minimum_feasible_radius", 0.13)
        self.declare_parameter("maximum_feasible_radius", 0.21)
        self.declare_parameter("minimum_theta", -math.pi / 2.0)
        self.declare_parameter("maximum_theta", math.pi / 2.0)
        self.declare_parameter("target_radius", 0.19)
        self.declare_parameter("target_theta", -1.5)
        self.declare_parameter("target_z", 0.0205)
        self.declare_parameter("cube_height", 0.041)
        self.declare_parameter("grasp_offset_above_cube_top", 0.015)
        self.declare_parameter("approach_height", 0.08)
        self.declare_parameter("shoulder_height", 0.123001)
        self.declare_parameter("upperarm_length", 0.1878829423)
        self.declare_parameter("lowerarm_length", 0.12909)
        self.declare_parameter("radial_tool_offset", 0.0)
        self.declare_parameter("vertical_tool_offset", 0.0)
        self.declare_parameter("shoulder_sign", -1.0)
        self.declare_parameter("shoulder_offset", 0.0)
        self.declare_parameter("upperarm_sign", 1.0)
        self.declare_parameter("upperarm_offset", -1.1177305070)
        self.declare_parameter("lowerarm_sign", -1.0)
        self.declare_parameter("lowerarm_offset", -1.1169401780)
        self.declare_parameter("elbow_up", True)
        self.declare_parameter("gripper_open_position", 1.0)
        self.declare_parameter("gripper_closed_position", 2.5)
        self.declare_parameter("home_positions", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("simulate_only", False)

        self.sample_period = float(self.get_parameter("sample_period").value)
        self.segment_duration = float(self.get_parameter("segment_duration").value)
        self.minimum_radius = float(
            self.get_parameter("minimum_feasible_radius").value
        )
        self.maximum_radius = float(
            self.get_parameter("maximum_feasible_radius").value
        )
        self.minimum_theta = float(self.get_parameter("minimum_theta").value)
        self.maximum_theta = float(self.get_parameter("maximum_theta").value)
        self.target_radius = float(self.get_parameter("target_radius").value)
        self.target_theta = float(self.get_parameter("target_theta").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self.cube_height = float(self.get_parameter("cube_height").value)
        self.grasp_offset = float(
            self.get_parameter("grasp_offset_above_cube_top").value
        )
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.simulate_only = bool(self.get_parameter("simulate_only").value)
        self.home_positions = [
            float(value) for value in self.get_parameter("home_positions").value
        ]
        if len(self.home_positions) != len(JOINT_NAMES):
            raise ValueError("home_positions must contain four joint positions")
        if self.sample_period <= 0.0 or self.segment_duration <= 0.0:
            raise ValueError("sample_period and segment_duration must be positive")
        if self.cube_height <= 0.0 or self.grasp_offset < 0.0:
            raise ValueError(
                "cube_height must be positive and grasp offset must be non-negative"
            )
        if not 0.0 < self.minimum_radius <= self.maximum_radius:
            raise ValueError("configure positive and ordered feasible radii")
        if not self.minimum_theta < self.maximum_theta:
            raise ValueError("minimum_theta must be smaller than maximum_theta")
        if not self.minimum_theta <= self.target_theta <= self.maximum_theta:
            raise ValueError("target_theta must be inside the feasible angular region")
        if not self.minimum_radius <= self.target_radius <= self.maximum_radius:
            raise ValueError("target_radius must be inside the feasible radial region")

        self.pending_cubes = deque()
        self.current_positions = None
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
            Point,
            str(self.get_parameter("cylindrical_cube_topic").value),
            self._receive_cylindrical_cube,
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
        cylindrical_pose = cartesian_to_cylindrical(message)
        self._queue_cube(cylindrical_pose)

    def _receive_cylindrical_cube(self, message):
        self._queue_cube((message.x, message.y, message.z))

    def _queue_cube(self, cylindrical_pose):
        radius, theta, height = cylindrical_pose
        if not all(math.isfinite(value) for value in cylindrical_pose):
            self.get_logger().error("Rejected cube pose containing a non-finite value")
            return
        try:
            self.validate_cube_pose(radius, theta)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        cartesian_pose = cylindrical_to_cartesian(radius, theta, height)
        self.goal_pose_publisher.publish(cartesian_pose)
        self.pending_cubes.append(cylindrical_pose)
        self.get_logger().info(
            f"Queued cube r={radius:.3f} m, theta={theta:.3f} rad, z={height:.3f} m; "
            f"published Cartesian ({cartesian_pose.x:.3f}, "
            f"{cartesian_pose.y:.3f}, {cartesian_pose.z:.3f})"
        )

    def _receive_joint_state(self, message):
        positions_by_name = dict(zip(message.name, message.position))
        if all(name in positions_by_name for name in JOINT_NAMES):
            self.current_positions = [
                float(positions_by_name[name]) for name in JOINT_NAMES
            ]

    def validate_cube_pose(self, radius, theta):
        """Reject poses outside the documented collision-free feasible region."""
        if not self.minimum_radius <= radius <= self.maximum_radius:
            raise ValueError(
                f"cube radius {radius:.3f} m is outside "
                f"[{self.minimum_radius:.3f}, {self.maximum_radius:.3f}] m"
            )
        if not self.minimum_theta <= theta <= self.maximum_theta:
            raise ValueError(
                f"cube theta {theta:.3f} rad is outside "
                f"[{self.minimum_theta:.3f}, {self.maximum_theta:.3f}] rad"
            )

    def inverse_kinematics(self, radius, theta, height, gripper_position):
        """Return all four commanded joints for a cylindrical tool pose."""
        effective_radius = radius - float(
            self.get_parameter("radial_tool_offset").value
        )
        effective_height = height - float(
            self.get_parameter("vertical_tool_offset").value
        )
        if effective_radius <= 0.0:
            raise ValueError("radial_tool_offset leaves a non-positive wrist radius")

        upperarm, lowerarm = solve_planar_ik(
            effective_radius,
            effective_height,
            float(self.get_parameter("shoulder_height").value),
            float(self.get_parameter("upperarm_length").value),
            float(self.get_parameter("lowerarm_length").value),
            bool(self.get_parameter("elbow_up").value),
        )
        joints = [
            float(self.get_parameter("shoulder_sign").value) * theta
            + float(self.get_parameter("shoulder_offset").value),
            float(self.get_parameter("upperarm_sign").value) * upperarm
            + float(self.get_parameter("upperarm_offset").value),
            float(self.get_parameter("lowerarm_sign").value) * lowerarm
            + float(self.get_parameter("lowerarm_offset").value),
            gripper_position,
        ]
        for index, joint in enumerate(joints[:3]):
            if not -math.pi / 2.0 <= joint <= math.pi / 2.0:
                raise ValueError(
                    f"IK solution for {JOINT_NAMES[index]} ({joint:.3f} rad) "
                    "exceeds the URDF joint limits"
                )
        return joints

    def build_pick_and_place_waypoints(self, cube_pose):
        """Build the Task-1-style approach, grasp, lift, place, and home sequence."""
        cube_radius, cube_theta, cube_z = cube_pose
        target_radius = self.target_radius
        target_theta = self.target_theta
        open_gripper = float(self.get_parameter("gripper_open_position").value)
        closed_gripper = float(self.get_parameter("gripper_closed_position").value)
        # Input z is the cube center. The controlled point is the end-effector
        # midpoint, positioned 15 mm above the center of the cube's top face.
        cube_grasp_z = cube_z + self.cube_height / 2.0 + self.grasp_offset
        target_grasp_z = (
            self.target_z + self.cube_height / 2.0 + self.grasp_offset
        )
        cube_approach_z = cube_grasp_z + self.approach_height
        target_approach_z = target_grasp_z + self.approach_height

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
        target_approach_closed = self.inverse_kinematics(
            target_radius, target_theta, target_approach_z, closed_gripper
        )
        target_grasp_closed = self.inverse_kinematics(
            target_radius, target_theta, target_grasp_z, closed_gripper
        )
        target_grasp_open = self.inverse_kinematics(
            target_radius, target_theta, target_grasp_z, open_gripper
        )
        target_approach_open = self.inverse_kinematics(
            target_radius, target_theta, target_approach_z, open_gripper
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
            target_approach_closed[0],
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
            target_approach_closed,
            target_grasp_closed,
            target_grasp_open,
            target_approach_open,
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
            "Ready: publish Cartesian Point(x,y,z) on /goal_poses or "
            "cylindrical Point(r,theta,z) on /cube_pose_cylindrical"
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
