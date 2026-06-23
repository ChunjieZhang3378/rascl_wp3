"""Generate and execute offline minimum-jerk trajectories for Task 1."""

import csv
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = (
    "shoulder_joint",
    "upperarm_joint",
    "lowerarm_joint",
    "end_effector_joint",
)
TRAJECTORY_FILES = ("cube_1.csv", "cube_2.csv", "cube_3.csv")


def minimum_jerk_segment(start, goal, duration, sample_period):
    """Return sampled position, velocity, and acceleration for one segment."""
    if duration <= 0.0:
        raise ValueError("segment duration must be positive")
    if sample_period <= 0.0:
        raise ValueError("sample period must be positive")
    if len(start) != len(goal):
        raise ValueError("segment endpoints must have equal dimensions")

    samples = []
    sample_count = max(1, math.ceil(duration / sample_period))
    delta = [goal[index] - start[index] for index in range(len(start))]
    for sample in range(1, sample_count + 1):
        time_from_start = min(sample * sample_period, duration)
        normalized_time = time_from_start / duration
        position_scale = (
            10.0 * normalized_time ** 3
            - 15.0 * normalized_time ** 4
            + 6.0 * normalized_time ** 5
        )
        velocity_scale = (
            30.0 * normalized_time ** 2
            - 60.0 * normalized_time ** 3
            + 30.0 * normalized_time ** 4
        ) / duration
        acceleration_scale = (
            60.0 * normalized_time
            - 180.0 * normalized_time ** 2
            + 120.0 * normalized_time ** 3
        ) / (duration * duration)
        samples.append(
            (
                time_from_start,
                [start[index] + delta[index] * position_scale for index in range(len(start))],
                [delta[index] * velocity_scale for index in range(len(start))],
                [delta[index] * acceleration_scale for index in range(len(start))],
            )
        )
    return samples


class Task1Node(Node):
    """Load, generate, save, and execute the three offline trajectories."""

    def __init__(self):
        super().__init__("wp3_tsk1")
        self.declare_parameter("trajectory_input_directory", "trajectories/input")
        self.declare_parameter("trajectory_output_directory", "trajectories/output")
        self.declare_parameter("controller_mode", "CSP")
        self.declare_parameter("sample_period", 0.02)
        self.declare_parameter("simulate_only", False)
        self.declare_parameter(
            "trajectory_action", "/joint_trajectory_controller/follow_joint_trajectory"
        )

        package_share = Path(get_package_share_directory("rascl_wp3_ss26_group16"))
        input_directory = Path(
            self.get_parameter("trajectory_input_directory").value
        ).expanduser()
        output_directory = Path(
            self.get_parameter("trajectory_output_directory").value
        ).expanduser()
        self.input_directory = (
            input_directory if input_directory.is_absolute() else package_share / input_directory
        )
        self.output_directory = (
            output_directory
            if output_directory.is_absolute()
            else package_share / output_directory
        )
        self.sample_period = float(self.get_parameter("sample_period").value)
        controller_mode = str(self.get_parameter("controller_mode").value).upper()
        if controller_mode != "CSP":
            raise ValueError("Task 1 requires controller_mode='CSP'")
        if self.sample_period <= 0.0:
            raise ValueError("sample_period must be positive")

        self.simulate_only = bool(self.get_parameter("simulate_only").value)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.joint_state_publisher = None
        self.action_client = None
        if self.simulate_only:
            self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        else:
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                str(self.get_parameter("trajectory_action").value),
            )

    def load_waypoints(self, input_path):
        """Read one cube trajectory and validate its waypoint rows."""
        with input_path.open("r", newline="", encoding="utf-8") as input_file:
            reader = csv.DictReader(input_file)
            required_columns = {"duration", *JOINT_NAMES}
            if reader.fieldnames is None or not required_columns.issubset(reader.fieldnames):
                raise ValueError(
                    f"{input_path} must contain columns: "
                    f"duration,{','.join(JOINT_NAMES)}"
                )

            waypoints = []
            for line_number, row in enumerate(reader, start=2):
                try:
                    duration = float(row["duration"])
                    positions = [float(row[joint]) for joint in JOINT_NAMES]
                except (TypeError, ValueError) as error:
                    raise ValueError(
                        f"{input_path}:{line_number} contains a non-numeric value"
                    ) from error
                if not math.isfinite(duration) or not all(map(math.isfinite, positions)):
                    raise ValueError(f"{input_path}:{line_number} contains a non-finite value")
                waypoints.append((duration, positions))

        if len(waypoints) < 2:
            raise ValueError(f"{input_path} requires at least two waypoints")
        if waypoints[0][0] != 0.0:
            raise ValueError(f"{input_path} first waypoint duration must be 0")
        if any(duration <= 0.0 for duration, _ in waypoints[1:]):
            raise ValueError(f"{input_path} segment durations must be positive")
        return waypoints

    def generate_trajectory(self, waypoints):
        """Generate one continuous sampled trajectory from input waypoints."""
        trajectory = [(0.0, waypoints[0][1], [0.0] * 4, [0.0] * 4)]
        elapsed_time = 0.0
        for waypoint_index in range(1, len(waypoints)):
            duration, goal = waypoints[waypoint_index]
            start = waypoints[waypoint_index - 1][1]
            for local_time, positions, velocities, accelerations in minimum_jerk_segment(
                start, goal, duration, self.sample_period
            ):
                trajectory.append(
                    (elapsed_time + local_time, positions, velocities, accelerations)
                )
            elapsed_time += duration
        return trajectory

    def save_trajectory(self, output_path, trajectory):
        """Save generated samples for inspection and repeatability."""
        fieldnames = ["time"]
        for prefix in ("position", "velocity", "acceleration"):
            fieldnames.extend(f"{prefix}_{joint}" for joint in JOINT_NAMES)
        with output_path.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.writer(output_file)
            writer.writerow(fieldnames)
            for time_from_start, positions, velocities, accelerations in trajectory:
                writer.writerow(
                    [time_from_start, *positions, *velocities, *accelerations]
                )

    def execute_trajectory(self, trajectory, trajectory_name):
        """Send one generated trajectory and wait for its action result."""
        if self.action_client is None:
            raise RuntimeError("trajectory action client is unavailable in simulation preview mode")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for time_from_start, positions, velocities, accelerations in trajectory:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            whole_seconds = int(time_from_start)
            point.time_from_start.sec = whole_seconds
            point.time_from_start.nanosec = int(
                round((time_from_start - whole_seconds) * 1_000_000_000)
            )
            if point.time_from_start.nanosec == 1_000_000_000:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec = 0
            goal.trajectory.points.append(point)

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"controller rejected {trajectory_name}")

        self.get_logger().info(f"Executing {trajectory_name}")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError(f"controller returned no result for {trajectory_name}")
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"{trajectory_name} failed with code {result.error_code}: "
                f"{result.error_string}"
            )

    def preview_trajectory(self, trajectory, trajectory_name):
        """Publish one generated trajectory as joint states for RViz preview."""
        if self.joint_state_publisher is None:
            raise RuntimeError("joint state publisher is unavailable outside simulation preview mode")

        self.get_logger().info(f"Previewing {trajectory_name}")
        previous_time = 0.0
        for time_from_start, positions, velocities, _accelerations in trajectory:
            delay = max(0.0, time_from_start - previous_time)
            if delay > 0.0:
                rclpy.spin_once(self, timeout_sec=delay)

            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = positions
            message.velocity = velocities
            self.joint_state_publisher.publish(message)
            previous_time = time_from_start

    def run(self):
        """Generate and execute cube 1, cube 2, then cube 3."""
        generated_trajectories = []
        for filename in TRAJECTORY_FILES:
            input_path = self.input_directory / filename
            if not input_path.is_file():
                raise FileNotFoundError(f"missing Task 1 input trajectory: {input_path}")
            trajectory = self.generate_trajectory(self.load_waypoints(input_path))
            output_path = self.output_directory / filename
            self.save_trajectory(output_path, trajectory)
            generated_trajectories.append((filename, trajectory))
            self.get_logger().info(
                f"Generated {len(trajectory)} samples in {output_path}"
            )

        if self.simulate_only:
            self.get_logger().info("Simulation preview mode: publishing trajectories to RViz")
            for filename, trajectory in generated_trajectories:
                self.preview_trajectory(trajectory, filename)
            self.get_logger().info("Task 1 preview completed: cube 1, cube 2, cube 3")
            return

        self.get_logger().info("Waiting for joint trajectory controller")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("joint trajectory controller action is unavailable")
        for filename, trajectory in generated_trajectories:
            self.execute_trajectory(trajectory, filename)
        self.get_logger().info("Task 1 completed: cube 1, cube 2, cube 3")


def main(args=None):
    """Run Task 1 once and report errors through ROS logging."""
    rclpy.init(args=args)
    node = None
    try:
        node = Task1Node()
        node.run()
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
