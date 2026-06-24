"""Launch the Task 2 online pick-and-place planner."""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rascl_wp3_ss26_group16",
            executable="wp3_tsk2",
            name="wp3_tsk2",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare("rascl_wp3_ss26_group16"), "config", "task2.yaml"]
                )
            ],
        ),
    ])
