"""Launch description placeholder for Task 1."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rascl_wp3_ss26_group16",
            executable="wp3_tsk1",
            name="wp3_tsk1",
            output="screen",
        ),
    ])
