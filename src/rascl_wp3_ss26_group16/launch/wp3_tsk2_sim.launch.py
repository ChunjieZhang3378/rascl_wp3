"""Launch the Task 2 online planner in RViz preview mode."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [FindPackageShare("rascl_description"), "urdf", "rascl.urdf"]
                ),
            ]
        )
    }
    task_config = PathJoinSubstitution(
        [FindPackageShare("rascl_wp3_ss26_group16"), "config", "task2.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rascl_description"), "rviz", "urdf.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
            Node(
                package="rascl_wp3_ss26_group16",
                executable="wp3_tsk2",
                name="wp3_tsk2",
                parameters=[task_config, {"simulate_only": True}],
                output="screen",
            ),
        ]
    )
