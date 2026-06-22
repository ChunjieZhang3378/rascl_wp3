from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("rascl_description")

    default_model_path = PathJoinSubstitution([
        package_share,
        "urdf",
        "rascl.urdf"
    ])
    default_rviz_config_path = PathJoinSubstitution([
        package_share,
        "rviz",
        "urdf.rviz",
    ])

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    robot_description = {"robot_description": Command(["xacro ", model])}

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=default_model_path,
            description="Absolute path to the robot URDF/Xacro file",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config_path,
            description="Absolute path to the RViz config file",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
