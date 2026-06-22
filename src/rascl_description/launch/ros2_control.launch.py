from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare("rascl_description"), "urdf", "rascl.urdf"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("rascl_description"), "config", "controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_joint_trajectory_controller_spawner = TimerAction(
        period=5.0,
        actions=[joint_trajectory_controller_spawner],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            controller_manager,
            delayed_joint_state_broadcaster_spawner,
            delayed_joint_trajectory_controller_spawner,
        ]
    )
