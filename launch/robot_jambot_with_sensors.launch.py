import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_actions(_context):
    enable_control = LaunchConfiguration("enable_control")
    enable_rviz = LaunchConfiguration("enable_rviz")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("jambot_nano"), "launch", "robot_jambot.launch.py"]
            )
        )
    )

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera_node",
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(enable_control),
    )

    controller_node = Node(
        package="jambot_nano",
        executable="controller_node",
        name="controller_node",
        output="screen",
        condition=IfCondition(enable_control),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jambot_nano"), "rviz", "jambot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(enable_rviz),
    )

    actions = [robot_launch, camera_node, joy_node, controller_node, rviz_node]

    try:
        ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ydlidar_share, "launch", "ydlidar_launch.py")
            )
        )
        actions.append(lidar_launch)
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "ydlidar_ros2_driver not found. "
                    "Starting robot + camera only."
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_control",
                default_value="false",
                description="Start joy and controller_node for buzzer/LED controls",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
                description="Start RViz with jambot.rviz profile",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
