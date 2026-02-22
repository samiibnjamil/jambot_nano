import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_actions(_context):
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

    actions = [robot_launch, camera_node]

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
    return LaunchDescription([OpaqueFunction(function=_build_actions)])
