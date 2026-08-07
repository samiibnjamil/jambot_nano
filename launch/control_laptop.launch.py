# Laptop-side launch file for Nav2 + optional joystick teleop
# This runs on the laptop while the RPi runs robot_jambot_with_sensors.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    jambot_dir = FindPackageShare("jambot_nano")

    # Launch arguments
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_joystick = LaunchConfiguration("enable_joystick")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    map_file = LaunchConfiguration("map")

    # RViz config
    rviz_config_file = PathJoinSubstitution([jambot_dir, "rviz", "jambot.rviz"])

    # Declare launch arguments
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz for visualization",
            ),
            DeclareLaunchArgument(
                "enable_joystick",
                default_value="false",
                description="Launch joy_node and controller_node for joystick teleop (requires /cmd_vel listener on RPi)",
            ),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="true",
                description="Launch Nav2 stack (map_file must be provided)",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Path to the map YAML file (required if enable_nav2:=true)",
            ),
            # RViz node
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(enable_rviz),
            ),
            # Joy node (joystick input)
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(enable_joystick),
            ),
            # Controller node (joystick to cmd_vel)
            Node(
                package="jambot_nano",
                executable="controller_node",
                name="controller_node",
                output="screen",
                condition=IfCondition(enable_joystick),
            ),
            # Nav2 bringup (if enabled)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "bringup_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "map": map_file,
                    "use_rviz": enable_rviz,
                }.items(),
                condition=IfCondition(enable_nav2),
            ),
        ]
    )
