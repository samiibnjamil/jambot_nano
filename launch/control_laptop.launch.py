# Laptop-side launch file for Nav2 + optional joystick teleop
# This runs on the laptop while the RPi runs robot_jambot_with_sensors.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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

    # RViz config. Nav2/AMCL only starts publishing map->odom after a human
    # sets an initial pose, so with Fixed Frame: odom (jambot.rviz, correct
    # for plain driving and for SLAM, which self-localizes immediately) the
    # static map can never render -- there's nothing to click "2D Pose
    # Estimate" on. nav2.rviz is the same config with Fixed Frame: map, so
    # the map draws in its own frame with no transform needed.
    jambot_rviz_config = PathJoinSubstitution([jambot_dir, "rviz", "jambot.rviz"])
    nav2_rviz_config = PathJoinSubstitution([jambot_dir, "rviz", "nav2.rviz"])

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
            # RViz node: nav2.rviz (Fixed Frame: map) when Nav2 is running,
            # jambot.rviz (Fixed Frame: odom) otherwise.
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", nav2_rviz_config],
                condition=IfCondition(
                    PythonExpression(["'", enable_rviz, "' == 'true' and '", enable_nav2, "' == 'true'"])
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", jambot_rviz_config],
                condition=IfCondition(
                    PythonExpression(["'", enable_rviz, "' == 'true' and '", enable_nav2, "' == 'false'"])
                ),
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
            # Nav2 bringup (if enabled). use_rviz is always "false" here --
            # RViz is already started explicitly above (nav2.rviz), and
            # nav2_bringup's own bringup_launch.py would otherwise start a
            # second, separate rviz2 instance with its own default config.
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
                    "use_rviz": "false",
                }.items(),
                condition=IfCondition(enable_nav2),
            ),
        ]
    )
