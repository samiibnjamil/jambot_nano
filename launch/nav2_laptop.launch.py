# Copyright 2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get directories
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    jambot_dir = get_package_share_directory("jambot_nano")

    # Launch args
    use_rviz = LaunchConfiguration("use_rviz")
    map_file = LaunchConfiguration("map")

    # Declare args
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization and goal setting",
    )

    declare_map_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(jambot_dir, "maps", "room.yaml"),
        description="Full path to the map file to load",
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_rviz": use_rviz,
        }.items(),
    )

    # RViz node (only if enabled, Nav2 bringup usually includes it anyway)
    rviz_config_dir = os.path.join(jambot_dir, "rviz")
    rviz_config_file = os.path.join(rviz_config_dir, "nav2.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    # Waypoint follower (Nav2's built-in lifecycle node for following a list of poses)
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_rviz_cmd,
            declare_map_cmd,
            nav2_launch,
            rviz_node,
            waypoint_follower,
        ]
    )
