# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get RViz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("jambot_nano"), "rviz", "jambot.rviz"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("jambot_nano"), "config", "ekf.yaml"]
    )
    mapper_params = PathJoinSubstitution(
        [FindPackageShare("jambot_nano"), "config", "mapper_params_online_sync.yaml"]
    )
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_ekf = LaunchConfiguration("enable_ekf")
    enable_slam = LaunchConfiguration("enable_slam")

    # Joy node for joystick input
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

        # Controller node for receiving joystick commands
    controller_node = Node(
        package="jambot_nano",
        executable="controller_node",
        name="controller_node",
        output="screen",
    )

    # RViz node for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(enable_rviz),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
        condition=IfCondition(enable_ekf),
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[mapper_params],
        remappings=[("/odom", "/odometry/filtered")],
        condition=IfCondition(enable_slam),
    )

    nodes = [
        joy_node,
        rviz_node,
        controller_node,
        ekf_node,
        slam_node,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Start RViz with jambot.rviz profile",
            ),
            DeclareLaunchArgument(
                "enable_ekf",
                default_value="true",
                description="Fuse /jambot_base_controller/odom + /imu/data_raw into /odometry/filtered",
            ),
            DeclareLaunchArgument(
                "enable_slam",
                default_value="true",
                description="Start slam_toolbox and use /odometry/filtered as odom input",
            ),
        ]
        + nodes
    )
