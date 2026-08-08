import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def _build_actions(_context):
    enable_control = LaunchConfiguration("enable_control")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_camera_flip = (
        LaunchConfiguration("enable_camera_flip").perform(_context).lower() == "true"
    )
    camera_rotation_steps_cfg = LaunchConfiguration("camera_rotation_steps").perform(
        _context
    )
    try:
        camera_rotation_steps = int(camera_rotation_steps_cfg)
    except ValueError:
        camera_rotation_steps = 2

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("jambot_nano"), "launch", "robot_jambot.launch.py"]
            )
        )
    )

    if enable_camera_flip:
        camera_node = Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            output="screen",
            remappings=[("~/image_raw", "/camera_node/image_raw_orig")],
            condition=IfCondition(enable_camera),
        )
        camera_flip_node = Node(
            package="image_rotate",
            executable="image_flip_node",
            name="camera_flip_node",
            output="screen",
            remappings=[
                ("image", "/camera_node/image_raw_orig"),
                ("rotated/image", "/camera_node/image_raw"),
            ],
            parameters=[
                {
                    "rotation_steps": camera_rotation_steps,
                    "use_camera_info": False,
                    "output_frame_id": "camera_flipped",
                }
            ],
            condition=IfCondition(enable_camera),
        )
    else:
        camera_node = Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            output="screen",
            condition=IfCondition(enable_camera),
        )
        camera_flip_node = None

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
    if camera_flip_node is not None:
        actions.append(camera_flip_node)

    try:
        ydlidar_share = get_package_share_directory("ydlidar_ros2_driver")
        # Launching the driver node directly instead of including the
        # vendor's ydlidar_launch.py: that file also starts its own
        # static_transform_publisher hardcoding base_link -> laser_frame at
        # identity rotation, which permanently overrides (in every
        # listener's TF buffer, since it starts after robot_state_publisher)
        # the real chassis -> laser_frame transform this package's own URDF
        # provides. Measured live: base_link->laser_frame stayed at the
        # vendor's [0,0,0.02]/identity no matter what the URDF said, so scan
        # position/orientation fixes in jambot_description.urdf.xacro were
        # silently discarded. Dropping just that one action, keeping
        # everything else identical to the vendor launch file.
        lidar_launch = LifecycleNode(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=[os.path.join(ydlidar_share, "params", "ydlidar.yaml")],
            namespace="/",
        )
        # Delayed, not launched immediately alongside robot_launch: opening
        # the Arduino's serial port resets the board, and its setup() runs a
        # one-shot MPU6050 gyro/accel calibration that discards any round
        # where the sensor looks like it moved. Measured on real hardware,
        # the spinning LIDAR motor alone -- mechanically coupled through the
        # chassis -- is enough to blow that motion guard by 10-50x (raw gyro
        # spread ~1100-5300 vs a threshold of 100), so starting the LIDAR at
        # the same time as the hardware interface made calibration fail on
        # essentially every boot ("motion detected throughout, skipped").
        # 12s comfortably clears the measured ~5s worst-case firmware boot
        # window plus this launch's own process-startup latency before the
        # LIDAR motor spins up.
        actions.append(TimerAction(period=12.0, actions=[lidar_launch]))
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "ydlidar_ros2_driver not found. "
                    "Starting robot + camera only."
                )
            )
        )

    try:
        get_package_share_directory("slam_toolbox")
        mapper_params = os.path.join(
            get_package_share_directory("jambot_nano"),
            "config",
            "mapper_params_online_sync.yaml",
        )
        slam_node = LifecycleNode(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            namespace="",
            output="screen",
            parameters=[mapper_params],
            remappings=[("/odom", "/odometry/filtered")],
            condition=IfCondition(enable_slam),
            # slam_toolbox is a lifecycle node -- without autostart it just
            # sits in the "unconfigured" state forever (no /scan subscription,
            # no /map), since nothing else in this stack drives its lifecycle.
            autostart=True,
        )
        actions.append(slam_node)
    except PackageNotFoundError:
        actions.append(
            LogInfo(
                msg=(
                    "slam_toolbox not found. "
                    "Skipping SLAM node startup."
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
            DeclareLaunchArgument(
                "enable_slam",
                default_value="false",
                description="Start slam_toolbox and use /odometry/filtered as odom input",
            ),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="false",
                description=(
                    "Start camera_ros (and image_flip_node, if "
                    "enable_camera_flip). Off by default -- the CSI camera "
                    "pipeline is a noticeable CPU cost on the Pi, only pay "
                    "it when you're actually using the camera."
                ),
            ),
            DeclareLaunchArgument(
                "enable_camera_flip",
                default_value="true",
                description="Rotate camera image via image_flip_node (only matters if enable_camera:=true)",
            ),
            DeclareLaunchArgument(
                "camera_rotation_steps",
                default_value="2",
                description="Flip rotation steps (0..3), 2 = 180 degrees",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
