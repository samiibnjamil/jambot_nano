# Nav2 Distributed Setup: RPi Robot + Laptop Navigation

This guide sets up jambot_nano for autonomous navigation using Nav2 on your laptop, while the RPi handles hardware control and SLAM mapping.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROS2 Network                             │
│  (ROS_DOMAIN_ID=0, multicast on same subnet)                    │
└─────────────────────────────────────────────────────────────────┘
        ▲                                          ▲
        │                                          │
        │                                          │
   ┌────┴─────────┐                        ┌──────┴──────────┐
   │  RPi (Robot) │                        │ Laptop (Nav2)   │
   ├───────────────┤                        ├─────────────────┤
   │               │                        │                 │
   │ Hardware      │ publishes:             │ Nav2 stack      │
   │  - Arduino    │  /jambot_base_         │  - Costmap      │
   │  - Motors     │    controller/odom     │  - Planner      │
   │  - IMU        │  /odometry/filtered    │  - Controller   │
   │  - Encoders   │  /scan                 │  - AMCL         │
   │  - LIDAR      │  /map                  │  - Waypoint     │
   │               │  /imu/data_raw         │    Follower     │
   │ SLAM Mapping  │                        │                 │
   │  - sync_slam  │ subscribes:            │ Mission Node    │
   │    _toolbox   │  /cmd_vel              │  - Sends        │
   │               │ (TwistStamped)         │    waypoints    │
   │               │                        │                 │
   └───────────────┘                        └─────────────────┘
```

## Prerequisites

### On RPi
- ROS2 Jazzy installed
- `jambot_nano` package built
- `robot_localization` (EKF) installed
- `slam_toolbox` installed
- `ydlidar_ros2_driver` built and sourced
- Network connectivity to laptop (same subnet, multicast working)

### On Laptop
- ROS2 Jazzy installed
- `nav2_bringup`, `nav2_simple_commander` installed:
  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander
  ```
- Network connectivity to RPi (same subnet, multicast working)

## Step 1: Generate the Map (One-Time)

On the **RPi**, launch the robot in SLAM mapping mode and manually drive it around the room to build the map:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash

# Launch base hardware + EKF + SLAM (no camera, no rviz)
ros2 launch jambot_nano robot_jambot_with_sensors.launch.py \
  enable_slam:=true \
  enable_camera:=false \
  enable_control:=false \
  enable_rviz:=false
```

Then, on your **laptop** (or on the Pi if you have a display), use RViz or joystick to manually drive the robot around the room. The SLAM node builds a map in real-time.

Once the room is fully mapped, save the map:

```bash
# On the laptop/machine where slam_toolbox is running
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'room'"
```

This creates `room.pgm` and `room.yaml` (the map files). Move them to:

```bash
mkdir -p ~/ros2_ws/src/jambot_nano/maps
cp room.* ~/ros2_ws/src/jambot_nano/maps/
```

## Step 2: Configure Nav2 Parameters

Edit or review `~/ros2_ws/src/jambot_nano/config/nav2_params.yaml` (if it doesn't exist, Nav2 bringup will provide defaults). Key things to check:

- `map_frame: "map"`
- `base_frame_id: "base_link"`
- `odom_frame: "odom"`
- `amcl/` parameters tuned for your room size
- `controller_server/` parameters tuned for your robot speed (0-1.0 m/s linear, 0-2.0 rad/s angular)

## Step 3: Run the Distributed System

### Terminal 1 (RPi): Start the Robot Hardware + SLAM

```bash
# On the RPi
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash

ros2 launch jambot_nano robot_jambot_with_sensors.launch.py \
  enable_slam:=true \
  enable_camera:=false \
  enable_control:=false \
  enable_rviz:=false
```

This starts:
- ROS2 control node (hardware interface)
- Joint state broadcaster
- `jambot_base_controller` (diff_drive_controller)
- EKF (sensor fusion from encoders + IMU)
- SLAM Toolbox (maps LIDAR scans, publishes `/map`)
- LIDAR driver
- Battery state publisher

The robot is now **ready to receive commands** but not moving — waiting for Nav2.

### Terminal 2 (Laptop): Start Nav2

```bash
# On the laptop (same ROS_DOMAIN_ID=0, same subnet)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0  # Must match RPi's domain ID

ros2 launch jambot_nano nav2_laptop.launch.py \
  map:=$HOME/ros2_ws/src/jambot_nano/maps/room.yaml \
  use_rviz:=true
```

This starts:
- **AMCL** (localization in the saved map using LIDAR scans)
- **Nav2 stack** (planner, controller, behavior trees)
- **RViz** for visualization
- **Waypoint Follower** (lifecycle node ready to receive waypoints)

You'll see RViz open. The map and robot position should appear.

### Terminal 3 (Laptop): Send Waypoint Mission

Once both stacks are running and RViz shows the robot localized on the map:

```bash
# On the laptop
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

ros2 run jambot_nano mission_commander.py \
  --waypoints ~/ros2_ws/src/jambot_nano/missions/example_waypoints.yaml \
  --loiter 2
```

This sends the waypoints from `example_waypoints.yaml` to the robot. The robot will:
1. Localize itself on the map (AMCL converges within a few seconds)
2. Plan paths between waypoints (Nav2 planner)
3. Execute the paths (Nav2 controller → `/cmd_vel` → hardware)
4. Pause 2 seconds at each waypoint (loiter)

Watch RViz: you'll see the planned path (green line) and the robot's actual trajectory (red line).

## Troubleshooting

### "AMCL failed to initialize" or robot doesn't localize
- Ensure the map file path is correct and readable
- Check that LIDAR scans (`/scan`) are being published and match the map
- In RViz, use the "2D Pose Estimate" tool to manually set the robot's initial pose if AMCL is lost

### "Nav2 is not active" or waypoint_follower doesn't respond
- Run `ros2 lifecycle get /waypoint_follower` — should show `active`
- If not active, run:
  ```bash
  ros2 lifecycle set /waypoint_follower configure
  ros2 lifecycle set /waypoint_follower activate
  ```

### Robot doesn't move or moves slowly
- Check `/jambot_base_controller/cmd_vel` is receiving commands:
  ```bash
  ros2 topic echo /jambot_base_controller/cmd_vel
  ```
- Verify motor speeds aren't saturating (`/diagnostics` or check `/jambot_base_controller/odom`)
- Increase `controller_server/max_vel_x` and `max_vel_theta` in nav2_params.yaml if the robot is underpowered

### Laptop and RPi don't see each other
- Confirm both have `export ROS_DOMAIN_ID=0` (or the same non-default ID)
- Check firewall allows multicast (UDP 7400-7410, typical)
- Test with:
  ```bash
  ros2 topic list  # Should show topics from both machines
  ```

## Creating Custom Waypoint Missions

Edit `~/ros2_ws/src/jambot_nano/missions/example_waypoints.yaml` or create a new file with the same format:

```yaml
waypoints:
  - name: "corner_1"
    x: 1.5
    y: 0.5
    theta_deg: 45.0
  - name: "corner_2"
    x: -1.5
    y: 2.0
    theta_deg: 180.0
```

Run the mission with:

```bash
ros2 run jambot_nano mission_commander.py --waypoints my_mission.yaml
```

## Notes

- The **RPi publishes** hardware state (`/odom`, `/scan`, `/map`, `/imu`, etc.)
- The **Laptop subscribes** to these topics and publishes `/cmd_vel` commands
- There's **no centralized server** — just ROS2 middleware connecting them
- Shutting down either the RPi or laptop gracefully stops the robot
- If the RPi loses connection, the robot stops (no cached commands)
- If the laptop loses connection, the robot **keeps executing its last command** — press Ctrl+C on the laptop to stop the waypoint mission and manually command `/cmd_vel` to zero if needed
