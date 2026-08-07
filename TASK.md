See `docs/known-issues.md` for root-caused bugs (shaking, wheel mapping)
before debugging anything that looks similar -- both took a while to track
down the first time.

## ✅ Hardware Setup
[x] Arduino Nano + TB6612
[x] RPi V1 Camera (CSI)
[x] YDLIDAR X3
[x] MPU-6050 IMU

## ⚙️ Control
[x] Finalize ros2_control hardware_interface
[x] Peripheral control for buzzer, LED, PID tuning -- buzzer/LED via topics (`/jambot/buzzer_mode`, `/jambot/led_rgb`), PID gains as live ROS2 parameters on `/jambot_hardware_io` (`ros2 param set pid_p/pid_d/pid_i/pid_o`), not a bespoke service: gains are tunable config, which parameters are the idiomatic ROS2 mechanism for (this replaced ROS1's dynamic_reconfigure). All three share the hardware interface's `io_node_` since only one process can own the serial port at a time.
[x] Add error handling and retry mechanism for serial communication (consecutive-error tracking + resync-retry parsing, see `ArduinoComms`)
[x] Implement watchdog timer for hardware interface (`kMaxConsecutiveSerialErrors` trips `read()`/`write()` to ERROR)
[x] Add proper command validation for motor values (RPM clamp in `set_motor_values()`)

## 🏗️ Chassis
[ ] Redesign chassis for LiDAR, Camera, Display, Dock contacts, wiring

## 🧠 AI/UX
[ ] AI Chat Assistant Node
[ ] Eye Display Node (face expressions)

## 🗺️ Mapping & SLAM
[x] Test Manufacturer's LiDAR Pointcloud publisher node
[x] EKF fusion (`robot_localization`) verified live on the ground: no NaN, filtered odom tracks raw wheel odom within ~0.1deg yaw over a straight+turn sequence. Enabled by default (`enable_ekf:=false` to opt out, e.g. once LIDAR/camera/SLAM are also loaded and CPU is tight)
[ ] LiDAR Pointcloud publisher node
[ ] Integrate SLAM Toolbox
[ ] Integrate Nav2

## 🎮 Manual Control
[x] Joystick teleop (`controller_node.cpp`): triggers=drive, left stick=turn, SLOW/NORMAL/FAST speed modes on dpad up/down (0.5/0.75/1.0 m/s), buzzer + LED on face buttons
[x] Turning (`angular.z`) verified live: correct kinematic pattern (opposite-signed wheels) and confirmed CCW/left rotation for positive angular.z, matching REP-103. Note: in-place rotation needs noticeably more torque than straight driving (normal -- wheel-scrub friction, not a bug), and low angular.z commands (~10 RPM/wheel) may not overcome floor friction from a stop on real ground even though they moved fine lifted.

## 📦 Task Manager
[ ] Autonomous task manager node
[ ] Docking control logic

## 📦 LEGO Detection
[ ] YOLO/OpenCV LEGO detection node
[ ] Detection topic and position publishing

## 🔋 Charging
[ ] Battery monitor node
[ ] Dock detection sensor integration
[ ] Dock navigation behavior

## 📐 URDF Updates
[ ] Update robot xacro:
  [ ] LiDAR mount frame
  [ ] Camera frame
  [ ] Eye display link
  [ ] Dock contact geometry

## 📊 Testing
[ ] Gazebo simulation world config
[ ] Rviz mapping test
[ ] AI Assistant CLI test
[ ] LEGO detection demo test
[ ] Docking navigation demo
[ ] Hardware interface unit tests
[ ] Serial communication stress tests
