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
[ ] Peripheral service node for buzzer, LED, PID tuning (buzzer/LED/PID all controllable via topics/serial today, just not behind a dedicated service node)
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
[ ] LiDAR Pointcloud publisher node
[ ] Integrate SLAM Toolbox
[ ] Integrate Nav2

## 🎮 Manual Control
[x] Joystick teleop (`controller_node.cpp`): triggers=drive, left stick=turn, SLOW/NORMAL/FAST speed modes on dpad up/down (0.5/0.75/1.0 m/s), buzzer + LED on face buttons
[ ] Turning (`angular.z`) not yet driven/verified this session -- only straight-line tested. Sanity-check yaw direction before trusting nav2/SLAM.

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
