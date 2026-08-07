# jambot_nano known issues and how they were found

This file exists so the next debugging session (human or AI agent) doesn't have
to re-derive these from scratch. Each entry: symptom you'd actually observe,
root cause, exact fix location, and how it was diagnosed -- so a similar bug
in a similar spot can be found the same way.

---

## "Robot shakes/oscillates violently when driven through ROS2, but raw serial `m`/`t` commands are smooth"

**Symptom:** commanding a steady velocity via `/jambot_base_controller/cmd_vel`
causes the wheels to slam full-power forward then full-power reverse several
times a second. The same target RPM sent directly over serial (bypassing
ROS2 entirely) converges cleanly with no oscillation.

**Root cause:** `on_activate()` in
`src/hardware_interface/jambot_system.cpp` pushes `pid_p`/`pid_d`/`pid_i`/
`pid_o` from the URDF to the firmware's `p` (live PID tuning) command on
every launch. At one point those URDF values were `20/12/0/50` -- carried
over from a different hardware interface's PID scaling -- while this
firmware is tuned for `Kp=0.6/Kd=0.001/Ki=1.7/Ko=1.0`. `Ko` multiplies the
PID output directly in the firmware's `applyMotorSpeed()` call, so `Ko=50`
alone means any nonzero PID output instantly saturates to +/-255, which
turns the loop into a full-scale limit cycle.

This was invisible for a long time because the firmware's `p` handler used
to be a hardcoded no-op ("keep PID fixed to non-beta tuned values") -- the
URDF could say anything and it wouldn't matter. The bug only fires once `p`
does something, so re-enabling live tuning (for convenience, to allow
sweeping gains without reflashing) without first checking what the URDF was
already sending is exactly what triggers this class of bug.

**Where to look if this class of bug recurs:**
- `urdf/jambot.ros2_control.xacro` -- the `pid_p`/`pid_d`/`pid_i`/`pid_o`
  params. These **must** match whatever the firmware's `speedPID()` /
  `PID motor1PID(...)` construction is actually tuned to (see
  `Arduino/jambot_beta/jambot_beta.ino`, the `Kp`/`Ki`/`Kd`/`Ko` globals near
  the top). If you change the firmware's tuned gains, update this file too,
  or delete the params entirely to fall back to the firmware's own defaults
  (the `on_activate()` code only pushes gains if `pid_p > 0`).
- `include/jambot_nano/arduino_comms.hpp` / `.cpp`,
  `ArduinoComms::set_pid_values()` -- this used to take `int`, which
  silently truncated real fractional gains (`0.6`, `0.001`, `1.7`) to
  `0`/`0`/`1`. Any future "why did my gain change do nothing / do something
  wildly different" question in this function should check the parameter
  type first.

**How it was actually diagnosed** (useful pattern for a similar "works
standalone, breaks under the real system" bug): built a raw-Python script
(`mimic_ros2.py` pattern -- send `t` then `m`, `\n\r` line endings, 20Hz,
same idle-then-ramp command sequence ROS2 sends) that reproduced ROS2's
*exact* byte-level behavior without ROS2 itself. When that stayed clean but
the real ROS2 launch didn't, the difference had to be something ROS2 does
beyond the bytes on the wire -- which pointed at `on_activate()`'s one-time
setup calls, not the steady-state command loop. Sending the URDF's `p`
values manually over raw serial reproduced the exact shaking signature in
isolation, confirming it before touching any code.

**Extra diagnostic added while chasing this** (kept, not removed): firmware
telemetry now appends `tgt`/`pwm`/`vf`/`rd`/`pid` fields after the original
10-field `t` response (see `sendTelemetry()` in the `.ino` and
`parse_ext`-style parsing on the host side) -- the host parser only reads
the first 10 fields, so this is backward compatible. A `verbose_telemetry`
hardware parameter (default `false`) gates per-cycle 20Hz logging of this
data in `arduino_comms.cpp`/`jambot_system.cpp` -- set it to `true` in the
URDF when you need to see live PID/PWM state without reflashing, and back to
`false` when done (leaving it on is pure log spam at 20Hz).

---

## "Odometry reports the wrong direction / wheels appear to move opposite ways during a straight drive"

**Symptom:** commanding a symmetric straight-forward `cmd_vel` (equal
left/right wheel speed) produces `/joint_states` positions for `wheel_l` and
`wheel_r` that diverge -- one trending positive, one trending negative --
even though both wheels are physically spinning the same way. Motion itself
looks fine when driven directly; only the *reported* state (and therefore
`/odom`, TF, EKF, SLAM) is wrong. This is easy to miss with straight-line-only
testing, since a screwed up per-wheel sign can partially cancel out in the
aggregate numbers.

**Root cause:** in `read()`
(`src/hardware_interface/jambot_system.cpp`), the encoder-to-wheel
assignment was:
```cpp
wheel_r_.enc_ = -enc_1;
wheel_l_.enc_ = -enc_2;
```
but the command path (`write()`, and `ArduinoComms::set_motor_values()`)
sends `wheel_l_.cmd_` to firmware motor1 and `wheel_r_.cmd_` to motor2,
unswapped and unnegated. So `wheel_l_` was being *commanded* through motor1
but had its *position/velocity reported* from motor2's encoder -- reading
back the wrong physical wheel's motion, with the sign inverted on top.

**How this was resolved, and why not to just "swap it back" blindly:**
there were two structurally different possible explanations, and they call
for opposite fixes:
1. A real bug needing the write and read sides reconciled to agree, or
2. An intentional, already-correct fix -- if the *encoder* cables were
   physically cross-wired at some point (independent of the motor-drive
   wiring), swapping only the read side would have been exactly right, and
   "fixing" it back would reintroduce the original problem.

The only way to tell these apart is to look at the actual hardware, which
software can't infer. This was resolved with a live isolation test: command
`m 60 0` (motor1 only) and `m 0 60` (motor2 only) on the real robot,
lifted, and directly observe which physical wheel spins and which direction
it looks like ("forward" or "backward"). Result: motor1 is physically the
left wheel, motor2 is physically the right wheel, and a positive command
spins each one forward -- confirmed a second, independent way by the fact
that the closed-loop PID had converged cleanly to every positive target all
session (a wrong-sign feedback loop is unstable and would run away, not
converge). That fixed the ambiguity: the motor-drive wiring was always
correct; only the encoder-read assignment was wrong. Fix applied:
```cpp
wheel_l_.enc_ = enc_1;
wheel_r_.enc_ = enc_2;
```
No swap, no negation -- each wheel's reported position now comes straight
from its own motor's own encoder.

**If a similar wheel-identity question comes up again** (new wiring, new
robot, a wheel reversed after reassembly): reach for the same isolation
test before touching any code. `m <rpm1> <rpm2>` accepts either argument as
zero, so one motor at a time is trivial to isolate; watch the physical
wheel, don't infer it from odometry math, since that's exactly the
assumption that was wrong here.

**Verification:** after the fix, driving straight (`0.2 m/s`, both wheels
equal) produced `wheel_l`/`wheel_r` positions climbing together and staying
matched (within normal encoder noise) for the whole run -- not diverging in
sign as before.

**Update:** turning was verified in a later session -- positive `angular.z`
produces the correct CCW/left rotation per REP-103, both on the ground and
lifted. In-place rotation needs noticeably more torque than straight driving
(wheel-scrub friction against the floor, not a bug); low `angular.z` (~10
RPM/wheel) may not overcome floor friction from a dead stop even though the
same command moves the wheels fine lifted.

---

## "`/map` never appears / slam_toolbox never subscribes to `/scan`, even though the node starts with no errors"

**Symptom:** `robot_jambot_with_sensors.launch.py enable_slam:=true` starts
`sync_slam_toolbox_node` cleanly (no errors in the log), but `ros2 node info
/slam_toolbox` shows it's only publishing/subscribing to
`/parameter_events` and `/rosout` -- no `/scan`, no `/map`. `ros2 lifecycle
get /slam_toolbox` reports `unconfigured`.

**Root cause:** `sync_slam_toolbox_node` is a lifecycle node
(`rclcpp_lifecycle::LifecycleNode`). Launching it with a plain
`launch_ros.actions.Node(...)` starts the process but leaves it in the
`unconfigured` state forever -- it never subscribes to anything or does any
work until something explicitly drives it through
`configure` -> `activate`. Nothing in this stack (no `nav2_lifecycle_manager`,
no manual transition) was doing that, so the node just sat there looking
healthy while doing nothing.

**Fix:** in `launch/robot_jambot_with_sensors.launch.py`, use
`launch_ros.actions.LifecycleNode` instead of `Node` for `slam_node`, with
`autostart=True`. This is a built-in `LifecycleNode` feature (see
`launch_ros/actions/lifecycle_node.py`): when `autostart` is true, it emits
the `configure` + `activate` transitions itself right after the process
starts -- no manual `EmitEvent`/`OnStateTransition` wiring or lifecycle
manager needed for a single node like this.

**How to tell if a new lifecycle-managed node (nav2, another slam_toolbox
mode, etc.) has this same problem:** `ros2 node info <name>` showing only
the generic `/parameter_events`/`/rosout`/`/*/transition_event` topics with
none of the topics that node's own docs promise is the tell. Confirm with
`ros2 lifecycle get <name>` -- `unconfigured` means it needs an
autostart/lifecycle-manager fix like this one, not a params or wiring bug.

---

## "Custom workspace packages (jambot_nano, camera_ros, ...) aren't found from a fresh terminal"

**Symptom:** `ros2 launch jambot_nano ...` (or any `ros2 pkg prefix
jambot_nano`) fails with "package not found" in a brand new terminal, but
works fine if you `cd ~/ros2_ws` first and re-source manually.

**Root cause:** `~/.bashrc` sourced the workspace with a *relative* path
(`source install/setup.bash`), which only resolves if the shell's cwd
happens to already be `~/ros2_ws` when `.bashrc` runs -- not true for a
normal login shell (starts at `$HOME`). Confirmed by running a real login
shell (`bash -lic ...`) and seeing `install/setup.bash: No such file or
directory` plus `camera_ros`/`jambot_nano` both unresolvable. Separately,
`ydlidar_ros2_driver` lives in a completely different workspace
(`~/ydlidar_ros2_ws`) that was never sourced anywhere, so LIDAR auto-detect
in `robot_jambot_with_sensors.launch.py` (a `try/except PackageNotFoundError`
around `get_package_share_directory("ydlidar_ros2_driver")`) always took the
"not found" fallback in practice, even though the driver was built and
worked when its workspace was sourced by hand.

**Fix:** `~/.bashrc` now sources both workspaces with absolute, `-f`-guarded
paths:
```bash
source /opt/ros/jazzy/setup.bash
if [ -f /home/sami/ros2_ws/install/setup.bash ]; then
    source /home/sami/ros2_ws/install/setup.bash
fi
if [ -f /home/sami/ydlidar_ros2_ws/install/setup.bash ]; then
    source /home/sami/ydlidar_ros2_ws/install/setup.bash
fi
```
If a package that's definitely built still isn't found, check
`echo $AMENT_PREFIX_PATH | tr ':' '\n'` in a *fresh* shell (not one that's
been manually `source`d already this session) before assuming the code is
wrong.
