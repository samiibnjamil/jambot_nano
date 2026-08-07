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

**Not yet verified:** turning (`angular.z` != 0) was never driven this
session, only straight-line `linear.x`. The yaw-direction sign convention
(does positive `angular.z` turn the way TF/REP-103 expects) should be
sanity-checked with an actual turn before trusting SLAM/nav2 on this
mapping.
