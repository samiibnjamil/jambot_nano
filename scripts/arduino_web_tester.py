#!/usr/bin/env python3
"""Interactive web-based test console for the jambot Arduino firmware.

Serves a local web UI (default http://0.0.0.0:8765) for bench-testing the
firmware over serial: connect/disconnect, read battery/encoders/IMU, drive
each motor (raw PWM or closed-loop RPM), tune and graph the PID loop, drive
with a browser-connected gamepad, and compile+upload a sketch straight from
the page.

Only depends on pyserial (already used by the ROS2 hardware interface) --
the HTTP side is stdlib http.server, so it runs on a bare Raspberry Pi OS
Python with no pip installs. Compile/upload uses the system arduino-builder
+ avrdude (apt: arduino-core-avr, avrdude) rather than arduino-cli, since
that's what's available on this image.

Usage:
  python3 arduino_web_tester.py [--http-port 8765] [--port /dev/ttyUSB0] [--baud 115200]

If --port is omitted, pick a port from the Connect panel in the browser.

IMPORTANT: only one process can safely own the serial port at a time. Stop
the ROS2 hardware interface (or any other node talking to the Arduino)
before using this tool -- two processes reading/writing the same UART will
corrupt both sides' commands.

This tool has no authentication and binds 0.0.0.0 by default so it's
reachable from another machine on the LAN (e.g. testing from a laptop while
it runs on the robot). Anyone on the same network can drive the motors or
reflash firmware through it -- only run it on a trusted network.
"""
import argparse
import json
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import serial
import serial.tools.list_ports

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TwistStamped
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# Launch files this tool is allowed to run, and the launch arguments each
# accepts -- deliberately whitelisted rather than accepting an arbitrary
# launch file or arg name, since this spawns a real subprocess.
ROS2_LAUNCH_FILES = {
    "robot_jambot.launch.py": {"enable_ekf"},
    "robot_jambot_with_sensors.launch.py": {
        "enable_control", "enable_rviz", "enable_slam",
        "enable_camera", "enable_camera_flip", "camera_rotation_steps",
    },
}

# DiffDriveController's command topic (jambot_controllers.yaml has
# use_stamped_vel: true, so it's TwistStamped, not the older Twist).
CMD_VEL_TOPIC = "/jambot_base_controller/cmd_vel"

REPO_ROOT = Path(__file__).resolve().parent.parent
ARDUINO_ROOT = REPO_ROOT / "Arduino"
STATIC_DIR = Path(__file__).resolve().parent / "arduino_web_tester_static"

ARDUINO_HARDWARE_DIR = "/usr/share/arduino/hardware"
ARDUINO_TOOLS_DIR = "/usr/share/arduino/hardware/tools"

# Whitelisted board options -- deliberately narrow rather than accepting an
# arbitrary FQBN, since each entry also needs a matching avrdude mcu/speed.
FQBN_OPTIONS = {
    "arduino:avr:nano:cpu=atmega328old": {
        "label": "Arduino Nano (Old Bootloader)", "mcu": "atmega328p",
        "programmer": "arduino", "baud": 57600,
    },
    "arduino:avr:nano:cpu=atmega328": {
        "label": "Arduino Nano (New Bootloader / atmega328)", "mcu": "atmega328p",
        "programmer": "arduino", "baud": 115200,
    },
    "arduino:avr:uno": {
        "label": "Arduino Uno", "mcu": "atmega328p",
        "programmer": "arduino", "baud": 115200,
    },
}


def list_sketches():
    """.ino files under Arduino/, excluding vendored libraries/examples."""
    sketches = []
    if ARDUINO_ROOT.is_dir():
        for ino in sorted(ARDUINO_ROOT.glob("*/*.ino")):
            sketches.append(str(ino.relative_to(REPO_ROOT)))
    return sketches


class ArduinoLink:
    """Serializes all serial access behind a lock. The firmware speaks a
    strict one-command/one-reply protocol over a single UART; concurrent
    writes from the telemetry poller, manual commands, and the joystick
    loop would otherwise corrupt each other's commands."""

    def __init__(self):
        self._lock = threading.Lock()
        self._ser = None
        self.port = None
        self.baud = None
        self.boot_log = []

    @property
    def connected(self):
        return self._ser is not None and self._ser.is_open

    def connect(self, port, baud=115200, boot_wait=3.0):
        with self._lock:
            self._close_locked()
            self._ser = serial.Serial(port, baud, timeout=0.2)
            self.port = port
            self.baud = baud
            self.boot_log = []
            # Opening the port toggles DTR on most boards, resetting the
            # Arduino. Drain whatever it prints during setup() (MPU status,
            # calibration result, "ready") instead of just sleeping past
            # it -- that's genuinely useful for confirming boot health.
            deadline = time.time() + boot_wait
            while time.time() < deadline:
                line = self._ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    self.boot_log.append(line)
            self._ser.timeout = 1.0

    def disconnect(self):
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.write(b"m 0 0\n\r")
                    self._ser.flush()
                except Exception:
                    pass
            self._close_locked()

    def _close_locked(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def send(self, cmd, timeout=0.3):
        # The firmware replies in low single-digit ms for every command (see
        # jambot_beta.ino's command switch) -- 0.3s is generous headroom, not
        # a tight cutoff. Keeping this short matters: a stuck read holds the
        # lock for the full timeout, delaying everything queued behind it,
        # including a STOP command.
        with self._lock:
            return self.send_locked(cmd, timeout)

    def send_locked(self, cmd, timeout=0.3):
        """Same as send(), but for a caller that already holds the lock via
        `with link._lock:` -- used by the autotuner, which is a whole
        control loop that must own the serial link exclusively for its
        entire run, not just one command at a time."""
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("not connected")
        self._ser.timeout = timeout
        self._ser.reset_input_buffer()
        self._ser.write((cmd.strip() + "\n\r").encode("ascii"))
        self._ser.flush()
        line = self._ser.readline().decode("ascii", errors="replace").strip()
        if not line:
            raise RuntimeError("no reply (timeout)")
        return line


link = ArduinoLink()


class LidarBridge:
    """Subscribes to the YDLidar driver's /scan (sensor_msgs/LaserScan,
    published with SensorDataQoS -- see ydlidar_ros2_driver_node.cpp) on a
    background rclpy thread, independent of the Arduino serial connection.
    A no-op if rclpy isn't importable in this environment (e.g. the ROS2
    workspace isn't sourced) or the driver node isn't running."""

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None

    def start(self):
        if not ROS_AVAILABLE:
            return
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        rclpy.init(args=None)
        node = rclpy.create_node("jambot_web_tester_lidar_bridge")
        node.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        rclpy.spin(node)

    def _on_scan(self, msg):
        # inf/nan are normal in LaserScan (out-of-range / invalid readings)
        # but aren't valid JSON tokens the browser's JSON.parse will accept,
        # so they're converted to null here rather than at serialize time.
        ranges = [r if math.isfinite(r) else None for r in msg.ranges]
        with self._lock:
            self._latest = {
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": ranges,
                "stamp": time.time(),
            }

    def latest(self):
        with self._lock:
            return dict(self._latest) if self._latest else None


lidar_bridge = LidarBridge()


class Ros2LaunchManager:
    """Runs `ros2 launch jambot_nano <file> <args>` as a subprocess in its
    own process group, so Stop can SIGINT the whole group the way Ctrl+C
    would -- a plain kill of just the parent leaves ros2_control_node and
    the spawner processes running as orphans."""

    def __init__(self):
        self._lock = threading.Lock()
        self._proc = None
        self._launch_file = None
        self._log_lines = []
        self._log_lock = threading.Lock()

    @property
    def running(self):
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def start(self, launch_file, extra_args):
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                raise RuntimeError("ROS2 stack is already running -- stop it first")
            if launch_file not in ROS2_LAUNCH_FILES:
                raise ValueError("unknown launch file")
            allowed = ROS2_LAUNCH_FILES[launch_file]
            for arg in extra_args:
                m = re.match(r"^([a-zA-Z_][a-zA-Z0-9_]*):=(.*)$", arg)
                if not m or m.group(1) not in allowed:
                    raise ValueError(f"bad or unsupported arg for {launch_file}: {arg!r}")

            # ros2_control needs exclusive access to the Arduino; release
            # this tool's own direct connection first.
            if link.connected:
                link.disconnect()

            with self._log_lock:
                self._log_lines = []
            cmd = ["ros2", "launch", "jambot_nano", launch_file] + list(extra_args)
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                start_new_session=True, cwd=str(REPO_ROOT))
            self._launch_file = launch_file
            threading.Thread(target=self._pump_output, args=(self._proc,), daemon=True).start()

    def _pump_output(self, proc):
        for line in proc.stdout:
            with self._log_lock:
                self._log_lines.append(line.rstrip())
        with self._log_lock:
            self._log_lines.append(f"[process exited, code {proc.returncode}]")

    def stop(self):
        with self._lock:
            proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        os.killpg(pgid, signal.SIGINT)  # same as Ctrl+C: lets nodes shut down cleanly
        try:
            proc.wait(timeout=6.0)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(pgid, signal.SIGTERM)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)

    def status(self):
        return {"running": self.running, "launch_file": self._launch_file}

    def log_since(self, index):
        with self._log_lock:
            return list(self._log_lines[index:]), len(self._log_lines)


ros2_manager = Ros2LaunchManager()


class CmdVelBridge:
    """Publishes TwistStamped on the diff_drive_controller's command topic
    so the web UI's joystick/motor controls can drive the robot through
    the real ROS2 stack instead of the raw Arduino serial protocol, once
    ROS2 mode is running. Lazily creates its node -- pure publishing needs
    no spin(), so it doesn't compete with LidarBridge's spin thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self._node = None
        self._pub = None

    def _ensure_node(self):
        if not ROS_AVAILABLE:
            raise RuntimeError("rclpy not available in this environment")
        with self._lock:
            if self._node is None:
                if not rclpy.ok():
                    rclpy.init(args=None)
                self._node = rclpy.create_node("jambot_web_tester_cmd_vel_bridge")
                self._pub = self._node.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

    def publish(self, linear_x, angular_z):
        self._ensure_node()
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self._pub.publish(msg)


cmd_vel_bridge = CmdVelBridge()


class PidAutotuner:
    """Relay-feedback PID autotuner (Astrom & Hagglund, 1984) -- the
    describing-function method underlying essentially every commercial
    auto-tuning PID controller, still the standard reference technique for
    closed-loop identification without needing an existing stable
    controller. Toggling the plant's output between a fixed +d/-d around a
    setpoint drives a limit-cycle oscillation whose period (Pu) and
    amplitude (a) give the ultimate gain Ku = 4d/(pi*a) directly, with no
    process model needed.

    Runs open-loop (PID disabled) via raw PWM ('o'), reading RPM from raw
    encoder deltas host-side -- see the comment on why 't' can't be used
    here. Holds the ArduinoLink lock for its entire run: this is one
    control loop, not a series of independent requests, and it must not
    interleave with any other command while relaying.

    Two gain sets are computed from the identified Ku/Pu:
      - Ziegler-Nichols (1942 closed-loop rule): fast but often
        under-damped/oscillatory in practice.
      - Tyreus-Luyben (1992): developed specifically to detune ZN's
        aggressiveness for relay-identified Ku/Pu, generally the safer
        default on real hardware.
    Both are reported; the caller (web UI) picks which to apply.
    """

    MAX_RELAY_PWM = 150
    MAX_DURATION_S = 30.0
    MAX_RPM_SAFETY = 500.0
    CPR = 404.0  # must match jambot_beta.ino's #define CPR

    def __init__(self):
        self._run_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._state = {"running": False, "phase": "idle", "log": [], "result": None, "error": None, "live_tail": []}

    def _log(self, msg):
        with self._state_lock:
            self._state["log"].append(msg)

    def _set(self, **kwargs):
        with self._state_lock:
            self._state.update(kwargs)

    @property
    def running(self):
        with self._state_lock:
            return self._state["running"]

    def status(self):
        with self._state_lock:
            return {k: v for k, v in self._state.items() if k != "log"}

    def log_since(self, index):
        with self._state_lock:
            lines = list(self._state["log"][index:])
            total = len(self._state["log"])
        return lines, total

    def start(self, setpoint_rpm, relay_pwm, duration_s):
        if self.running:
            raise RuntimeError("autotune already running")
        if not link.connected:
            raise RuntimeError("not connected to the Arduino")
        if ros2_manager.running:
            raise RuntimeError("ROS2 mode owns the serial port -- stop it first")
        setpoint_rpm = float(setpoint_rpm)
        relay_pwm = min(abs(float(relay_pwm)), self.MAX_RELAY_PWM)
        duration_s = min(max(float(duration_s), 3.0), self.MAX_DURATION_S)
        if relay_pwm < 10:
            raise ValueError("relay_pwm too small to reliably move the motors")

        self._stop_event.clear()
        with self._state_lock:
            self._state.update(running=True, phase="starting", result=None, error=None, log=[], live_tail=[])
        threading.Thread(
            target=self._run, args=(setpoint_rpm, relay_pwm, duration_s), daemon=True).start()

    def stop(self):
        self._stop_event.set()

    def _run(self, setpoint_rpm, relay_pwm, duration_s):
        FILTER_TAU_S = 0.05
        hysteresis_rpm = max(2.0, setpoint_rpm * 0.02)
        samples = []   # (elapsed_s, filt_rpm1, filt_rpm2, pwm)
        switches = []  # (elapsed_s, new_relay_sign)

        try:
            self._log(
                f"Starting relay autotune: setpoint={setpoint_rpm:.0f} rpm, "
                f"relay=+/-{relay_pwm:.0f} pwm, up to {duration_s:.1f}s")
            with link._lock:
                link.send_locked("o 0 0")
                link.send_locked("r")  # zero encoder baseline

                last_enc1 = last_enc2 = 0
                last_t = time.monotonic()
                filt1 = filt2 = 0.0
                relay_sign = 1
                start_t = last_t
                self._set(phase="relaying")

                while True:
                    now = time.monotonic()
                    elapsed = now - start_t
                    if self._stop_event.is_set():
                        self._log("Aborted by user.")
                        break
                    if elapsed > duration_s:
                        self._log(f"Reached time limit ({duration_s:.1f}s).")
                        break

                    try:
                        reply = link.send_locked("e")
                    except RuntimeError:
                        continue  # dropped/garbled reply -- try again next tick
                    parts = reply.split()
                    if len(parts) < 3 or parts[0] != "e":
                        continue
                    try:
                        enc1, enc2 = int(parts[1]), int(parts[2])
                    except ValueError:
                        continue

                    dt = max(now - last_t, 1e-3)
                    rpm1 = ((enc1 - last_enc1) * 60.0) / (self.CPR * dt)
                    rpm2 = ((enc2 - last_enc2) * 60.0) / (self.CPR * dt)
                    alpha = dt / (FILTER_TAU_S + dt)
                    filt1 += alpha * (rpm1 - filt1)
                    filt2 += alpha * (rpm2 - filt2)
                    last_enc1, last_enc2, last_t = enc1, enc2, now

                    avg_rpm = (filt1 + filt2) / 2.0
                    if abs(avg_rpm) > self.MAX_RPM_SAFETY:
                        self._log(f"SAFETY ABORT: measured {avg_rpm:.0f} rpm exceeds the {self.MAX_RPM_SAFETY:.0f} rpm limit.")
                        self._set(error="safety limit exceeded -- aborted")
                        break

                    error = setpoint_rpm - avg_rpm
                    if relay_sign > 0 and error < -hysteresis_rpm:
                        relay_sign = -1
                        switches.append((elapsed, relay_sign))
                    elif relay_sign < 0 and error > hysteresis_rpm:
                        relay_sign = 1
                        switches.append((elapsed, relay_sign))

                    pwm = int(relay_sign * relay_pwm)
                    try:
                        link.send_locked(f"o {pwm} {pwm}")
                    except RuntimeError:
                        pass
                    samples.append((elapsed, filt1, filt2, pwm))
                    with self._state_lock:
                        self._state["live_tail"] = samples[-200:]

                link.send_locked("o 0 0")
        except Exception as exc:
            self._log(f"ERROR: {exc}")
            self._set(error=str(exc))
        finally:
            try:
                link.send("m 0 0", timeout=0.3)
            except Exception:
                pass
            result = self._analyze(samples, switches, relay_pwm)
            if result:
                # End-to-end means ending with the PID actually set, not
                # just numbers on screen -- apply the safer (Tyreus-Luyben)
                # set live immediately. This only changes RAM (lost on
                # reset), so it's a reversible, low-risk auto-action; a
                # permanent flash still requires an explicit user click.
                tl = result["tyreus_luyben"]
                try:
                    reply = link.send(
                        f"p {tl['kp']:.5f} {tl['kd']:.6f} {tl['ki']:.5f} {result['ko']:.2f}", timeout=1.0)
                    self._log(f"Applied Tyreus-Luyben gains live: {reply}")
                    result["applied_live"] = True
                except Exception as exc:
                    self._log(f"Identified gains but could not apply them live: {exc}")
                    result["applied_live"] = False
            self._set(running=False, phase="done", result=result)

    def _analyze(self, samples, switches, relay_pwm):
        if len(switches) < 5:
            self._log(
                f"Only {len(switches)} relay switches detected (need >=5). "
                "Try a larger relay amplitude, a lower setpoint, or check the motors spin freely.")
            return None

        steady = switches[2:]  # drop the first 2 switches as startup transient
        if len(steady) < 3:
            self._log("Not enough steady-state oscillation after discarding the startup transient.")
            return None

        periods = [steady[i + 2][0] - steady[i][0] for i in range(0, len(steady) - 2, 2)]
        if not periods:
            self._log("Could not measure a full oscillation period.")
            return None
        Pu = sum(periods) / len(periods)

        amps = []
        for i in range(len(steady) - 1):
            t0, t1 = steady[i][0], steady[i + 1][0]
            window = [(s[1] + s[2]) / 2.0 for s in samples if t0 <= s[0] < t1]
            if len(window) >= 2:
                amps.append((max(window) - min(window)) / 2.0)
        if not amps:
            self._log("Could not measure oscillation amplitude.")
            return None
        a = sum(amps) / len(amps)
        if a <= 0.01:
            self._log("Measured oscillation amplitude is ~0 -- motors may not be spinning freely.")
            return None

        Ku = (4.0 * relay_pwm) / (math.pi * a)
        zn = {"kp": 0.6 * Ku, "ki": 1.2 * Ku / Pu, "kd": 0.075 * Ku * Pu}
        tl_kp = Ku / 3.2
        tl = {"kp": tl_kp, "ki": tl_kp / (2.2 * Pu), "kd": tl_kp * (Pu / 6.3)}

        self._log(f"Identified: Ku={Ku:.4f} pwm/rpm, Pu={Pu:.3f}s (from {len(periods)} periods, {len(amps)} half-cycles)")
        self._log(f"Ziegler-Nichols: Kp={zn['kp']:.4f} Ki={zn['ki']:.4f} Kd={zn['kd']:.5f}")
        self._log(f"Tyreus-Luyben:   Kp={tl['kp']:.4f} Ki={tl['ki']:.4f} Kd={tl['kd']:.5f}  (recommended default)")
        self._log("Recommended Ko = 1.0 -- the relay test drives raw PWM directly, bypassing Ko entirely.")

        return {
            "ku": Ku, "pu": Pu, "amplitude": a, "num_periods": len(periods),
            "ziegler_nichols": zn, "tyreus_luyben": tl, "ko": 1.0,
            "samples": samples,
        }


autotuner = PidAutotuner()


def parse_telemetry(line):
    """Parses the firmware's 't' reply:
    "t <e1> <e2> <imuOk> <ax> <ay> <az> <gx> <gy> <gz> <batt> | tgt <t1> <t2> pwm <p1> <p2> vf <v1> <v2> rd <r> pid <p>"
    """
    if not line.startswith("t "):
        return None
    parts = line.split("|")
    head = parts[0].split()
    try:
        result = {
            "enc1": int(head[1]), "enc2": int(head[2]),
            "imu_ok": head[3] == "1",
            "ax": float(head[4]), "ay": float(head[5]), "az": float(head[6]),
            "gx": float(head[7]), "gy": float(head[8]), "gz": float(head[9]),
            "battery": float(head[10]),
        }
    except (IndexError, ValueError):
        return None
    if len(parts) > 1:
        tail = parts[1].split()
        try:
            result.update({
                "target_rpm1": float(tail[1]), "target_rpm2": float(tail[2]),
                "pwm1": float(tail[4]), "pwm2": float(tail[5]),
                "filtered_rpm1": float(tail[7]), "filtered_rpm2": float(tail[8]),
                "ramping": tail[10] == "1",
                "pid_enabled": tail[12] == "1",
            })
        except (IndexError, ValueError):
            pass
    return result


# Matches "double Kp = <x>, Ki = <x>, Kd = <x>, Ko = <x>;" in the sketch.
# NOTE: the *declaration* order is Kp, Ki, Kd, Ko, but the serial 'p'
# command's argument order is kp, kd, ki, ko (matching
# ArduinoComms::set_pid_values on the ROS2 side) -- these differ, and
# swapping them here would silently swap the saved D and I gains.
PID_DEFAULTS_PATTERN = re.compile(
    r"double\s+Kp\s*=\s*[^,]+,\s*Ki\s*=\s*[^,]+,\s*Kd\s*=\s*[^,]+,\s*Ko\s*=\s*[^;]+;"
)


def save_pid_defaults(sketch_rel, kp, kd, ki, ko):
    """Rewrites the sketch's hardcoded PID default constants so a fresh
    flash boots with the tuned values instead of the old baked-in ones.
    Live 'p' commands only change RAM and are lost on reset -- this is
    what makes a tuning session stick."""
    if sketch_rel not in list_sketches():
        raise ValueError("unknown sketch")
    sketch_path = REPO_ROOT / sketch_rel
    text = sketch_path.read_text()
    new_line = f"double Kp = {kp}, Ki = {ki}, Kd = {kd}, Ko = {ko};"
    new_text, count = PID_DEFAULTS_PATTERN.subn(new_line, text, count=1)
    if count == 0:
        raise ValueError(
            "could not find the PID defaults line in this sketch "
            "(expected 'double Kp = ..., Ki = ..., Kd = ..., Ko = ...;')")
    sketch_path.write_text(new_text)
    return new_line


def run_upload(sketch_rel, fqbn, port):
    """Generator yielding compile/upload log lines as they happen."""
    if sketch_rel not in list_sketches():
        yield "ERROR: unknown sketch"
        return
    if fqbn not in FQBN_OPTIONS:
        yield "ERROR: unknown board option"
        return
    board = FQBN_OPTIONS[fqbn]
    sketch_path = REPO_ROOT / sketch_rel

    was_connected = link.connected and link.port == port
    if link.connected:
        yield "Releasing serial port for upload..."
        link.disconnect()

    build_dir = tempfile.mkdtemp(prefix="jambot_build_")
    try:
        yield f"Compiling {sketch_rel} for {fqbn} ..."
        compile_cmd = [
            "arduino-builder", "-compile",
            "-hardware", ARDUINO_HARDWARE_DIR,
            "-tools", ARDUINO_TOOLS_DIR,
            "-libraries", str(ARDUINO_ROOT / "libraries"),
            "-fqbn", fqbn,
            "-build-path", build_dir,
            str(sketch_path),
        ]
        try:
            proc = subprocess.Popen(
                compile_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except FileNotFoundError:
            yield "ERROR: arduino-builder not found (apt install arduino-core-avr arduino)"
            return
        for out_line in proc.stdout:
            yield out_line.rstrip()
        proc.wait()
        if proc.returncode != 0:
            yield f"ERROR: compile failed (exit {proc.returncode})"
            return
        yield "Compile OK."

        hex_path = Path(build_dir) / f"{sketch_path.name}.hex"
        if not hex_path.is_file():
            yield f"ERROR: expected hex not found at {hex_path}"
            return

        yield f"Uploading to {port} at {board['baud']} baud ..."
        upload_cmd = [
            "avrdude", "-v",
            "-p", board["mcu"],
            "-c", board["programmer"],
            "-P", port,
            "-b", str(board["baud"]),
            "-D",
            "-U", f"flash:w:{hex_path}:i",
        ]
        try:
            proc = subprocess.Popen(
                upload_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except FileNotFoundError:
            yield "ERROR: avrdude not found (apt install avrdude)"
            return
        for out_line in proc.stdout:
            yield out_line.rstrip()
        proc.wait()
        if proc.returncode != 0:
            yield f"ERROR: upload failed (exit {proc.returncode}). If it says the port is busy, make sure no other process (ROS2, this tool's own connection) has it open."
            return
        yield "Upload OK."
    finally:
        shutil.rmtree(build_dir, ignore_errors=True)
        if was_connected:
            time.sleep(0.5)
            try:
                link.connect(port, link.baud or 115200)
                yield "Reconnected."
            except Exception as exc:
                yield f"WARNING: reconnect failed: {exc}"


class Handler(BaseHTTPRequestHandler):
    server_version = "JambotTester/1.0"

    def log_message(self, fmt, *args):
        pass  # the web UI has its own log panel; keep the console quiet

    def _json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._serve_static("index.html")
        elif parsed.path == "/api/ports":
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self._json({"ports": ports})
        elif parsed.path == "/api/status":
            self._json({
                "connected": link.connected, "port": link.port, "baud": link.baud,
                "boot_log": link.boot_log,
            })
        elif parsed.path == "/api/telemetry":
            self._handle_telemetry()
        elif parsed.path == "/api/sketches":
            self._json({"sketches": list_sketches()})
        elif parsed.path == "/api/fqbns":
            self._json({k: v["label"] for k, v in FQBN_OPTIONS.items()})
        elif parsed.path == "/api/upload/stream":
            self._handle_upload_stream(parse_qs(parsed.query))
        elif parsed.path == "/api/lidar":
            self._handle_lidar()
        elif parsed.path == "/api/ros2/status":
            self._json(ros2_manager.status())
        elif parsed.path == "/api/ros2/log/stream":
            self._handle_ros2_log_stream()
        elif parsed.path == "/api/autotune/status":
            self._json(autotuner.status())
        elif parsed.path == "/api/autotune/log/stream":
            self._handle_autotune_log_stream()
        else:
            self._serve_static(parsed.path.lstrip("/"))

    def do_POST(self):
        parsed = urlparse(self.path)
        try:
            body = self._read_json_body()
        except Exception as exc:
            self._json({"error": str(exc)}, status=400)
            return

        if parsed.path == "/api/connect":
            self._handle_connect(body)
        elif parsed.path == "/api/disconnect":
            link.disconnect()
            self._json({"connected": False})
        elif parsed.path == "/api/command":
            self._handle_command(body)
        elif parsed.path == "/api/save_pid":
            self._handle_save_pid(body)
        elif parsed.path == "/api/ros2/start":
            self._handle_ros2_start(body)
        elif parsed.path == "/api/ros2/stop":
            ros2_manager.stop()
            self._json(ros2_manager.status())
        elif parsed.path == "/api/cmd_vel":
            self._handle_cmd_vel(body)
        elif parsed.path == "/api/autotune/start":
            self._handle_autotune_start(body)
        elif parsed.path == "/api/autotune/stop":
            autotuner.stop()
            self._json(autotuner.status())
        else:
            self._json({"error": "not found"}, status=404)

    def _serve_static(self, relpath):
        path = (STATIC_DIR / relpath).resolve()
        if STATIC_DIR != path and STATIC_DIR not in path.parents:
            self.send_error(403)
            return
        if not path.is_file():
            self.send_error(404)
            return
        content_type = {
            ".html": "text/html", ".js": "application/javascript", ".css": "text/css",
        }.get(path.suffix, "application/octet-stream")
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _handle_connect(self, body):
        port = body.get("port")
        baud = int(body.get("baud", 115200))
        if not port:
            self._json({"error": "port required"}, status=400)
            return
        if ros2_manager.running:
            self._json({"error": "ROS2 mode is running and owns the serial port -- stop it first"}, status=409)
            return
        try:
            link.connect(port, baud)
            self._json({
                "connected": True, "port": port, "baud": baud,
                "boot_log": link.boot_log,
            })
        except Exception as exc:
            self._json({"error": str(exc)}, status=500)

    def _handle_command(self, body):
        cmd = body.get("cmd", "")
        if not cmd:
            self._json({"error": "cmd required"}, status=400)
            return
        try:
            reply = link.send(cmd)
            self._json({"sent": cmd, "reply": reply})
        except Exception as exc:
            self._json({"error": str(exc)}, status=500)

    def _handle_save_pid(self, body):
        try:
            sketch = body["sketch"]
            kp = float(body["kp"]); kd = float(body["kd"])
            ki = float(body["ki"]); ko = float(body["ko"])
        except (KeyError, ValueError) as exc:
            self._json({"error": f"bad request: {exc}"}, status=400)
            return
        try:
            new_line = save_pid_defaults(sketch, kp, kd, ki, ko)
        except ValueError as exc:
            self._json({"error": str(exc)}, status=400)
            return
        self._json({"saved": True, "line": new_line})

    def _handle_telemetry(self):
        try:
            reply = link.send("t")
        except Exception as exc:
            self._json({"error": str(exc)}, status=500)
            return
        data = parse_telemetry(reply)
        if data is None:
            self._json({"error": "bad telemetry reply", "raw": reply}, status=502)
        else:
            data["raw"] = reply
            self._json(data)

    def _handle_lidar(self):
        if not ROS_AVAILABLE:
            self._json({"available": False, "error": "rclpy not importable -- source the ROS2 workspace before launching this tool"})
            return
        data = lidar_bridge.latest()
        if data is None:
            self._json({"available": False, "error": "no /scan message received yet -- is the lidar driver running?"})
            return
        data["available"] = True
        data["stale"] = (time.time() - data["stamp"]) > 2.0
        self._json(data)

    def _handle_ros2_start(self, body):
        launch_file = body.get("launch_file", "")
        args = body.get("args", [])
        if not isinstance(args, list):
            self._json({"error": "args must be a list"}, status=400)
            return
        try:
            ros2_manager.start(launch_file, args)
        except (ValueError, RuntimeError) as exc:
            self._json({"error": str(exc)}, status=400)
            return
        self._json(ros2_manager.status())

    def _handle_autotune_start(self, body):
        try:
            setpoint_rpm = body.get("setpoint_rpm", 100)
            relay_pwm = body.get("relay_pwm", 80)
            duration_s = body.get("duration_s", 10)
            autotuner.start(setpoint_rpm, relay_pwm, duration_s)
        except (ValueError, RuntimeError) as exc:
            self._json({"error": str(exc)}, status=400)
            return
        self._json(autotuner.status())

    def _handle_autotune_log_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        index = 0
        try:
            while True:
                lines, index = autotuner.log_since(index)
                for line in lines:
                    self.wfile.write(f"data: {line}\n\n".encode("utf-8"))
                self.wfile.flush()
                if not autotuner.running and not lines:
                    break
                time.sleep(0.3)
        except (BrokenPipeError, ConnectionResetError):
            return
        try:
            self.wfile.write(b"event: done\ndata: \n\n")
            self.wfile.flush()
        except Exception:
            pass

    def _handle_ros2_log_stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        index = 0
        try:
            while True:
                lines, index = ros2_manager.log_since(index)
                for line in lines:
                    self.wfile.write(f"data: {line}\n\n".encode("utf-8"))
                self.wfile.flush()
                if not ros2_manager.running and not lines:
                    break
                time.sleep(0.3)
        except (BrokenPipeError, ConnectionResetError):
            return
        try:
            self.wfile.write(b"event: done\ndata: \n\n")
            self.wfile.flush()
        except Exception:
            pass

    def _handle_cmd_vel(self, body):
        try:
            linear_x = float(body.get("linear_x", 0.0))
            angular_z = float(body.get("angular_z", 0.0))
        except (TypeError, ValueError):
            self._json({"error": "linear_x/angular_z must be numbers"}, status=400)
            return
        try:
            cmd_vel_bridge.publish(linear_x, angular_z)
        except Exception as exc:
            self._json({"error": str(exc)}, status=500)
            return
        self._json({"published": True, "linear_x": linear_x, "angular_z": angular_z})

    def _handle_upload_stream(self, query):
        sketch = query.get("sketch", [""])[0]
        fqbn = query.get("fqbn", [""])[0]
        port = query.get("port", [""])[0]
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            for line in run_upload(sketch, fqbn, port):
                self.wfile.write(f"data: {line}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        try:
            self.wfile.write(b"event: done\ndata: \n\n")
            self.wfile.flush()
        except Exception:
            pass


def main():
    sys.stdout.reconfigure(line_buffering=True)  # visible immediately when redirected to a log file
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--http-host", default="0.0.0.0")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--port", help="Serial port to auto-connect to on startup (optional)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if args.port:
        print(f"Connecting to {args.port} @ {args.baud}...")
        link.connect(args.port, args.baud)
        print("Connected. Boot log:")
        for line in link.boot_log:
            print(f"  {line}")

    if ROS_AVAILABLE:
        lidar_bridge.start()
        print("Lidar bridge: subscribing to /scan (rclpy available).")
    else:
        print("Lidar bridge: rclpy not importable, /api/lidar will report unavailable.")

    server = ThreadingHTTPServer((args.http_host, args.http_port), Handler)
    hostname = socket.gethostname()
    print(f"Serving on:")
    print(f"  http://localhost:{args.http_port}")
    print(f"  http://{hostname}:{args.http_port}  (from your laptop, if on the same network)")
    print("Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.disconnect()
        ros2_manager.stop()
        server.server_close()
        if ROS_AVAILABLE:
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
