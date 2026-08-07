#!/usr/bin/env python3
"""Simple Nav2 waypoint mission commander for jambot_nano.

Usage:
  ros2 run jambot_nano mission_commander.py [--waypoints file.yaml] [--loiter 5]

Sends a sequence of poses to the robot via Nav2's action interface.
Each pose is (x, y, theta_deg) in the map frame.

Example waypoints.yaml:
  waypoints:
    - name: "start"
      x: 0.0
      y: 0.0
      theta_deg: 0.0
    - name: "corner"
      x: 2.0
      y: 2.0
      theta_deg: 90.0
"""
import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node


def quat_from_yaw(yaw_rad):
    """Convert yaw angle to quaternion."""
    half = yaw_rad / 2.0
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half),
        w=math.cos(half),
    )


class MissionCommander(Node):
    def __init__(self, waypoints_file=None, loiter_s=0):
        super().__init__("mission_commander")
        self.navigator = BasicNavigator()
        self.waypoints_file = waypoints_file
        self.loiter_s = loiter_s
        self.waypoints = []

    def load_waypoints(self):
        """Load waypoints from a YAML file."""
        if not self.waypoints_file:
            return False
        path = Path(self.waypoints_file)
        if not path.exists():
            self.get_logger().error(f"Waypoints file not found: {self.waypoints_file}")
            return False
        with open(path) as f:
            data = yaml.safe_load(f)
        self.waypoints = data.get("waypoints", [])
        if not self.waypoints:
            self.get_logger().error("No waypoints in file")
            return False
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}")
        return True

    def create_pose(self, x, y, theta_deg):
        """Create a PoseStamped from x, y, theta."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = quat_from_yaw(math.radians(theta_deg))
        return pose

    def run_mission(self):
        """Execute the waypoint mission."""
        if self.waypoints_file and not self.load_waypoints():
            return False

        # If no file, use hardcoded example: a small loop
        if not self.waypoints:
            self.get_logger().warn("No waypoints file; using example loop")
            self.waypoints = [
                {"name": "start", "x": 0.0, "y": 0.0, "theta_deg": 0.0},
                {"name": "forward", "x": 1.0, "y": 0.0, "theta_deg": 0.0},
                {"name": "left", "x": 1.0, "y": 1.0, "theta_deg": 90.0},
                {"name": "back", "x": 0.0, "y": 1.0, "theta_deg": 180.0},
                {"name": "start", "x": 0.0, "y": 0.0, "theta_deg": 270.0},
            ]

        self.navigator.waitUntilNav2Active()

        for wp in self.waypoints:
            name = wp.get("name", f"wp_{wp.get('x')}_{wp.get('y')}")
            x = wp.get("x", 0.0)
            y = wp.get("y", 0.0)
            theta = wp.get("theta_deg", 0.0)

            pose = self.create_pose(x, y, theta)
            self.get_logger().info(f"Going to {name}: ({x:.2f}, {y:.2f}, {theta:.0f}deg)")

            # Send goal
            self.navigator.goToPose(pose)

            # Wait for result with timeout
            i = 0
            while not self.navigator.isNavComplete():
                i += 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    self.get_logger().info(
                        f"  Distance to goal: {feedback.distance_remaining:.2f}m"
                    )
                time.sleep(0.1)
                # Timeout safety
                if i > 600:  # 60 seconds
                    self.get_logger().warn(f"  Timeout at {name}, moving to next waypoint")
                    self.navigator.cancelNav()
                    break

            self.get_logger().info(f"  Reached {name}")

            # Loiter (stay at waypoint for a bit)
            if self.loiter_s > 0:
                self.get_logger().info(f"  Loitering for {self.loiter_s}s")
                time.sleep(self.loiter_s)

        self.get_logger().info("Mission complete!")
        return True


def main():
    parser = argparse.ArgumentParser(description="Nav2 waypoint mission commander")
    parser.add_argument(
        "--waypoints",
        type=str,
        help="YAML file with waypoints list",
    )
    parser.add_argument(
        "--loiter",
        type=float,
        default=0,
        help="Seconds to pause at each waypoint (default 0)",
    )
    args = parser.parse_args()

    rclpy.init()
    commander = MissionCommander(waypoints_file=args.waypoints, loiter_s=args.loiter)

    try:
        success = commander.run_mission()
        sys.exit(0 if success else 1)
    except Exception as e:
        commander.get_logger().error(f"Mission failed: {e}")
        sys.exit(1)
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
