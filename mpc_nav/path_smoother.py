#!/usr/bin/env python3
"""
Path Smoother Node for ROS2.

Reads waypoints from a CSV file, generates a smooth trajectory using
cubic spline interpolation (scipy CubicSpline), and publishes it on /path.

Also publishes raw waypoints on /waypoints for visualization.

Usage:
    ros2 run mpc_nav path_smoother --ros-args -p path_file:=<abs_path>.csv
"""

import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import CubicSpline


class PathSmootherNode(Node):
    def __init__(self):
        super().__init__("path_smoother")

        self.declare_parameter("path_file", "waypoints/waypoint.csv")
        self.declare_parameter("path_resolution", 0.05)   # m between points
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("publish_rate", 1.0)        # Hz (republish for late subs)

        path_file = self.get_parameter("path_file").value
        self.resolution = self.get_parameter("path_resolution").value
        self.frame_id = self.get_parameter("frame_id").value

        # Expand relative paths relative to the installed package share directory.
        if not os.path.isabs(path_file):
            pkg_dir = get_package_share_directory("mpc_nav")
            path_file = os.path.join(pkg_dir, path_file)

        self.raw_wp = self._load_waypoints(path_file)
        self.smooth_path = self._smooth(self.raw_wp)

        self.wp_pub = self.create_publisher(Path, "/waypoints", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)

        rate = self.get_parameter("publish_rate").value
        self.create_timer(1.0 / rate, self.publish_paths)
        self.get_logger().info(
            f"Loaded {len(self.raw_wp)} waypoints -> {len(self.smooth_path)} smooth points."
        )

    # ──────────────────────────────────────────────────────────────────────

    def _load_waypoints(self, filepath: str):
        """Load waypoints from CSV file. Each row: x,y or x,y,theta."""
        waypoints = []
        try:
            with open(filepath, "r") as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            waypoints.append([float(row[0]), float(row[1])])
                        except ValueError:
                            pass  # skip header rows
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found: {filepath}")
            # Provide a default demo path
            waypoints = [
                [0.0, 0.0], [1.0, 0.0], [2.0, 0.5],
                [3.0, 0.5], [3.5, 1.5], [3.0, 2.5],
                [2.0, 3.0], [1.0, 2.5], [0.0, 2.0],
            ]
            self.get_logger().warn("Using built-in demo path.")
        return waypoints

    def _smooth(self, waypoints):
        """
        Generate a smooth path using cubic spline interpolation.

        Steps:
          1. Compute cumulative arc-length parameter t along waypoints.
          2. Fit independent CubicSplines x(t) and y(t).
          3. Resample at uniform arc-length resolution.
        """
        if len(waypoints) < 2:
            return waypoints

        pts = np.array(waypoints)

        # Remove duplicate consecutive points
        mask = np.ones(len(pts), dtype=bool)
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i] - pts[i - 1]) < 1e-6:
                mask[i] = False
        pts = pts[mask]

        if len(pts) < 2:
            return waypoints

        # Arc-length parameterization
        diffs = np.diff(pts, axis=0)
        seg_lengths = np.hypot(diffs[:, 0], diffs[:, 1])
        t = np.concatenate([[0.0], np.cumsum(seg_lengths)])

        # Avoid duplicate t values (can happen with closely spaced points)
        _, unique_idx = np.unique(t, return_index=True)
        t = t[unique_idx]
        pts = pts[unique_idx]

        if len(pts) < 2:
            return waypoints

        # Fit cubic splines
        cs_x = CubicSpline(t, pts[:, 0], bc_type="not-a-knot")
        cs_y = CubicSpline(t, pts[:, 1], bc_type="not-a-knot")

        # Resample at uniform resolution
        total_length = t[-1]
        n_samples = max(int(total_length / self.resolution) + 1, 2)
        t_uniform = np.linspace(0.0, total_length, n_samples)

        xs = cs_x(t_uniform)
        ys = cs_y(t_uniform)

        smooth = [[float(x), float(y)] for x, y in zip(xs, ys)]
        return smooth

    # ──────────────────────────────────────────────────────────────────────

    def _make_path_msg(self, points):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for pt in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_paths(self):
        self.wp_pub.publish(self._make_path_msg(self.raw_wp))
        self.path_pub.publish(self._make_path_msg(self.smooth_path))


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
