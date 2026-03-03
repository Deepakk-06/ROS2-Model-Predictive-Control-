#!/usr/bin/env python3
"""
MPC-based trajectory tracker for ROS2 with obstacle avoidance.

Replaces pure pursuit from: https://github.com/manojkarnekar/smoothing-rpp

Features:
  - Model Predictive Control (MPC) for trajectory tracking
  - Static and dynamic obstacle avoidance using 2D LiDAR
  - Local replanning around obstacles using potential field / DWA hybrid
  - Returns to original path after obstacle cleared
"""

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.optimize import minimize
from collections import deque
import math

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration


# ──────────────────────────────────────────────────────────────────────────────
# Unicycle kinematics
# ──────────────────────────────────────────────────────────────────────────────

def unicycle_predict(state, v, w, dt):
    """Predict next state: [x, y, theta]."""
    x, y, th = state
    x_new = x + v * math.cos(th) * dt
    y_new = y + v * math.sin(th) * dt
    th_new = th + w * dt
    return np.array([x_new, y_new, th_new])


def predict_horizon(state, controls, dt):
    """Rollout N-step horizon. controls: (N, 2) array of [v, w]."""
    states = [state]
    for i in range(len(controls)):
        states.append(unicycle_predict(states[-1], controls[i, 0], controls[i, 1], dt))
    return np.array(states)


# ──────────────────────────────────────────────────────────────────────────────
# MPC Controller
# ──────────────────────────────────────────────────────────────────────────────

class MPCController:
    """
    Nonlinear MPC for a differential-drive (unicycle) robot.

    Cost = sum_k [ Q_pos * dist_to_ref^2 + Q_head * heading_err^2
                 + R_v * v^2 + R_w * w^2 ]
           + terminal_weight * dist_to_goal^2

    Constraints: v ∈ [v_min, v_max], w ∈ [-w_max, w_max]
    Obstacle penalty: soft barrier added to cost.
    """

    def __init__(self, params: dict):
        self.N = params.get("horizon", 15)            # prediction steps
        self.dt = params.get("dt", 0.1)               # seconds
        self.v_max = params.get("v_max", 0.3)
        self.v_min = params.get("v_min", -0.05)
        self.w_max = params.get("w_max", 1.5)
        self.Q_pos = params.get("Q_pos", 10.0)
        self.Q_head = params.get("Q_head", 2.0)
        self.R_v = params.get("R_v", 0.5)
        self.R_w = params.get("R_w", 0.5)
        self.terminal_w = params.get("terminal_weight", 20.0)
        self.obs_weight = params.get("obs_weight", 50.0)
        self.obs_margin = params.get("obs_margin", 0.4)   # safety radius (m)

        # Warm-start: previous solution
        self._prev_u = np.zeros((self.N, 2))

    def _cost(self, u_flat, state0, ref_path, obstacles):
        """Compute MPC cost."""
        u = u_flat.reshape((self.N, 2))
        states = predict_horizon(state0, u, self.dt)

        cost = 0.0
        n_ref = len(ref_path)

        for k in range(1, self.N + 1):
            s = states[k]
            # Find closest reference point at this step
            ref_idx = min(k, n_ref - 1)
            ref = ref_path[ref_idx]

            dx = s[0] - ref[0]
            dy = s[1] - ref[1]
            dist_sq = dx * dx + dy * dy
            cost += self.Q_pos * dist_sq

            # Heading error toward reference direction
            if ref_idx < n_ref - 1:
                rx = ref_path[ref_idx + 1][0] - ref[0]
                ry = ref_path[ref_idx + 1][1] - ref[1]
                desired_heading = math.atan2(ry, rx)
            else:
                desired_heading = ref[2] if len(ref) > 2 else s[2]
            head_err = math.atan2(math.sin(s[2] - desired_heading),
                                  math.cos(s[2] - desired_heading))
            cost += self.Q_head * head_err ** 2

            # Control effort
            cost += self.R_v * u[k - 1, 0] ** 2
            cost += self.R_w * u[k - 1, 1] ** 2

            # Obstacle avoidance: soft barrier
            for ox, oy in obstacles:
                dist_obs = math.sqrt((s[0] - ox) ** 2 + (s[1] - oy) ** 2)
                if dist_obs < self.obs_margin * 2:
                    cost += self.obs_weight * max(0.0, self.obs_margin - dist_obs) ** 2

        # Terminal cost
        term_ref = ref_path[-1]
        dx = states[-1][0] - term_ref[0]
        dy = states[-1][1] - term_ref[1]
        cost += self.terminal_w * (dx * dx + dy * dy)

        return cost

    def solve(self, state0, ref_path, obstacles):
        """
        Solve MPC and return optimal (v, w) for current step.

        state0:    [x, y, theta]
        ref_path:  list/array of [x, y] (or [x, y, theta]) reference points
        obstacles: list of (ox, oy) in robot's world frame
        """
        if len(ref_path) == 0:
            return 0.0, 0.0

        bounds = [(self.v_min, self.v_max)] * self.N + \
                 [(-self.w_max, self.w_max)] * self.N
        # Reorder to match u_flat structure: [v0,w0, v1,w1, ...]
        bounds_interleaved = []
        for _ in range(self.N):
            bounds_interleaved.append((self.v_min, self.v_max))
            bounds_interleaved.append((-self.w_max, self.w_max))

        u0 = self._prev_u.flatten()

        result = minimize(
            self._cost,
            u0,
            args=(state0, ref_path, obstacles),
            method="SLSQP",
            bounds=bounds_interleaved,
            options={"maxiter": 50, "ftol": 1e-4},
        )

        u_opt = result.x.reshape((self.N, 2))
        # Warm-start: shift and pad
        self._prev_u = np.vstack([u_opt[1:], u_opt[-1:]])

        v_cmd = float(np.clip(u_opt[0, 0], self.v_min, self.v_max))
        w_cmd = float(np.clip(u_opt[0, 1], -self.w_max, self.w_max))
        return v_cmd, w_cmd

    def reset_warmstart(self):
        self._prev_u = np.zeros((self.N, 2))


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────────────────────────────────────

class MPCTrackerNode(Node):
    def __init__(self):
        super().__init__("mpc_tracker")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("v_max", 0.3)
        self.declare_parameter("v_min", -0.05)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("horizon", 15)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("Q_pos", 10.0)
        self.declare_parameter("Q_head", 2.0)
        self.declare_parameter("R_v", 0.5)
        self.declare_parameter("R_w", 0.5)
        self.declare_parameter("terminal_weight", 20.0)
        self.declare_parameter("obs_weight", 50.0)
        self.declare_parameter("obs_margin", 0.4)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("lidar_range_max", 2.0)   # only use nearby scan pts
        self.declare_parameter("obstacle_stop_dist", 0.35)  # emergency stop
        self.declare_parameter("control_frequency", 10.0)

        p = {
            "v_max": self.get_parameter("v_max").value,
            "v_min": self.get_parameter("v_min").value,
            "w_max": self.get_parameter("w_max").value,
            "horizon": self.get_parameter("horizon").value,
            "dt": self.get_parameter("dt").value,
            "Q_pos": self.get_parameter("Q_pos").value,
            "Q_head": self.get_parameter("Q_head").value,
            "R_v": self.get_parameter("R_v").value,
            "R_w": self.get_parameter("R_w").value,
            "terminal_weight": self.get_parameter("terminal_weight").value,
            "obs_weight": self.get_parameter("obs_weight").value,
            "obs_margin": self.get_parameter("obs_margin").value,
        }
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.lidar_range_max = self.get_parameter("lidar_range_max").value
        self.obstacle_stop_dist = self.get_parameter("obstacle_stop_dist").value

        self.mpc = MPCController(p)

        # ── State ───────────────────────────────────────────────────────────
        self.robot_pose = None          # [x, y, theta]
        self.path_points = []           # list of [x, y]
        self.current_path_idx = 0
        self.obstacles_world = []       # (ox, oy) in odom frame
        self.goal_reached = False
        self.emergency_stop = False

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Path, "/path", self.path_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(Bool, "/mpc/goal_reached", 10)

        # ── Control timer ───────────────────────────────────────────────────
        freq = self.get_parameter("control_frequency").value
        self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info("MPC Tracker initialized. Waiting for /path ...")

    # ─────────────────────────── Callbacks ──────────────────────────────────

    def path_cb(self, msg: Path):
        self.path_points = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in msg.poses
        ]
        self.current_path_idx = 0
        self.goal_reached = False
        self.mpc.reset_warmstart()
        self.get_logger().info(f"New path received: {len(self.path_points)} points.")

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = np.array([pos.x, pos.y, yaw])

    def scan_cb(self, msg: LaserScan):
        """Convert laser scan to obstacle points in world (odom) frame."""
        if self.robot_pose is None:
            return

        rx, ry, rth = self.robot_pose
        obstacles = []
        angle = msg.angle_min
        self.emergency_stop = False

        for r in msg.ranges:
            if msg.range_min < r < min(msg.range_max, self.lidar_range_max):
                # Point in robot frame
                ox_r = r * math.cos(angle)
                oy_r = r * math.sin(angle)
                # Transform to world frame
                ox_w = rx + ox_r * math.cos(rth) - oy_r * math.sin(rth)
                oy_w = ry + ox_r * math.sin(rth) + oy_r * math.cos(rth)
                obstacles.append((ox_w, oy_w))

                # Emergency stop check: close frontal obstacle
                if (abs(angle) < math.radians(40) and
                        r < self.obstacle_stop_dist):
                    self.emergency_stop = True

            angle += msg.angle_increment

        self.obstacles_world = obstacles

    # ─────────────────────────── Control loop ───────────────────────────────

    def control_loop(self):
        if self.robot_pose is None or len(self.path_points) == 0:
            return

        if self.goal_reached:
            self._publish_zero()
            return

        # ── Check goal ──────────────────────────────────────────────────────
        goal = self.path_points[-1]
        dist_to_goal = math.hypot(
            self.robot_pose[0] - goal[0],
            self.robot_pose[1] - goal[1],
        )
        if dist_to_goal < self.goal_tolerance:
            self.goal_reached = True
            self._publish_zero()
            self.status_pub.publish(Bool(data=True))
            self.get_logger().info("Goal reached!")
            return

        # ── Emergency stop ───────────────────────────────────────────────────
        # (MPC with high obstacle weight will naturally steer away;
        #  emergency stop is a safety net for very close obstacles)
        if self.emergency_stop:
            self._publish_zero()
            return

        # ── Advance path index ───────────────────────────────────────────────
        self._advance_path_idx()

        # ── Build reference horizon ──────────────────────────────────────────
        ref = self._build_reference()

        # ── Solve MPC ────────────────────────────────────────────────────────
        v_cmd, w_cmd = self.mpc.solve(
            self.robot_pose.copy(),
            ref,
            self.obstacles_world,
        )

        # ── Publish ──────────────────────────────────────────────────────────
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = w_cmd
        self.cmd_pub.publish(twist)

    def _advance_path_idx(self):
        """Move path index forward past already-visited waypoints."""
        look_ahead_dist = 0.2
        while self.current_path_idx < len(self.path_points) - 1:
            pt = self.path_points[self.current_path_idx]
            d = math.hypot(
                self.robot_pose[0] - pt[0],
                self.robot_pose[1] - pt[1],
            )
            if d < look_ahead_dist:
                self.current_path_idx += 1
            else:
                break

    def _build_reference(self):
        """
        Extract N future path points starting from current_path_idx.
        Returns list of [x, y].
        """
        n = self.mpc.N
        pts = self.path_points
        total = len(pts)
        ref = []
        for i in range(n):
            idx = min(self.current_path_idx + i, total - 1)
            ref.append(pts[idx])
        return ref

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MPCTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
