#!/usr/bin/env python3
"""
Dynamic Obstacle Spawner for Gazebo testing.

Randomly spawns and removes box obstacles in front of the robot
to test dynamic obstacle avoidance.

Usage (in separate terminal after launching sim):
    ros2 run mpc_nav obstacle_spawner
"""

import rclpy
from rclpy.node import Node
import random
import math
import subprocess
import time
from nav_msgs.msg import Odometry


BOX_SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} 0.25 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>0.3 0.3 0.5</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.3 0.3 0.5</size></box>
        </geometry>
        <material>
          <ambient>1 0.2 0.2 1</ambient>
          <diffuse>1 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__("obstacle_spawner")

        self.declare_parameter("spawn_interval", 8.0)   # seconds between spawns
        self.declare_parameter("obstacle_dist", 0.8)    # m ahead of robot
        self.declare_parameter("remove_after", 6.0)     # seconds before removing
        self.declare_parameter("max_obstacles", 3)

        self.spawn_interval = self.get_parameter("spawn_interval").value
        self.obs_dist = self.get_parameter("obstacle_dist").value
        self.remove_after = self.get_parameter("remove_after").value
        self.max_obs = self.get_parameter("max_obstacles").value

        self.robot_pose = None
        self.active_obstacles = {}  # name -> spawn_time
        self.obs_counter = 0

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_timer(self.spawn_interval, self.maybe_spawn)
        self.create_timer(1.0, self.maybe_remove)

        self.get_logger().info("Obstacle spawner ready.")

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.robot_pose = (pos.x, pos.y, yaw)

    def maybe_spawn(self):
        if self.robot_pose is None:
            return
        if len(self.active_obstacles) >= self.max_obs:
            return

        rx, ry, rth = self.robot_pose
        # Spawn slightly ahead + random lateral offset
        dist = self.obs_dist + random.uniform(0.0, 0.3)
        lat = random.uniform(-0.2, 0.2)
        ox = rx + dist * math.cos(rth) - lat * math.sin(rth)
        oy = ry + dist * math.sin(rth) + lat * math.cos(rth)

        name = f"dyn_obs_{self.obs_counter}"
        self.obs_counter += 1

        sdf = BOX_SDF_TEMPLATE.format(name=name, x=ox, y=oy)
        sdf_file = f"/tmp/{name}.sdf"
        with open(sdf_file, "w") as f:
            f.write(sdf)

        cmd = [
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-file", sdf_file,
            "-entity", name,
        ]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.active_obstacles[name] = time.time()
            self.get_logger().info(f"Spawned obstacle '{name}' at ({ox:.2f}, {oy:.2f})")
        except Exception as e:
            self.get_logger().warn(f"Failed to spawn obstacle: {e}")

    def maybe_remove(self):
        now = time.time()
        to_remove = [
            name for name, t in self.active_obstacles.items()
            if now - t > self.remove_after
        ]
        for name in to_remove:
            cmd = ["ros2", "service", "call",
                   "/delete_entity", "gazebo_msgs/srv/DeleteEntity",
                   f"{{name: '{name}'}}"]
            try:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                del self.active_obstacles[name]
                self.get_logger().info(f"Removed obstacle '{name}'")
            except Exception as e:
                self.get_logger().warn(f"Failed to remove obstacle: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
