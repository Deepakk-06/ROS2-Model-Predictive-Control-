# ROS 2 MPC Navigation for TurtleBot3

Model Predictive Control (MPC) path tracking for TurtleBot3 Burger using **ROS 2 Jazzy**, **Gazebo Sim**, and **RViz**.

The robot reads waypoints from a CSV, smooths them into a spline path, then tracks it using nonlinear MPC — publishing `TwistStamped` velocity commands to TurtleBot3 in Gazebo Sim.

## Demo

[Watch MPC demo video](media/MPC.mov)

![RViz demo 1](media/Screenshot%202026-05-30%20at%2012.05.28%E2%80%AFPM.png)
![RViz demo 2](media/Screenshot%202026-05-30%20at%2012.52.51%E2%80%AFPM.png)

---

## Features

- ROS 2 Jazzy + Gazebo Sim + RViz
- TurtleBot3 Burger simulation
- Cubic spline path smoothing from CSV waypoints
- Nonlinear MPC trajectory tracking (SciPy SLSQP with warm starting)
- Forward nearest-point tracking — robot does not reset to waypoint 0 on path republish
- `/cmd_vel` published as `geometry_msgs/msg/TwistStamped` (Jazzy-compatible)
- LaserScan-based obstacle awareness with soft-barrier cost
- Optional dynamic obstacle spawning in Gazebo Sim

---

## Package Structure

```
mpc_nav/
├── launch/
│   └── mpc_nav_bringup.py
├── media/
│   ├── MPC.mov
│   └── screenshots/
├── mpc_nav/
│   ├── __init__.py
│   ├── mpc_tracker.py
│   ├── obstacle_spawner.py
│   └── path_smoother.py
├── waypoints/
│   └── waypoint.csv
├── package.xml
├── setup.cfg
└── setup.py
```

---

## System Architecture

```
waypoints/waypoint.csv
        │
        ▼
path_smoother.py  ──publishes──▶  /path, /waypoints
        │
        ▼
mpc_tracker.py
  ├── subscribes: /path, /odom, /scan
  └── publishes:  /cmd_vel, /mpc/goal_reached
        │
        ▼
TurtleBot3 Burger (Gazebo Sim)
```

---

## Dependencies

Install ROS 2 Jazzy and TurtleBot3 packages:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-rviz2 \
  python3-numpy \
  python3-scipy
```

If `turtlebot3_gazebo` is missing, build from source:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Deepakk-06/ROS2-Model-Predictive-Control-.git mpc_nav
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select mpc_nav
source install/setup.bash
```

---

## Run

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch mpc_nav mpc_nav_bringup.py
```

With optional dynamic obstacles:

```bash
ros2 launch mpc_nav mpc_nav_bringup.py spawn_obstacles:=true
```

---

## RViz Setup

Set **Fixed Frame** to `odom`.

| Display    | Topic        | Style                          |
|------------|--------------|--------------------------------|
| Path       | `/path`      | Green line, width ~0.03        |
| Path       | `/waypoints` | Yellow line, width ~0.03       |
| LaserScan  | `/scan`      | Flat squares, size ~0.03       |
| Odometry   | `/odom`      | Arrow display                  |

> If RViz shows `Frame [map] does not exist`, go to **Global Options → Fixed Frame → odom**.

---

## ROS Topics

| Topic               | Type                              | Direction         | Description              |
|---------------------|-----------------------------------|-------------------|--------------------------|
| `/path`             | `nav_msgs/msg/Path`               | smoother → tracker | Smoothed reference path  |
| `/waypoints`        | `nav_msgs/msg/Path`               | smoother → RViz   | Raw waypoint path        |
| `/odom`             | `nav_msgs/msg/Odometry`           | Gazebo → tracker  | Robot pose               |
| `/scan`             | `sensor_msgs/msg/LaserScan`       | Gazebo → tracker  | LiDAR scan               |
| `/cmd_vel`          | `geometry_msgs/msg/TwistStamped`  | tracker → Gazebo  | Velocity commands        |
| `/mpc/goal_reached` | `std_msgs/msg/Bool`               | tracker → out     | Goal completion status   |

---

## MPC Details

**Unicycle motion model:**

```
x[k+1]     = x[k] + v[k] * cos(θ[k]) * dt
y[k+1]     = y[k] + v[k] * sin(θ[k]) * dt
θ[k+1]     = θ[k] + ω[k] * dt
```

**Cost function minimizes:** position error + heading error + velocity effort + angular effort + obstacle soft-barrier + terminal goal error.

**Tuned parameters** (set in `launch/mpc_nav_bringup.py`):

| Parameter          | Value  | Purpose                        |
|--------------------|--------|--------------------------------|
| `v_max`            | 0.18   | Max forward speed              |
| `v_min`            | -0.05  | Small reverse allowed          |
| `w_max`            | 1.5    | Angular velocity limit         |
| `horizon`          | 25     | MPC lookahead steps            |
| `dt`               | 0.1    | Timestep (s)                   |
| `Q_pos`            | 10.0   | Position tracking weight       |
| `Q_head`           | 2.0    | Heading tracking weight        |
| `R_v`              | 0.5    | Linear velocity effort weight  |
| `R_w`              | 0.5    | Angular effort weight          |
| `terminal_weight`  | 20.0   | Goal convergence weight        |
| `obs_weight`       | 100.0  | Obstacle penalty               |
| `obs_margin`       | 0.45   | Safety margin (m)              |
| `goal_tolerance`   | 0.08   | Goal completion threshold (m)  |
| `obstacle_stop_dist` | 0.12 | Emergency stop distance (m)   |
| `control_frequency` | 10.0  | MPC loop rate (Hz)             |

---

## Key Jazzy Changes

This package targets **ROS 2 Jazzy + Gazebo Sim** (not Humble/Gazebo Classic).

- `/cmd_vel` published as `geometry_msgs/msg/TwistStamped` to match `ros_gz_bridge`
- Fixed frame is `odom` (not `map`)
- Path republication no longer resets robot to waypoint 0
- Forward nearest-point search prevents backtracking on missed waypoints

---

## Debug Commands

```bash
# List active nodes and topics
ros2 node list
ros2 topic list

# Verify /cmd_vel message type
ros2 topic info /cmd_vel -v
# Expected: geometry_msgs/msg/TwistStamped

# Monitor key topics
ros2 topic echo /cmd_vel --field twist
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /mpc/goal_reached
ros2 topic hz /scan

# Manual motion test (if robot is not moving)
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  'twist: {linear: {x: 0.3}}' -r 10
```

> If `/odom` position changes after the manual test, Gazebo physics is working. RViz is visualization only — check `/odom` values if the robot appears stationary in Gazebo.
