# MPC Navigation Stack — FloMobility Assignment

> **Reference repo replaced:** [smoothing-rpp (Pure Pursuit)](https://github.com/manojkarnekar/smoothing-rpp)
> **Replacement:** Model Predictive Control (MPC) tracker with smooth path generation and static/dynamic obstacle avoidance.

---

## Architecture

```
┌─────────────────┐
│  waypoint.csv   │
│  (raw points)   │
└────────┬────────┘
         │
         ▼
┌──────────────────────────────┐
│     path_smoother.py         │
│  - CubicSpline interpolation │
│  - Uniform arc-length resamp │
│  Publishes /path (green)     │
│  Publishes /waypoints (red)  │
└──────────┬───────────────────┘
           │ /path
           ▼
┌──────────────────────────────┐
│      mpc_tracker.py          │
│  - Unicycle MPC (N=15 steps) │
│  - SLSQP optimizer           │
│  - Obstacle soft-barrier     │
│  - Emergency stop (<0.35m)   │
│  Subscribes /scan, /odom     │
│  Publishes /cmd_vel          │
└──────────────────────────────┘
           │
           ▼
     TurtleBot3 (Gazebo)

[Optional]
┌──────────────────────────────┐
│    obstacle_spawner.py       │
│  Randomly spawns/removes     │
│  Gazebo box obstacles in     │
│  front of robot for testing  │
└──────────────────────────────┘
```

---

## Task Checklist

| # | Task | Implementation |
|---|------|---------------|
| 1 | Replace Pure Pursuit with MPC | `mpc_tracker.py` — `MPCController` class using SLSQP optimization over N=15 step horizon |
| 2 | Generate smooth path from waypoints | `path_smoother.py` — CubicSpline with arc-length parameterization, 0.05 m resolution |
| 3 | Manoeuvre around obstacles, return to path | Soft obstacle barrier in MPC cost function; robot naturally steers around and re-joins the reference trajectory |
| 4 | Static **and** dynamic obstacles | Static: always in LiDAR; Dynamic: `obstacle_spawner.py` randomly spawns/removes boxes; both handled identically by MPC |

---

## How It Works

### 1. Path Smoothing (`path_smoother.py`)

- Loads waypoints from CSV (`x,y` per line)
- Computes cumulative arc-length parameter `t`
- Fits **independent CubicSplines** `x(t)` and `y(t)` with `not-a-knot` boundary conditions (scipy)
- Resamples uniformly at `path_resolution` (default 0.05 m)
- Publishes smooth path on `/path` and raw waypoints on `/waypoints`

### 2. MPC Tracker (`mpc_tracker.py`)

**State:** `[x, y, θ]` (unicycle model)

**Control inputs:** `[v, ω]` (linear and angular velocity)

**Prediction:**
```
x_{k+1} = x_k + v_k * cos(θ_k) * dt
y_{k+1} = y_k + v_k * sin(θ_k) * dt
θ_{k+1} = θ_k + ω_k * dt
```

**MPC cost function (minimized at each step):**
```
J = Σ_{k=1}^{N} [
      Q_pos * ||pos_k - ref_k||²
    + Q_head * heading_error_k²
    + R_v * v_k²
    + R_w * ω_k²
    + obs_weight * max(0, obs_margin - dist_to_obstacle)²  ← soft barrier
  ]
  + terminal_weight * ||pos_N - goal||²
```

**Constraints:**
- `v ∈ [-0.05, 0.3]` m/s
- `ω ∈ [-1.5, 1.5]` rad/s

**Obstacle handling:**
- LiDAR points within 2 m are converted to world-frame obstacle coordinates
- Soft barrier penalty in MPC cost naturally steers the robot around obstacles
- Emergency stop if any obstacle < 0.35 m in ±40° frontal cone
- Once obstacle clears, robot naturally re-converges to reference path (no explicit replanning needed — MPC handles it)

**Warm starting:** Previous optimal control sequence is shifted and reused as initial guess, reducing solve time.

---

## Installation

```bash
# Prerequisites: ROS2 Humble, TurtleBot3 packages, Gazebo, Python 3
pip install scipy numpy

cd ~/ros2_ws/src
git clone <this_repo> mpc_nav
cd ~/ros2_ws
colcon build --packages-select mpc_nav
source install/setup.bash
```

---

## Usage

### Run simulation with MPC tracker

**Terminal 1 — Full bringup:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch mpc_nav mpc_nav_bringup.py
```

**Terminal 1 — With dynamic obstacles:**
```bash
ros2 launch mpc_nav mpc_nav_bringup.py spawn_obstacles:=true
```

### Run nodes manually

**Terminal 1 — Simulation:**
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Terminal 2 — Path smoother:**
```bash
ros2 run mpc_nav path_smoother --ros-args \
  -p path_file:=$(pwd)/waypoints/waypoint.csv \
  -p path_resolution:=0.05
```

**Terminal 3 — MPC tracker:**
```bash
ros2 run mpc_nav mpc_tracker --ros-args \
  -p v_max:=0.3 \
  -p horizon:=15 \
  -p obs_weight:=100.0
```

**Terminal 4 (optional) — Dynamic obstacle spawner:**
```bash
ros2 run mpc_nav obstacle_spawner --ros-args \
  -p spawn_interval:=10.0 \
  -p obstacle_dist:=0.9
```

---

## Configuration Parameters

### `path_smoother`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `path_file` | `waypoints/waypoint.csv` | Path to waypoints CSV |
| `path_resolution` | `0.05` m | Distance between smooth path points |
| `frame_id` | `odom` | TF frame for published paths |
| `publish_rate` | `1.0` Hz | How often to republish paths |

### `mpc_tracker`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `horizon` | `15` | MPC prediction horizon (steps) |
| `dt` | `0.1` s | Prediction time step |
| `v_max` | `0.3` m/s | Max linear velocity |
| `w_max` | `1.5` rad/s | Max angular velocity |
| `Q_pos` | `10.0` | Position tracking weight |
| `Q_head` | `2.0` | Heading tracking weight |
| `R_v` | `0.5` | Linear velocity effort weight |
| `R_w` | `0.5` | Angular velocity effort weight |
| `terminal_weight` | `20.0` | Terminal state cost weight |
| `obs_weight` | `100.0` | Obstacle avoidance weight |
| `obs_margin` | `0.45` m | Safety margin around obstacles |
| `obstacle_stop_dist` | `0.35` m | Emergency stop distance |
| `lidar_range_max` | `2.0` m | Only consider LiDAR points within this range |
| `goal_tolerance` | `0.15` m | Goal reached threshold |
| `control_frequency` | `10.0` Hz | MPC solve frequency |

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/path` | `nav_msgs/Path` | sub | Smooth trajectory from path_smoother |
| `/waypoints` | `nav_msgs/Path` | sub | Raw waypoints (visualization) |
| `/odom` | `nav_msgs/Odometry` | sub | Robot pose |
| `/scan` | `sensor_msgs/LaserScan` | sub | 2D LiDAR data |
| `/cmd_vel` | `geometry_msgs/Twist` | pub | Velocity commands to robot |
| `/mpc/goal_reached` | `std_msgs/Bool` | pub | Goal reached notification |

---

## Design Decisions

**Why MPC over Pure Pursuit?**
- MPC optimizes over a future horizon → smoother velocity profiles
- Naturally handles constraints (velocity limits, obstacle margins)
- Soft obstacle barrier in cost = smooth avoidance without hard replanning
- Returns to original path automatically after obstacle clears

**Why CubicSpline over Bézier?**
- Arc-length parameterization gives truly uniform spacing
- scipy's `CubicSpline` is C² continuous (smooth curvature)
- Works with arbitrary numbers of waypoints

**Why SLSQP?**
- Handles box constraints (velocity bounds) efficiently
- Fast convergence for smooth, well-conditioned MPC problems
- Warm-starting further reduces solve time per iteration

---

## Dependencies

- ROS2 Humble
- `turtlebot3_*` packages
- Gazebo Classic
- Python: `numpy`, `scipy`
