# ROS 2 Jazzy MPC TurtleBot3 Navigation

This package targets ROS 2 Jazzy with TurtleBot3 Gazebo Sim.

## Dependencies

Install ROS 2 Jazzy, TurtleBot3, TurtleBot3 simulations, and Python math packages:

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

If `turtlebot3_gazebo` is not available from apt on your machine, build the Jazzy branch:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

## Build

```bash
mkdir -p ~/ros2_ws/src
cp -r "/Users/nagendrak/Documents/New project/ros2_mpc_nav_rewrite" ~/ros2_ws/src/mpc_nav
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select mpc_nav
source install/setup.bash
```

## Run

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch mpc_nav mpc_nav_bringup.py
```

With dynamic test obstacles:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch mpc_nav mpc_nav_bringup.py spawn_obstacles:=true
```

The obstacle spawner uses Jazzy/Gazebo Sim commands:

```bash
ros2 run ros_gz_sim create
ros2 run ros_gz_sim delete_entity
```

## Nodes

- `path_smoother`: reads `waypoints/waypoint.csv`, smooths the route, publishes `/path` and `/waypoints`.
- `mpc_tracker`: subscribes to `/path`, `/odom`, `/scan`, and publishes `/cmd_vel`.
- `obstacle_spawner`: optional Gazebo Sim obstacle test node.
