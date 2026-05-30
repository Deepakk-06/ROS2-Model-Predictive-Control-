"""
Launch file: mpc_nav_bringup.py

Starts:
  1. Gazebo simulation with TurtleBot3
  2. Path smoother node
  3. MPC tracker node
  4. RViz2 for visualization
  5. (Optional) Dynamic obstacle spawner

Usage:
    export TURTLEBOT3_MODEL=burger
    ros2 launch mpc_nav mpc_nav_bringup.py
    # With dynamic obstacles:
    ros2 launch mpc_nav mpc_nav_bringup.py spawn_obstacles:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("mpc_nav")
    tb3_dir = get_package_share_directory("turtlebot3_gazebo")

    # ── Arguments ────────────────────────────────────────────────────────────
    spawn_obstacles_arg = DeclareLaunchArgument(
        "spawn_obstacles", default_value="false",
        description="Automatically spawn dynamic obstacles for testing"
    )
    waypoint_file_arg = DeclareLaunchArgument(
        "waypoint_file",
        default_value=os.path.join(pkg_dir, "waypoints", "waypoint.csv"),
        description="Absolute path to waypoints CSV file"
    )

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_dir, "launch", "empty_world.launch.py")
        )
    )

    # ── Path Smoother ─────────────────────────────────────────────────────────
    path_smoother = Node(
        package="mpc_nav",
        executable="path_smoother",
        name="path_smoother",
        parameters=[{
            "path_file": LaunchConfiguration("waypoint_file"),
            "path_resolution": 0.05,
            "frame_id": "odom",
            "publish_rate": 1.0,
        }],
        output="screen",
    )

    # ── MPC Tracker ───────────────────────────────────────────────────────────
    mpc_tracker = Node(
        package="mpc_nav",
        executable="mpc_tracker",
        name="mpc_tracker",
        parameters=[{
            "v_max": 0.3,
            "v_min": -0.05,
            "w_max": 1.5,
            "horizon": 15,
            "dt": 0.1,
            "Q_pos": 10.0,
            "Q_head": 2.0,
            "R_v": 0.5,
            "R_w": 0.5,
            "terminal_weight": 20.0,
            "obs_weight": 100.0,    # High weight pushes robot around obstacles
            "obs_margin": 0.45,
            "goal_tolerance": 0.15,
            "lidar_range_max": 2.0,
            "obstacle_stop_dist": 0.35,
            "control_frequency": 10.0,
        }],
        output="screen",
    )

    # ── Dynamic Obstacle Spawner (optional) ───────────────────────────────────
    obstacle_spawner = Node(
        package="mpc_nav",
        executable="obstacle_spawner",
        name="obstacle_spawner",
        condition=IfCondition(LaunchConfiguration("spawn_obstacles")),
        parameters=[{
            "spawn_interval": 10.0,
            "obstacle_dist": 0.9,
            "remove_after": 8.0,
            "max_obstacles": 2,
        }],
        output="screen",
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, "config", "mpc_nav.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        output="screen",
    )

    return LaunchDescription([
        spawn_obstacles_arg,
        waypoint_file_arg,
        gazebo,
        # Delay other nodes to let Gazebo start
        TimerAction(period=5.0, actions=[path_smoother]),
        TimerAction(period=6.0, actions=[mpc_tracker]),
        TimerAction(period=7.0, actions=[obstacle_spawner]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
