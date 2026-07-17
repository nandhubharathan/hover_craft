#!/usr/bin/env python3
"""
hover_sim.launch.py — ROS 2 Launch File for Hovercraft Simulation
==================================================================
Launches:
  1. Gazebo with the hover_lab world (low-friction floor + obstacles)
  2. Spawns the hovercraft URDF model at the origin
  3. Starts the SITL bridge node

Usage:
  ros2 launch hover_sim.launch.py
  # or from the simulation directory:
  python3 hover_sim.launch.py   (via ros2 launch)

Prerequisites:
  - ROS 2 (Humble / Iron / Jazzy)
  - gazebo_ros_pkgs
  - pip install ros2launch  (usually bundled with ROS 2)
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Paths ────────────────────────────────────────────────────
    sim_dir = Path(__file__).resolve().parent
    urdf_path = sim_dir / 'hovercraft.urdf'
    world_path = sim_dir / 'world' / 'hover_lab.world'
    bridge_path = sim_dir / 'sitl_bridge.py'

    # Read URDF as string for robot_state_publisher
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ── Launch Arguments ─────────────────────────────────────────
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock')

    verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Enable verbose Gazebo output')

    # ── Gazebo Server + Client ───────────────────────────────────
    gazebo_server = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose' if True else '',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            str(world_path),
        ],
        output='screen',
    )

    # ── Robot State Publisher ────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='hovercraft',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── Spawn URDF Entity in Gazebo ──────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_hovercraft',
        output='screen',
        arguments=[
            '-entity', 'hovercraft',
            '-file', str(urdf_path),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05',   # slightly above ground
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
    )

    # ── SITL Bridge Node (delayed 3s to let Gazebo start) ────────
    sitl_bridge = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', str(bridge_path)],
                output='screen',
                prefix='xterm -e' if os.environ.get('DISPLAY') else '',
            ),
        ],
    )

    # ── Assemble ─────────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time,
        verbose,
        gazebo_server,
        robot_state_pub,
        spawn_entity,
        sitl_bridge,
    ])
