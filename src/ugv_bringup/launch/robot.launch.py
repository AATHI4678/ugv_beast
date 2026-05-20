"""
Master launch file — brings up the complete UGV Beast delivery robot stack.

Launch order (dependencies):
  1. ugv_base          — motor driver, battery monitor, teleop watchdog
  2. phone_sensor_bridge — WebSocket client (phone IMU + GPS)
  3. ugv_perception    — RPLIDAR C1 driver + laser filter chain
  4. ugv_localization  — static TF, dual EKF, navsat_transform, watchdog
  5. ugv_navigation    — Nav2 + mission manager

Arguments:
  phone_ip         Phone app IP address (default: 192.168.4.2 for Pi hotspot)
  home_latitude    Home GPS latitude (for return-to-home)
  home_longitude   Home GPS longitude
  sim_mode         Set true for desktop testing without hardware
  serial_port      RPLIDAR serial port (default: /dev/rplidar udev symlink)

Usage:
  ros2 launch ugv_bringup robot.launch.py phone_ip:=192.168.4.2 \\
    home_latitude:=44.305488 home_longitude:=-79.574232
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def pkg(name):
    return get_package_share_directory(name)


def launch(package, filename, **kwargs):
    """Helper: include a launch file with optional args."""
    extra = []
    for k, v in kwargs.items():
        if hasattr(v, '__iter__') and not isinstance(v, str):
            extra.append((k, v))
        else:
            extra.append((k, str(v)))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg(package), 'launch', filename)),
        launch_arguments=extra,
    )


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('phone_ip', default_value='192.168.4.2'),
        DeclareLaunchArgument('home_latitude', default_value='0.0'),
        DeclareLaunchArgument('home_longitude', default_value='0.0'),
        DeclareLaunchArgument('sim_mode', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/rplidar'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # ── Stage 1: Hardware drivers ───────────────────────────────────────
    base = launch('ugv_base', 'ugv_base.launch.py',
                  sim_mode=LaunchConfiguration('sim_mode'))

    phone = launch('phone_sensor_bridge', 'phone_sensor_bridge.launch.py',
                   phone_ip=LaunchConfiguration('phone_ip'))

    perception = launch('ugv_perception', 'perception.launch.py',
                        serial_port=LaunchConfiguration('serial_port'))

    # ── Stage 2: Localization (delay 3s to let hardware come up) ───────
    localization = TimerAction(
        period=3.0,
        actions=[launch('ugv_localization', 'localization.launch.py')],
    )

    # ── Stage 3: Navigation (delay 8s to let EKF converge) ─────────────
    navigation = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='Starting Nav2 (EKF should be converged by now)'),
            launch('ugv_navigation', 'navigation.launch.py',
                   home_latitude=LaunchConfiguration('home_latitude'),
                   home_longitude=LaunchConfiguration('home_longitude'),
                   use_sim_time=LaunchConfiguration('use_sim_time')),
        ],
    )

    return LaunchDescription(args + [base, phone, perception, localization, navigation])
