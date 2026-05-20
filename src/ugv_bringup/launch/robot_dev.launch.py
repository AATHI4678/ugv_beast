"""
Development launch — desktop testing without hardware.

Starts:
  - mock_phone (WebSocket server with simulated IMU/GPS data)
  - motor_driver in sim_mode (integrates cmd_vel for odometry)
  - full localization stack
  - Nav2
  - RViz

Usage:
  ros2 launch ugv_bringup robot_dev.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def pkg(name):
    return get_package_share_directory(name)


def generate_launch_description():
    rviz_config = os.path.join(pkg('ugv_bringup'), 'config', 'ugv.rviz')

    mock_phone = Node(
        package='phone_sensor_bridge',
        executable='mock_phone',
        name='mock_phone',
        output='screen',
        parameters=[{
            'port': 2343,
            'lat': 44.305488,
            'lon': -79.574232,
            'alt': 234.0,
            'imu_hz': 50.0,
            'gps_hz': 1.0,
            'simulate_motion': True,
        }],
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg('phone_sensor_bridge'), 'launch',
                         'phone_sensor_bridge.launch.py')),
        launch_arguments=[('phone_ip', '127.0.0.1')],
    )

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg('ugv_base'), 'launch', 'ugv_base.launch.py')),
        launch_arguments=[('sim_mode', 'true')],
    )

    localization = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg('ugv_localization'), 'launch',
                             'localization.launch.py')))],
    )

    navigation = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg('ugv_navigation'), 'launch',
                             'navigation.launch.py')),
            launch_arguments=[('use_sim_time', 'false')])],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        mock_phone, bridge, base, localization, navigation, rviz,
    ])
