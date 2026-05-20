"""
Perception launch file.

Starts:
  1. RPLIDAR C1 driver (sllidar_ros2) → publishes /scan
  2. Outdoor laser filter chain (laser_filters) → publishes /scan_filtered

/scan_filtered is what Nav2 and the local costmap consume.

Prerequisites:
  sudo apt install ros-jazzy-laser-filters
  cd ~/ugv_ws/src && git clone https://github.com/Slamtec/sllidar_ros2.git
  colcon build --packages-select sllidar_ros2

udev rule for RPLIDAR C1 (run once):
  echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",
        MODE:="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("ugv_perception")
    filter_config = os.path.join(pkg, "config", "scan_filter.yaml")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/rplidar",
        description="Serial port for RPLIDAR C1 (udev symlink or /dev/ttyUSB0)",
    )

    # ── RPLIDAR C1 driver ────────────────────────────────────────────────
    # Uses sllidar_ros2 from Slamtec. Publishes /scan (sensor_msgs/LaserScan).
    # frame_id must match the TF static transform: 'laser'
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": 460800,  # C1 uses 460800 baud
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
        output="screen",
    )

    # ── Laser filter chain ───────────────────────────────────────────────
    # scan_to_scan_filter_chain reads the filter chain from parameters.
    # Input:  /scan         (raw from lidar)
    # Output: /scan_filtered (cleaned, fed to Nav2)
    filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_filter_chain",
        parameters=[filter_config],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
        output="screen",
    )

    return LaunchDescription([serial_port_arg, lidar_node, filter_node])
