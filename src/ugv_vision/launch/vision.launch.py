"""
vision.launch.py
Launches the vision_server node with its parameter file.
Can be included standalone or from ugv_bringup/robot.launch.py.

Usage:
  ros2 launch ugv_vision vision.launch.py
  ros2 launch ugv_vision vision.launch.py camera_device:=/dev/video1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    params_file = PathJoinSubstitution([
        FindPackageShare("ugv_vision"), "config", "vision_params.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video0",
            description="V4L2 camera device path",
        ),
        DeclareLaunchArgument(
            "flask_port",
            default_value="5000",
            description="Port for the HTTP MJPEG stream and /control endpoint",
        ),

        Node(
            package="ugv_vision",
            executable="vision_server",
            name="vision_server",
            output="screen",
            parameters=[
                params_file,
                {
                    "camera_device": LaunchConfiguration("camera_device"),
                    "flask_port":    LaunchConfiguration("flask_port"),
                },
            ],
        ),
    ])
