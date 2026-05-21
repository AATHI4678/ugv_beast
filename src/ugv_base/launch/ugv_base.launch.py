import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("ugv_base")
    params = os.path.join(pkg, "config", "motor_params.yaml")

    sim_arg = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Run without hardware (integrates cmd_vel for odom)",
    )

    motor_node = Node(
        package="ugv_base",
        executable="motor_driver",
        name="motor_driver",
        parameters=[
            params,
            {
                "sim_mode": ParameterValue(
                    LaunchConfiguration("sim_mode"), value_type=bool
                )
            },
        ],
        output="screen",
        # Remapping: Nav2 publishes to /cmd_vel/nav2; teleop_watchdog muxes to /cmd_vel
        remappings=[],
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        output="screen",
        parameters=[
            {
                "video_device": "/dev/camera_ugv",
                "pixel_format": "mjpeg2rgb",
                "image_width": 640,
                "image_height": 480,
                "framerate": 15.0,
                "frame_id": "camera_link",
                "camera_name": "ugv_camera",
            }
        ],
    )

    battery_node = Node(
        package="ugv_base",
        executable="battery_monitor",
        name="battery_monitor",
        parameters=[params],
        output="screen",
    )

    teleop_node = Node(
        package="ugv_base",
        executable="teleop_watchdog",
        name="teleop_watchdog",
        parameters=[params],
        output="screen",
    )

    return LaunchDescription(
        [sim_arg, motor_node, battery_node, teleop_node, camera_node]
    )
