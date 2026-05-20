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
            {
                "sim_mode": ParameterValue(
                    LaunchConfiguration("sim_mode"), value_type=bool
                )
            }
        ],
        output="screen",
        # Remapping: Nav2 publishes to /cmd_vel/nav2; teleop_watchdog muxes to /cmd_vel
        remappings=[],
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

    return LaunchDescription([sim_arg, motor_node, battery_node, teleop_node])
