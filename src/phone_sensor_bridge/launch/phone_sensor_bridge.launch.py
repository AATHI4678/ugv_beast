import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('phone_sensor_bridge')
    params = os.path.join(pkg, 'config', 'bridge_params.yaml')

    phone_ip_arg = DeclareLaunchArgument(
        'phone_ip', default_value='192.168.4.2',
        description='IP address of phone running the sensor app')

    bridge_node = Node(
        package='phone_sensor_bridge',
        executable='bridge',
        name='phone_sensor_bridge',
        parameters=[params, {'phone_ip': LaunchConfiguration('phone_ip')}],
        output='screen',
    )

    return LaunchDescription([phone_ip_arg, bridge_node])
