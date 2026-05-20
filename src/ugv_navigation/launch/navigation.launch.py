import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_bringup.launch import bringup_launch


def generate_launch_description():
    pkg_nav = get_package_share_directory('ugv_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    use_sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    home_lat_arg = DeclareLaunchArgument(
        'home_latitude', default_value='0.0',
        description='Home GPS latitude (for return-to-home)')
    home_lon_arg = DeclareLaunchArgument(
        'home_longitude', default_value='0.0',
        description='Home GPS longitude (for return-to-home)')

    # ── Nav2 bringup (standard stack) ────────────────────────────────────
    # Uses Jazzy nav2_bringup. No map_server launch (outdoor, no static map).
    nav2 = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', 'cmd_vel_smoothed')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_smoothed'),
                ('cmd_vel_smoothed', '/cmd_vel/nav2'),
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                    'collision_monitor',
                ],
            }],
        ),
        # Costmap nodes (local + global)
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[nav2_params],
        ),
    ])

    # ── Mission manager ───────────────────────────────────────────────────
    mission = Node(
        package='ugv_navigation',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'home_latitude': LaunchConfiguration('home_latitude'),
            'home_longitude': LaunchConfiguration('home_longitude'),
            'home_altitude': 0.0,
            'arrival_tolerance_m': 2.0,
            'waypoint_dwell_s': 3.0,
            'low_battery_return_home': True,
        }],
    )

    return LaunchDescription([
        use_sim_arg, home_lat_arg, home_lon_arg,
        nav2, mission,
    ])
