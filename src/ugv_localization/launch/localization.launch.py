import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ugv_localization')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')

    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'static_tf.launch.py')))

    # ── LOCAL EKF (odom frame) ───────────────────────────────────────────
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_yaml],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
            ('accel/filtered', '/accel/local'),
        ],
    )

    # ── GLOBAL EKF (map frame) ───────────────────────────────────────────
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_yaml],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
            ('accel/filtered', '/accel/global'),
        ],
    )

    # ── navsat_transform ─────────────────────────────────────────────────
    # Converts /gps/fix → /odometry/gps (map-frame Odometry)
    # This feeds the global EKF as its GPS measurement source.
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_yaml],
        remappings=[
            ('imu/data',          '/imu/data'),
            ('gps/fix',           '/gps/fix'),
            ('gps/filtered',      '/gps/filtered'),
            ('odometry/gps',      '/odometry/gps'),
            ('odometry/filtered', '/odometry/global'),  # must match global EKF output
        ],
    )

    # ── Localization watchdog ─────────────────────────────────────────────
    watchdog = Node(
        package='ugv_localization',
        executable='localization_watchdog',
        name='localization_watchdog',
        output='screen',
        parameters=[{
            'imu_timeout_s': 2.0,
            'gps_timeout_s': 10.0,
            'odom_timeout_s': 2.0,
            'max_position_cov': 25.0,
            'estop_on_imu_loss': True,
            'estop_on_ekf_diverge': True,
        }],
    )

    return LaunchDescription([static_tf, local_ekf, global_ekf, navsat, watchdog])
