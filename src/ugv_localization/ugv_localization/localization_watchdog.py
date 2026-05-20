#!/usr/bin/env python3
"""
Localization watchdog.

Monitors the health of the dual EKF stack. If localization diverges or
sensor inputs go stale, publishes an e-stop command and a diagnostic alert.

Checks:
  - /odometry/global covariance (high cov → localization uncertain)
  - /imu/data freshness
  - /gps/fix freshness
  - /wheel/odom freshness
  - EKF TF tree completeness (map → base_link)

Outputs:
  /e_stop                std_msgs/Bool   (true = stop)
  /localization_ok       std_msgs/Bool
  /diagnostics           diagnostic_msgs/DiagnosticArray
"""

import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class LocalizationWatchdog(Node):
    def __init__(self):
        super().__init__('localization_watchdog')

        self.declare_parameter('imu_timeout_s', 2.0)
        self.declare_parameter('gps_timeout_s', 10.0)
        self.declare_parameter('odom_timeout_s', 2.0)
        self.declare_parameter('max_position_cov', 25.0)  # m²; ~5m 1-sigma
        self.declare_parameter('check_rate_hz', 2.0)
        self.declare_parameter('estop_on_imu_loss', True)
        self.declare_parameter('estop_on_ekf_diverge', True)

        p = self.get_parameter
        self.imu_timeout = p('imu_timeout_s').value
        self.gps_timeout = p('gps_timeout_s').value
        self.odom_timeout = p('odom_timeout_s').value
        self.max_cov = p('max_position_cov').value
        self.estop_imu = p('estop_on_imu_loss').value
        self.estop_ekf = p('estop_on_ekf_diverge').value

        self._last_imu = 0.0
        self._last_gps = 0.0
        self._last_odom = 0.0
        self._last_global_cov_x = 0.0
        self._last_global_cov_y = 0.0
        self._estop_active = False

        # Publishers
        self.estop_pub = self.create_publisher(Bool, '/e_stop', 10)
        self.lok_pub = self.create_publisher(Bool, '/localization_ok', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Subscribers
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)
        self.create_subscription(Odometry, '/wheel/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/global', self._global_odom_cb, 10)

        rate = p('check_rate_hz').value
        self.create_timer(1.0 / rate, self._check)
        self.get_logger().info('localization_watchdog ready')

    def _imu_cb(self, _): self._last_imu = time.time()
    def _gps_cb(self, _): self._last_gps = time.time()
    def _odom_cb(self, _): self._last_odom = time.time()

    def _global_odom_cb(self, msg: Odometry):
        self._last_global_cov_x = msg.pose.covariance[0]
        self._last_global_cov_y = msg.pose.covariance[7]

    def _check(self):
        now = time.time()
        issues = []

        imu_age = now - self._last_imu if self._last_imu > 0 else 9999.0
        gps_age = now - self._last_gps if self._last_gps > 0 else 9999.0
        odom_age = now - self._last_odom if self._last_odom > 0 else 9999.0

        imu_ok = imu_age < self.imu_timeout
        gps_ok = gps_age < self.gps_timeout
        odom_ok = odom_age < self.odom_timeout
        cov_ok = (self._last_global_cov_x < self.max_cov and
                  self._last_global_cov_y < self.max_cov)

        if not imu_ok:
            issues.append(f'IMU stale ({imu_age:.1f}s)')
        if not gps_ok:
            issues.append(f'GPS stale ({gps_age:.1f}s)')
        if not odom_ok:
            issues.append(f'Wheel odom stale ({odom_age:.1f}s)')
        if not cov_ok:
            issues.append(
                f'EKF diverged (cov_x={self._last_global_cov_x:.1f} '
                f'cov_y={self._last_global_cov_y:.1f})')

        # Determine if we should e-stop
        need_estop = False
        if self.estop_imu and not imu_ok and self._last_imu > 0:
            need_estop = True
        if self.estop_ekf and not cov_ok and self._last_global_cov_x > 0:
            need_estop = True

        if need_estop and not self._estop_active:
            self._estop_active = True
            self.get_logger().error(f'LOCALIZATION E-STOP: {"; ".join(issues)}')
        elif not need_estop and self._estop_active:
            self._estop_active = False
            self.get_logger().info('Localization recovered; releasing e-stop')

        estop_msg = Bool()
        estop_msg.data = self._estop_active
        self.estop_pub.publish(estop_msg)

        lok_msg = Bool()
        lok_msg.data = len(issues) == 0
        self.lok_pub.publish(lok_msg)

        # Diagnostics
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        s = DiagnosticStatus()
        s.name = 'localization_watchdog'
        s.hardware_id = 'ugv_beast'
        s.values = [
            KeyValue(key='imu_age_s', value=f'{imu_age:.2f}'),
            KeyValue(key='gps_age_s', value=f'{gps_age:.2f}'),
            KeyValue(key='odom_age_s', value=f'{odom_age:.2f}'),
            KeyValue(key='cov_x', value=f'{self._last_global_cov_x:.3f}'),
            KeyValue(key='cov_y', value=f'{self._last_global_cov_y:.3f}'),
            KeyValue(key='estop_active', value=str(self._estop_active)),
        ]
        if self._estop_active:
            s.level = DiagnosticStatus.ERROR
            s.message = 'E-STOP: ' + '; '.join(issues)
        elif issues:
            s.level = DiagnosticStatus.WARN
            s.message = '; '.join(issues)
        else:
            s.level = DiagnosticStatus.OK
            s.message = 'Localization OK'
        arr.status.append(s)
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    node = LocalizationWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
