#!/usr/bin/env python3
"""
Battery monitor node for UGV Beast.

Reads battery voltage/current from the ESP32 telemetry serial stream
(same port as motor_driver, but this node subscribes to a shared ROS topic
published by the motor_driver to avoid port conflicts).

Alternatively, if a dedicated INA219 is wired to the Pi's I2C bus, this
node can read it directly (set use_i2c: true).

Publishes:
  /battery_state        ugv_interfaces/BatteryState  @ 1 Hz
  /diagnostics          diagnostic_msgs/DiagnosticArray

Thresholds (configurable):
  warning  < 20% → log + slow down recommendation
  low      < 15% → trigger return-to-home
  critical < 10% → emergency stop
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ugv_interfaces.msg import BatteryState

# 3S 18650 LiPo typical: 12.6V full, 9.0V cutoff
CELL_COUNT = 3
VOLT_FULL = 4.2 * CELL_COUNT    # 12.6 V
VOLT_EMPTY = 3.0 * CELL_COUNT   # 9.0 V


def voltage_to_percent(v: float) -> float:
    pct = (v - VOLT_EMPTY) / (VOLT_FULL - VOLT_EMPTY) * 100.0
    return max(0.0, min(100.0, pct))


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('warn_percent', 20.0)
        self.declare_parameter('low_percent', 15.0)
        self.declare_parameter('critical_percent', 10.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        # If True, read INA219 over I2C directly (requires smbus2 + pi-ina219)
        self.declare_parameter('use_i2c', False)
        self.declare_parameter('i2c_address', 0x40)

        self.warn_pct = self.get_parameter('warn_percent').value
        self.low_pct = self.get_parameter('low_percent').value
        self.critical_pct = self.get_parameter('critical_percent').value
        self.use_i2c = self.get_parameter('use_i2c').value

        self.voltage = 0.0
        self.current = 0.0
        self.percent = 100.0

        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Motor driver publishes raw voltage on this topic
        self.create_subscription(Float32, '/motor/voltage_raw', self._volt_cb, 10)
        self.create_subscription(Float32, '/motor/current_raw', self._curr_cb, 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish)

        if self.use_i2c:
            self._init_ina219()

        self.get_logger().info('battery_monitor ready')

    def _init_ina219(self):
        try:
            import smbus2
            from ina219 import INA219
            addr = self.get_parameter('i2c_address').value
            self._ina = INA219(0.1, address=addr)
            self._ina.configure()
            self.create_timer(0.5, self._read_ina219)
            self.get_logger().info(f'INA219 connected at 0x{addr:02x}')
        except Exception as e:
            self.get_logger().error(f'INA219 init failed: {e}')

    def _read_ina219(self):
        try:
            self.voltage = self._ina.voltage()
            self.current = self._ina.current() / 1000.0  # mA -> A
            self.percent = voltage_to_percent(self.voltage)
        except Exception as e:
            self.get_logger().warn(f'INA219 read error: {e}')

    def _volt_cb(self, msg: Float32):
        self.voltage = float(msg.data)
        self.percent = voltage_to_percent(self.voltage)

    def _curr_cb(self, msg: Float32):
        self.current = float(msg.data)

    def _publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.voltage
        msg.current = self.current
        msg.percent = self.percent
        msg.low_battery_warning = self.percent < self.warn_pct
        msg.critical_battery = self.percent < self.critical_pct
        self.batt_pub.publish(msg)

        # Diagnostics
        arr = DiagnosticArray()
        arr.header.stamp = msg.header.stamp
        s = DiagnosticStatus()
        s.name = 'battery_monitor'
        s.hardware_id = 'ugv_beast_battery'
        s.values = [
            KeyValue(key='voltage_V', value=f'{self.voltage:.2f}'),
            KeyValue(key='current_A', value=f'{self.current:.2f}'),
            KeyValue(key='percent', value=f'{self.percent:.1f}'),
        ]
        if self.percent < self.critical_pct:
            s.level = DiagnosticStatus.ERROR
            s.message = f'CRITICAL battery: {self.percent:.0f}%'
        elif self.percent < self.low_pct:
            s.level = DiagnosticStatus.WARN
            s.message = f'LOW battery: {self.percent:.0f}%'
        elif self.percent < self.warn_pct:
            s.level = DiagnosticStatus.WARN
            s.message = f'Warning battery: {self.percent:.0f}%'
        else:
            s.level = DiagnosticStatus.OK
            s.message = f'OK {self.percent:.0f}%'
        arr.status.append(s)
        self.diag_pub.publish(arr)

        if self.percent < self.warn_pct:
            self.get_logger().warn(
                f'Battery {self.percent:.0f}% ({self.voltage:.2f}V)')


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
