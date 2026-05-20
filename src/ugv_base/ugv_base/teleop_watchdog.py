#!/usr/bin/env python3
"""
Teleop watchdog and cmd_vel multiplexer.

Manages two cmd_vel sources:
  - /cmd_vel/nav2     (autonomous navigation)
  - /cmd_vel/teleop   (operator override via joystick/keyboard)

Teleop override activates when:
  - A non-zero twist arrives on /cmd_vel/teleop
  - The /teleop_override topic is set True

Override expires after teleop_timeout_s of silence.

Outputs:
  /cmd_vel            geometry_msgs/Twist  (motor_driver input)
  /teleop_active      std_msgs/Bool
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from ugv_interfaces.msg import DeliveryStatus


class TeleopWatchdog(Node):
    def __init__(self):
        super().__init__('teleop_watchdog')

        self.declare_parameter('teleop_timeout_s', 2.0)
        self.declare_parameter('max_teleop_speed_mps', 0.35)
        self.declare_parameter('max_teleop_yaw_rps', 1.0)

        self.teleop_timeout = self.get_parameter('teleop_timeout_s').value
        self.max_speed = self.get_parameter('max_teleop_speed_mps').value
        self.max_yaw = self.get_parameter('max_teleop_yaw_rps').value

        self.teleop_active = False
        self.last_teleop_time = 0.0
        self.last_teleop_twist = Twist()
        self.last_nav_twist = Twist()

        # Outputs
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.override_pub = self.create_publisher(Bool, '/teleop_active', 10)

        # Inputs
        self.create_subscription(Twist, '/cmd_vel/nav2', self._nav_cb, 10)
        self.create_subscription(Twist, '/cmd_vel/teleop', self._teleop_cb, 10)
        self.create_subscription(Bool, '/teleop_override', self._override_cb, 10)

        self.create_timer(0.05, self._timer_cb)   # 20 Hz mux output
        self.get_logger().info('teleop_watchdog ready (mux: nav2 + teleop -> /cmd_vel)')

    def _nav_cb(self, msg: Twist):
        self.last_nav_twist = msg

    def _teleop_cb(self, msg: Twist):
        # Any non-trivial teleop command activates override
        speed = abs(msg.linear.x) + abs(msg.angular.z)
        if speed > 0.01:
            self.last_teleop_time = time.time()
            if not self.teleop_active:
                self.teleop_active = True
                self.get_logger().info('Teleop override ACTIVE')

        # Clamp teleop to safe limits
        msg.linear.x = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        msg.angular.z = max(-self.max_yaw, min(self.max_yaw, msg.angular.z))
        self.last_teleop_twist = msg

    def _override_cb(self, msg: Bool):
        if msg.data and not self.teleop_active:
            self.teleop_active = True
            self.last_teleop_time = time.time()
            self.get_logger().info('Teleop override forced ON via topic')
        elif not msg.data:
            self.teleop_active = False

    def _timer_cb(self):
        # Check timeout
        if self.teleop_active:
            if time.time() - self.last_teleop_time > self.teleop_timeout:
                self.teleop_active = False
                self.get_logger().info('Teleop override expired')

        out = self.last_teleop_twist if self.teleop_active else self.last_nav_twist
        self.cmd_pub.publish(out)

        flag = Bool()
        flag.data = self.teleop_active
        self.override_pub.publish(flag)


def main():
    rclpy.init()
    node = TeleopWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
