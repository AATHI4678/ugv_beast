#!/usr/bin/env python3
"""
Phone sensor bridge — GPS only (WebSocket client).

The robot now uses its onboard IMU (via ESP32 UART, published by motor_driver).
This node only connects to the phone's GPS WebSocket route.

Phone app runs a WebSocket SERVER:
  ws://<phone_ip>:2343/GPS  → streams GPS JSON at ~1 Hz

GPS JSON format (from PYNAVIGATION.md):
  {
    "type": "location",
    "latitude": 44.30548735, "longitude": -79.57423129, "altitude": 234.05,
    "accuracy": 4.28,          # Android 68% confidence radius in metres
    "speed": 0,
    "provider": "gps",
    "timestamp": 1778166934436
  }

Publishes:
  /gps/fix           sensor_msgs/NavSatFix   @ ~1 Hz
  /phone/wifi_status ugv_interfaces/WifiStatus
  /diagnostics       diagnostic_msgs/DiagnosticArray

NOT published by this node (now comes from motor_driver via ESP32):
  /imu/data          → motor_driver.py
"""

import asyncio
import json
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from ugv_interfaces.msg import WifiStatus

try:
    import websockets

    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False


class PhoneSensorBridge(Node):
    """
    Connects to the phone's GPS WebSocket and republishes as sensor_msgs/NavSatFix.

    A single asyncio task runs in a background thread with exponential-backoff
    reconnection so WiFi dropouts are handled without crashing the node.
    """

    def __init__(self):
        super().__init__("phone_sensor_bridge")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("phone_ip", "192.168.4.2")
        self.declare_parameter("phone_port", 2343)
        self.declare_parameter("gps_route", "/GPS")
        self.declare_parameter("gps_frame_id", "gps")
        self.declare_parameter("reconnect_base_delay_s", 2.0)
        self.declare_parameter("reconnect_max_delay_s", 30.0)
        self.declare_parameter("connection_timeout_s", 10.0)

        p = self.get_parameter
        self.phone_ip = p("phone_ip").value
        self.phone_port = p("phone_port").value
        self.gps_route = p("gps_route").value
        self.gps_frame = p("gps_frame_id").value
        self.reconnect_base = p("reconnect_base_delay_s").value
        self.reconnect_max = p("reconnect_max_delay_s").value
        self.conn_timeout = p("connection_timeout_s").value

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.gps_count = 0
        self.parse_errors = 0
        self.gps_connected = False
        self.gps_reconnects = 0
        self._last_gps_time = 0.0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.wifi_pub = self.create_publisher(WifiStatus, "/phone/wifi_status", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.create_timer(5.0, self._log_stats)
        self.create_timer(2.0, self._publish_wifi_status)
        self.create_timer(1.0, self._publish_diagnostics)

        # ------------------------------------------------------------------
        # Background asyncio thread
        # ------------------------------------------------------------------
        if not WS_AVAILABLE:
            self.get_logger().error(
                "python3-websockets not installed! "
                "Run: pip3 install websockets  or  apt install python3-websockets"
            )
            return

        self._loop = asyncio.new_event_loop()
        self._ws_thread = threading.Thread(target=self._run_asyncio_loop, daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            f"phone_sensor_bridge started (GPS only) | "
            f"phone={self.phone_ip}:{self.phone_port}{self.gps_route}"
        )

    # ------------------------------------------------------------------
    # Asyncio
    # ------------------------------------------------------------------

    def _run_asyncio_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_client_loop())

    async def _ws_client_loop(self):
        uri = f"ws://{self.phone_ip}:{self.phone_port}{self.gps_route}"
        delay = self.reconnect_base

        while True:
            try:
                self.get_logger().info(f"[GPS] Connecting to {uri}")
                async with websockets.connect(
                    uri,
                    open_timeout=self.conn_timeout,
                    ping_interval=20,
                    ping_timeout=10,
                ) as ws:
                    self.gps_connected = True
                    delay = self.reconnect_base
                    self.get_logger().info("[GPS] Connected")
                    async for raw in ws:
                        self._handle_message(raw)

            except Exception as e:
                self.gps_connected = False
                self.gps_reconnects += 1
                self.get_logger().warn(
                    f"[GPS] Disconnected ({type(e).__name__}: {e}); "
                    f"retry in {delay:.0f}s"
                )

            await asyncio.sleep(delay)
            delay = min(delay * 2.0, self.reconnect_max)

    # ------------------------------------------------------------------
    # Message handling
    # ------------------------------------------------------------------

    def _handle_message(self, raw):
        try:
            msg = json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.parse_errors += 1
            return

        if msg.get("type") == "location":
            self._publish_gps(msg)
        else:
            self.parse_errors += 1

    def _publish_gps(self, msg: dict):
        try:
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = self.gps_frame

            fix.latitude = float(msg["latitude"])
            fix.longitude = float(msg["longitude"])
            fix.altitude = float(msg.get("altitude", 0.0))

            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS

            # Android accuracy is the 68% confidence radius in metres.
            # variance = radius² gives a reasonable 1-sigma approximation.
            acc = float(msg.get("accuracy", 5.0))
            var_h = acc * acc
            var_v = (acc * 2.5) ** 2  # vertical always worse than horizontal
            fix.position_covariance = [
                var_h,
                0.0,
                0.0,
                0.0,
                var_h,
                0.0,
                0.0,
                0.0,
                var_v,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            self.gps_pub.publish(fix)
            self.gps_count += 1
            self._last_gps_time = time.time()

        except (KeyError, TypeError, ValueError) as e:
            self.parse_errors += 1
            self.get_logger().warn(f"GPS parse error: {e}", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def _log_stats(self):
        pass
        # self.get_logger().info(
        #     f'GPS={self.gps_count} ParseErr={self.parse_errors} '
        #     f'Reconnects={self.gps_reconnects} Connected={self.gps_connected}')

    def _publish_wifi_status(self):
        ws = WifiStatus()
        ws.header.stamp = self.get_clock().now().to_msg()
        ws.connected = self.gps_connected
        ws.reconnect_count = self.gps_reconnects
        self.wifi_pub.publish(ws)

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        s = DiagnosticStatus()
        s.name = "phone_gps_ws"
        s.hardware_id = f"phone@{self.phone_ip}"
        age = time.time() - self._last_gps_time if self._last_gps_time > 0 else 9999.0
        s.values = [
            KeyValue(key="connected", value=str(self.gps_connected)),
            KeyValue(key="gps_count", value=str(self.gps_count)),
            KeyValue(key="reconnects", value=str(self.gps_reconnects)),
            KeyValue(key="last_msg_age_s", value=f"{age:.1f}"),
            KeyValue(key="parse_errors", value=str(self.parse_errors)),
        ]
        if not self.gps_connected:
            s.level = DiagnosticStatus.ERROR
            s.message = "GPS WebSocket disconnected"
        elif age > 10.0:
            s.level = DiagnosticStatus.WARN
            s.message = f"No GPS data for {age:.0f}s"
        else:
            s.level = DiagnosticStatus.OK
            s.message = f"OK ({self.gps_count} fixes)"
        arr.status.append(s)
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    node = PhoneSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
