#!/usr/bin/env python3
"""
Mock phone server for desktop testing — GPS only.

The robot now uses its onboard IMU (ESP32 UART). This mock server only
serves the GPS WebSocket route, matching what the real phone app provides.

  ws://0.0.0.0:2343/GPS  → streams fake GPS JSON at 1 Hz

Usage:
  ros2 run phone_sensor_bridge mock_phone [--ros-args -p lat:=44.305 -p lon:=-79.574]

Then set phone_ip to 127.0.0.1 in bridge parameters.
For IMU simulation in desktop testing, use motor_driver sim_mode which
integrates cmd_vel and publishes /wheel/odom + fake /imu/data is not
produced — wire up a separate fake_imu node or the robot_localization
will work from wheel odom alone in sim.
"""

import asyncio
import json
import math
import time

import rclpy
from rclpy.node import Node

try:
    import websockets
    from websockets.server import serve
    WS_AVAILABLE = True
except ImportError:
    WS_AVAILABLE = False


class MockPhone(Node):
    def __init__(self):
        super().__init__('mock_phone')
        self.declare_parameter('port', 2343)
        self.declare_parameter('lat', 44.305488)
        self.declare_parameter('lon', -79.574232)
        self.declare_parameter('alt', 234.0)
        self.declare_parameter('gps_hz', 1.0)
        self.declare_parameter('simulate_motion', True)

        self.port = self.get_parameter('port').value
        self.lat = self.get_parameter('lat').value
        self.lon = self.get_parameter('lon').value
        self.alt = self.get_parameter('alt').value
        self.gps_hz = self.get_parameter('gps_hz').value
        self.simulate_motion = self.get_parameter('simulate_motion').value

        self._t = 0.0

        import threading
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        self.get_logger().info(
            f'mock_phone GPS server on port {self.port} (GPS only; IMU from robot)')

    def _gps_payload(self) -> str:
        # Slowly drift position to simulate movement
        dlat = 0.00001 * math.sin(self._t * 0.1)
        dlon = 0.00001 * math.cos(self._t * 0.1)
        return json.dumps({
            'type': 'location',
            'latitude': self.lat + dlat,
            'longitude': self.lon + dlon,
            'altitude': self.alt,
            'accuracy': 3.5,
            'speed': 0.2,
            'provider': 'gps',
            'timestamp': int(time.time() * 1000),
        })

    def _run(self):
        asyncio.run(self._server())

    async def _server(self):
        gps_clients = set()

        async def handler(ws):
            path = ws.request.path if hasattr(ws, 'request') else getattr(ws, 'path', '/')
            if path == '/GPS':
                gps_clients.add(ws)
                self.get_logger().info('GPS client connected')
                try:
                    await ws.wait_closed()
                finally:
                    gps_clients.discard(ws)

        async def gps_broadcaster():
            interval = 1.0 / self.gps_hz
            while True:
                if gps_clients:
                    payload = self._gps_payload()
                    dead = set()
                    for ws in list(gps_clients):
                        try:
                            await ws.send(payload)
                        except Exception:
                            dead.add(ws)
                    gps_clients -= dead
                await asyncio.sleep(interval)

        async with serve(handler, '0.0.0.0', self.port):
            await gps_broadcaster()


def main():
    rclpy.init()
    node = MockPhone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
