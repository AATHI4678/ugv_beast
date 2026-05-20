#!/usr/bin/env python3
"""
GPS waypoint file loader and converter.

Reads a YAML mission file (see missions/example_mission.yaml),
converts GPS lat/lon to map-frame poses via navsat_transform fromLL,
and sends the mission to the mission_manager via SetDeliveryWaypoints service.

Usage:
  ros2 run ugv_navigation waypoint_converter \
    --ros-args -p mission_file:=/path/to/mission.yaml
"""

import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from ugv_interfaces.srv import SetDeliveryWaypoints


class WaypointConverter(Node):
    def __init__(self):
        super().__init__('waypoint_converter')
        self.declare_parameter('mission_file', '')

        mission_file = self.get_parameter('mission_file').value
        if not mission_file:
            self.get_logger().error('mission_file parameter is required')
            return

        if not os.path.exists(mission_file):
            self.get_logger().error(f'Mission file not found: {mission_file}')
            return

        with open(mission_file, 'r') as f:
            data = yaml.safe_load(f)

        mission = data.get('mission', {})
        waypoints = mission.get('waypoints', [])

        client = self.create_client(SetDeliveryWaypoints, '/set_delivery_waypoints')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('SetDeliveryWaypoints service not available')
            return

        req = SetDeliveryWaypoints.Request()
        req.mission_id = mission.get('id', 'mission_001')
        req.waypoint_ids = [str(wp.get('id', f'wp{i}')) for i, wp in enumerate(waypoints)]
        req.latitudes = [float(wp['lat']) for wp in waypoints]
        req.longitudes = [float(wp['lon']) for wp in waypoints]
        req.altitudes = [float(wp.get('alt', 0.0)) for wp in waypoints]
        req.arrival_tolerance_m = float(mission.get('arrival_tolerance_m', 2.0))

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        if future.result() and future.result().accepted:
            self.get_logger().info(f'Mission accepted: {future.result().message}')
        else:
            self.get_logger().error(
                f'Mission rejected: {future.result().message if future.result() else "timeout"}')


def main():
    rclpy.init()
    node = WaypointConverter()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
