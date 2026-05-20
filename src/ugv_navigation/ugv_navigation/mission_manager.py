#!/usr/bin/env python3
"""
Delivery mission manager — state machine for the UGV Beast delivery robot.

States:
  IDLE             → waiting for a mission
  CONVERTING       → calling navsat_transform fromLL to get map-frame poses
  NAVIGATING       → Nav2 FollowWaypoints action in progress
  WAITING          → paused at a waypoint (configurable dwell)
  RETURNING_HOME   → autonomous return-to-home (low battery or mission complete)
  EMERGENCY_STOP   → e-stop asserted by watchdog or operator

Inputs:
  /set_delivery_waypoints  (ugv_interfaces/srv/SetDeliveryWaypoints)
  /e_stop                  (std_msgs/Bool)
  /battery_state           (ugv_interfaces/BatteryState)
  /localization_ok         (std_msgs/Bool)
  /teleop_active           (std_msgs/Bool)

Outputs:
  /delivery_status         (ugv_interfaces/DeliveryStatus)
  /cmd_vel/nav2            (geometry_msgs/Twist) — via Nav2 action
  /e_stop                  (std_msgs/Bool) — can assert on critical battery
"""

import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import FollowWaypoints
from robot_localization.srv import FromLL

from ugv_interfaces.msg import DeliveryStatus, BatteryState
from ugv_interfaces.srv import SetDeliveryWaypoints, EmergencyStop


class MissionState:
    IDLE = 0
    CONVERTING = 1
    NAVIGATING = 2
    WAITING = 3
    RETURNING_HOME = 4
    EMERGENCY_STOP = 7
    FAULT = 8


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        self.declare_parameter('home_altitude', 0.0)
        self.declare_parameter('arrival_tolerance_m', 2.0)
        self.declare_parameter('waypoint_dwell_s', 3.0)
        self.declare_parameter('low_battery_return_home', True)
        self.declare_parameter('nav2_timeout_s', 300.0)

        p = self.get_parameter
        self.home_lat = p('home_latitude').value
        self.home_lon = p('home_longitude').value
        self.home_alt = p('home_altitude').value
        self.arr_tol = p('arrival_tolerance_m').value
        self.dwell = p('waypoint_dwell_s').value
        self.auto_rth = p('low_battery_return_home').value

        # --- State ---
        self.state = MissionState.IDLE
        self.mission_id = ''
        self.waypoint_ids = []
        self.waypoint_poses = []
        self.current_wp_idx = 0
        self.e_stop_external = False
        self.localization_ok = True
        self.teleop_active = False
        self.battery_percent = 100.0
        self.battery_critical = False

        self._nav_handle = None
        self._nav_future = None

        cb = ReentrantCallbackGroup()

        # --- Publishers ---
        self.status_pub = self.create_publisher(DeliveryStatus, '/delivery_status', 10)
        self.estop_pub = self.create_publisher(Bool, '/e_stop', 10)

        # --- Services ---
        self.create_service(
            SetDeliveryWaypoints, '/set_delivery_waypoints',
            self._set_waypoints_cb, callback_group=cb)
        self.create_service(
            EmergencyStop, '/emergency_stop',
            self._emergency_stop_cb, callback_group=cb)

        # --- Subscribers ---
        self.create_subscription(Bool, '/e_stop', self._estop_cb, 10)
        self.create_subscription(Bool, '/localization_ok', self._lok_cb, 10)
        self.create_subscription(Bool, '/teleop_active', self._teleop_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self._batt_cb, 10)

        # --- Nav2 action client ---
        self._nav_client = ActionClient(
            self, FollowWaypoints, '/follow_waypoints', callback_group=cb)

        # --- navsat_transform fromLL service client ---
        self._fromll_client = self.create_client(
            FromLL, '/navsat_transform_node/fromLL', callback_group=cb)

        # --- Timers ---
        self.create_timer(0.5, self._status_timer)
        self.create_timer(0.1, self._state_machine)

        self.get_logger().info('mission_manager ready')

    # ----------------------------------------------------------------- service callbacks

    def _set_waypoints_cb(self, request, response):
        if self.state not in (MissionState.IDLE, MissionState.FAULT):
            response.accepted = False
            response.message = f'Mission in progress (state={self.state}); stop first'
            return response

        n = len(request.latitudes)
        if n == 0 or n != len(request.longitudes):
            response.accepted = False
            response.message = 'Invalid waypoint count'
            return response

        self.mission_id = request.mission_id
        self.waypoint_ids = list(request.waypoint_ids)
        self._pending_lats = list(request.latitudes)
        self._pending_lons = list(request.longitudes)
        self._pending_alts = list(request.altitudes) if request.altitudes else [0.0] * n
        self.arr_tol = request.arrival_tolerance_m or self.arr_tol
        self.current_wp_idx = 0
        self.waypoint_poses = []
        self.state = MissionState.CONVERTING

        response.accepted = True
        response.message = f'Mission {self.mission_id} accepted ({n} waypoints)'
        self.get_logger().info(response.message)
        return response

    def _emergency_stop_cb(self, request, response):
        if request.stop:
            self.state = MissionState.EMERGENCY_STOP
            msg = Bool()
            msg.data = True
            self.estop_pub.publish(msg)
            response.success = True
            response.message = f'E-STOP engaged: {request.reason}'
            self.get_logger().error(response.message)
        else:
            if self.state == MissionState.EMERGENCY_STOP:
                self.state = MissionState.IDLE
            msg = Bool()
            msg.data = False
            self.estop_pub.publish(msg)
            response.success = True
            response.message = 'E-STOP released'
            self.get_logger().info(response.message)
        return response

    # ----------------------------------------------------------------- topic callbacks

    def _estop_cb(self, msg: Bool):
        self.e_stop_external = bool(msg.data)
        if self.e_stop_external and self.state != MissionState.EMERGENCY_STOP:
            self.state = MissionState.EMERGENCY_STOP
            self._cancel_nav()

    def _lok_cb(self, msg: Bool):
        self.localization_ok = bool(msg.data)

    def _teleop_cb(self, msg: Bool):
        self.teleop_active = bool(msg.data)

    def _batt_cb(self, msg: BatteryState):
        self.battery_percent = float(msg.percent)
        self.battery_critical = bool(msg.critical_battery)
        if self.battery_critical and self.state == MissionState.NAVIGATING:
            self.get_logger().warn('Critical battery during mission; triggering RTH')
            self._cancel_nav()
            self.state = MissionState.RETURNING_HOME

    # ----------------------------------------------------------------- state machine

    def _state_machine(self):
        if self.state == MissionState.CONVERTING:
            self._do_convert()
        elif self.state == MissionState.NAVIGATING:
            self._do_navigate()
        elif self.state == MissionState.RETURNING_HOME:
            self._do_return_home()

    def _do_convert(self):
        """Convert all GPS waypoints to map-frame PoseStamped via navsat fromLL."""
        if not self._fromll_client.service_is_ready():
            return  # navsat not ready yet; keep waiting

        if len(self.waypoint_poses) < len(self._pending_lats):
            idx = len(self.waypoint_poses)
            req = FromLL.Request()
            req.ll_point.latitude = self._pending_lats[idx]
            req.ll_point.longitude = self._pending_lons[idx]
            req.ll_point.altitude = self._pending_alts[idx]

            future = self._fromll_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is None:
                self.get_logger().error(f'fromLL failed for waypoint {idx}')
                self.state = MissionState.FAULT
                return

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = future.result().map_point.x
            pose.pose.position.y = future.result().map_point.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0   # heading: let Nav2 choose approach angle
            self.waypoint_poses.append(pose)
            self.get_logger().info(
                f'Converted waypoint {idx} ({self._pending_lats[idx]:.6f}, '
                f'{self._pending_lons[idx]:.6f}) → map ({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f})')
            return  # process one per cycle to avoid blocking

        # All waypoints converted; start navigation
        self.get_logger().info(
            f'All {len(self.waypoint_poses)} waypoints converted; sending to Nav2')
        self._send_waypoints()

    def _send_waypoints(self):
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 FollowWaypoints action server not available')
            self.state = MissionState.FAULT
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoint_poses

        self._nav_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)
        self._nav_future.add_done_callback(self._nav_goal_response_cb)
        self.state = MissionState.NAVIGATING

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected waypoint goal')
            self.state = MissionState.FAULT
            return
        self._nav_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback):
        self.current_wp_idx = feedback.feedback.current_waypoint

    def _nav_result_cb(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)
        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
        else:
            self.get_logger().info('Mission complete — all waypoints reached')

        if self.auto_rth and self.home_lat != 0.0:
            self.state = MissionState.RETURNING_HOME
        else:
            self.state = MissionState.IDLE

    def _do_navigate(self):
        pass  # nav result callback drives state transitions

    def _do_return_home(self):
        if self.home_lat == 0.0 and self.home_lon == 0.0:
            self.get_logger().warn('No home position set; going IDLE')
            self.state = MissionState.IDLE
            return

        # Synthesize a single-waypoint mission to home
        home_req = SetDeliveryWaypoints.Request()
        home_req.mission_id = f'rth_{int(time.time())}'
        home_req.waypoint_ids = ['home']
        home_req.latitudes = [self.home_lat]
        home_req.longitudes = [self.home_lon]
        home_req.altitudes = [self.home_alt]
        home_req.arrival_tolerance_m = 2.0

        self._pending_lats = [self.home_lat]
        self._pending_lons = [self.home_lon]
        self._pending_alts = [self.home_alt]
        self.waypoint_poses = []
        self.mission_id = home_req.mission_id
        self.waypoint_ids = home_req.waypoint_ids
        # Switch to CONVERTING to reuse the conversion + navigation path
        self.state = MissionState.CONVERTING

    def _cancel_nav(self):
        if self._nav_handle:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None

    # ----------------------------------------------------------------- status publisher

    def _status_timer(self):
        msg = DeliveryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.mission_id = self.mission_id
        msg.current_waypoint_id = (
            self.waypoint_ids[self.current_wp_idx]
            if self.current_wp_idx < len(self.waypoint_ids) else '')
        msg.waypoints_remaining = max(
            0, len(self.waypoint_poses) - self.current_wp_idx)
        msg.battery_percent = self.battery_percent
        msg.teleop_override_active = self.teleop_active
        msg.status_message = self._state_label()
        self.status_pub.publish(msg)

    def _state_label(self) -> str:
        labels = {
            MissionState.IDLE: 'IDLE',
            MissionState.CONVERTING: 'CONVERTING_WAYPOINTS',
            MissionState.NAVIGATING: 'NAVIGATING',
            MissionState.WAITING: 'WAITING_AT_WAYPOINT',
            MissionState.RETURNING_HOME: 'RETURNING_HOME',
            MissionState.EMERGENCY_STOP: 'EMERGENCY_STOP',
            MissionState.FAULT: 'FAULT',
        }
        return labels.get(self.state, 'UNKNOWN')


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
