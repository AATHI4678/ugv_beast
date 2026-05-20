#!/usr/bin/env python3
"""
Motor driver node for WaveShare UGV Beast.

The UGV Beast uses a Pi 4B (high-level) + ESP32 (low-level PID controller).
Communication is over UART at 115200 baud using JSON.

Sources:
  https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2
  https://github.com/waveshareteam/ugv_base_ros

Serial protocol (Pi -> ESP32)  — Waveshare confirmed format:
  Motor drive:  {"T":1,"L":<float>,"R":<float>}
                T=1 is the drive command type.
                L/R are left/right speeds in the range [-1.0, +1.0]
                where ±1.0 maps to the firmware's max speed (~0.35 m/s).
                We normalise our m/s values to this [-1, 1] range.
  Stop:         {"T":1,"L":0,"R":0}
  Heartbeat:    {"T":6}   (keep-alive; ESP32 stops after 3 s of silence)

Serial protocol (ESP32 -> Pi)  — telemetry JSON lines:
  Odometry:  {"T":1001,"L":<float m/s>,"R":<float m/s>,"dt":<float s>}
  IMU:       {"T":1002,
               "ax":<float>,"ay":<float>,"az":<float>,   # m/s², WITH gravity
               "gx":<float>,"gy":<float>,"gz":<float>,   # rad/s
               "qw":<float>,"qx":<float>,"qy":<float>,"qz":<float>}  # optional (DMP)
  Battery:   {"T":1003,"V":<float V>,"C":<float A>}
  Fault:     {"T":9,"code":<int>,"msg":<str>}

IMU chip: ICM-20948 (9-axis) confirmed via Waveshare firmware (Adafruit_ICM20948).
  - ax/ay/az include gravity → EKF imu0_remove_gravitational_acceleration: true
  - qw/qx/qy/qz present only if DMP is enabled in firmware
  - If absent, orientation_covariance[0] = -1 (EKF integrates gyro for yaw)

Publishes:
  /wheel/odom           nav_msgs/Odometry
  /imu/data             sensor_msgs/Imu
  /motor/status         ugv_interfaces/MotorStatus
  /motor/voltage_raw    std_msgs/Float32
  /motor/current_raw    std_msgs/Float32
  /diagnostics          diagnostic_msgs/DiagnosticArray

Subscribes:
  /cmd_vel              geometry_msgs/Twist
  /e_stop               std_msgs/Bool
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ugv_interfaces.msg import MotorStatus

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class MotorDriver(Node):
    """
    Differential-drive motor driver + IMU publisher for the WaveShare UGV Beast.

    Owns the UART serial link to the ESP32 and dispatches all incoming
    message types: odometry, IMU, battery telemetry, and fault codes.
    """

    def __init__(self):
        super().__init__('motor_driver')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base_m', 0.295)
        self.declare_parameter('wheel_radius_m', 0.0525)
        self.declare_parameter('ticks_per_rev', 1560)
        self.declare_parameter('max_speed_mps', 0.35)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sim_mode', False)

        # IMU parameters
        self.declare_parameter('imu_frame_id', 'imu_link')
        # True if the ESP32 runs sensor fusion and sends qw/qx/qy/qz.
        # False (default) for raw 6-axis IMUs (MPU6050, ICM-42688, etc.)
        # where only accel + gyro are available.
        self.declare_parameter('imu_has_orientation', False)
        # Covariances — tune after measuring your specific IMU's noise floor.
        # These are conservative defaults for a typical MEMS IMU on an ESP32.
        self.declare_parameter('imu_accel_variance', 0.04)     # (m/s²)²
        self.declare_parameter('imu_gyro_variance', 0.001)     # (rad/s)²
        self.declare_parameter('imu_orientation_variance', 0.01)  # (rad)²; used only when has_orientation=True

        p = self.get_parameter
        self.port = p('serial_port').value
        self.baud = p('baud_rate').value
        self.wheel_base = p('wheel_base_m').value
        self.wheel_radius = p('wheel_radius_m').value
        self.ticks_per_rev = p('ticks_per_rev').value
        self.max_speed = p('max_speed_mps').value
        self.cmd_timeout = p('cmd_vel_timeout_s').value
        self.publish_tf = p('publish_tf').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.sim_mode = p('sim_mode').value
        self.imu_frame = p('imu_frame_id').value
        self.imu_has_orientation = p('imu_has_orientation').value
        self.accel_var = p('imu_accel_variance').value
        self.gyro_var = p('imu_gyro_variance').value
        self.orient_var = p('imu_orientation_variance').value

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.e_stop = False
        self.last_cmd_time = time.time()
        self.serial_ok = False
        self.fault_code = 0
        self.fault_msg = ''
        self._lock = threading.Lock()
        self._sim_last_time = time.time()
        self._sim_vx = 0.0
        self._sim_vyaw = 0.0
        self._batt_voltage = 0.0
        self._batt_current = 0.0
        self._imu_count = 0
        self._last_imu_time = 0.0
        self._ser = None          # shared serial handle; guarded by _lock

        # --- Publishers ---
        odom_qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', odom_qos)
        # IMU published at whatever rate the ESP32 sends (typically 50–200 Hz)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 50)
        self.motor_status_pub = self.create_publisher(MotorStatus, '/motor/status', 10)
        self.volt_pub = self.create_publisher(Float32, '/motor/voltage_raw', 10)
        self.curr_pub = self.create_publisher(Float32, '/motor/current_raw', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        if self.publish_tf:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # --- Subscribers ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/e_stop', self._estop_cb,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # --- Timers ---
        self.create_timer(0.05, self._watchdog_timer)
        self.create_timer(1.0, self._diag_timer)
        # Heartbeat keeps ESP32 alive; firmware stops motors after 3 s of silence.
        self.create_timer(1.0, self._heartbeat_timer)

        # --- Serial / sim ---
        if not self.sim_mode:
            self._serial_thread = threading.Thread(
                target=self._serial_loop, daemon=True)
            self._serial_thread.start()
        else:
            self.get_logger().warn('SIM MODE: no serial; integrating cmd_vel for odom')
            self.create_timer(0.05, self._sim_odom_timer)

        self.get_logger().info(
            f'motor_driver ready | port={self.port} | sim={self.sim_mode} | '
            f'imu_has_orientation={self.imu_has_orientation} | '
            f'imu_frame={self.imu_frame}')

    # ------------------------------------------------------------------ serial

    def _serial_loop(self):
        reconnect_delay = 2.0
        while rclpy.ok():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1)
                with self._lock:
                    self._ser = ser
                self.serial_ok = True
                self.get_logger().info(f'Serial connected: {self.port}')
                reconnect_delay = 2.0
            except serial.SerialException as e:
                self.serial_ok = False
                self.get_logger().warn(
                    f'Serial open failed ({e}); retry in {reconnect_delay:.0f}s')
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2, 30.0)
                continue

            buf = b''
            try:
                while rclpy.ok():
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            self._parse_esp32(line.strip())
            except serial.SerialException as e:
                self.serial_ok = False
                self.get_logger().warn(f'Serial read error: {e}; reconnecting')
            finally:
                with self._lock:
                    self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass

    def _parse_esp32(self, raw: bytes):
        try:
            msg = json.loads(raw.decode('utf-8'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return

        # Waveshare T-type protocol: integer T field selects message type.
        t = msg.get('T')
        if t == 1001:
            self._handle_odom(msg)
        elif t == 1002:
            self._handle_imu(msg)
        elif t == 1003:
            with self._lock:
                self._batt_voltage = float(msg.get('V', 0.0))
                self._batt_current = float(msg.get('C', 0.0))
            v = Float32(); v.data = self._batt_voltage
            c = Float32(); c.data = self._batt_current
            self.volt_pub.publish(v)
            self.curr_pub.publish(c)
        elif t == 9:
            with self._lock:
                self.fault_code = int(msg.get('code', -1))
                self.fault_msg = str(msg.get('msg', ''))
            self.get_logger().error(
                f'ESP32 fault {self.fault_code}: {self.fault_msg}')

    def _send_serial(self, payload: dict):
        """Write a single JSON line to the shared serial handle."""
        if not self.serial_ok or self.sim_mode:
            return
        with self._lock:
            ser = self._ser
        if ser is None:
            return
        try:
            ser.write((json.dumps(payload, separators=(',', ':')) + '\n').encode('utf-8'))
        except Exception:
            pass

    # ------------------------------------------------------------------ IMU

    def _handle_imu(self, msg: dict):
        """
        Parse ESP32 IMU packet and publish sensor_msgs/Imu.

        The robot IMU (e.g. MPU6050, ICM-42688) reports acceleration WITH
        gravity included. The EKF must have imu0_remove_gravitational_acceleration
        set to TRUE to compensate (opposite of the phone IMU setting).

        If the ESP32 runs onboard sensor fusion (BNO055 or similar) and sends
        quaternion fields, orientation is populated directly. For raw 6-axis IMUs
        orientation_covariance[0] is set to -1, signalling the EKF to ignore
        orientation measurements and rely on gyro integration for yaw.
        """
        now = self.get_clock().now()
        try:
            imu = Imu()
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = self.imu_frame

            # Linear acceleration — includes gravity (robot IMU default)
            imu.linear_acceleration.x = float(msg.get('ax', 0.0))
            imu.linear_acceleration.y = float(msg.get('ay', 0.0))
            imu.linear_acceleration.z = float(msg.get('az', 0.0))
            imu.linear_acceleration_covariance = [
                self.accel_var, 0.0, 0.0,
                0.0, self.accel_var, 0.0,
                0.0, 0.0, self.accel_var,
            ]

            # Angular velocity
            imu.angular_velocity.x = float(msg.get('gx', 0.0))
            imu.angular_velocity.y = float(msg.get('gy', 0.0))
            imu.angular_velocity.z = float(msg.get('gz', 0.0))
            imu.angular_velocity_covariance = [
                self.gyro_var, 0.0, 0.0,
                0.0, self.gyro_var, 0.0,
                0.0, 0.0, self.gyro_var,
            ]

            # Orientation — only if ESP32 sends quaternion (e.g. BNO055 fusion)
            if self.imu_has_orientation and 'qw' in msg:
                imu.orientation.w = float(msg['qw'])
                imu.orientation.x = float(msg['qx'])
                imu.orientation.y = float(msg['qy'])
                imu.orientation.z = float(msg['qz'])
                imu.orientation_covariance = [
                    self.orient_var, 0.0, 0.0,
                    0.0, self.orient_var, 0.0,
                    0.0, 0.0, self.orient_var * 5.0,  # yaw less reliable without mag
                ]
            else:
                # Signal EKF to ignore orientation; it will integrate gyro for yaw.
                # robot_localization treats orientation_covariance[0] == -1 as "unknown".
                imu.orientation_covariance[0] = -1.0

            self.imu_pub.publish(imu)
            with self._lock:
                self._imu_count += 1
                self._last_imu_time = time.time()

        except (KeyError, TypeError, ValueError) as e:
            self.get_logger().warn(f'IMU parse error: {e}', throttle_duration_sec=5.0)

    # ----------------------------------------------------------------- odometry

    def _handle_odom(self, msg: dict):
        now = self.get_clock().now()
        dt = float(msg.get('dt', 0.0))
        if dt <= 0.0 or dt > 1.0:
            return

        # T:1001 uses L/R for left/right wheel speeds in m/s
        left_vel = float(msg.get('L', 0.0))
        right_vel = float(msg.get('R', 0.0))
        vx = (left_vel + right_vel) / 2.0
        vyaw = (right_vel - left_vel) / self.wheel_base

        with self._lock:
            self.left_vel = left_vel
            self.right_vel = right_vel
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt
            self.theta += vyaw * dt

        self._publish_odom(now, vx, vyaw)

    def _publish_odom(self, stamp, vx, vyaw):
        with self._lock:
            x, y, theta = self.x, self.y, self.theta

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        ms = MotorStatus()
        ms.header.stamp = stamp.to_msg()
        with self._lock:
            ms.left_velocity_mps = self.left_vel
            ms.right_velocity_mps = self.right_vel
            ms.left_fault = self.fault_code != 0
            ms.right_fault = self.fault_code != 0
        self.motor_status_pub.publish(ms)

    # --------------------------------------------------------- sim mode

    def _sim_odom_timer(self):
        now = time.time()
        dt = now - self._sim_last_time
        self._sim_last_time = now
        with self._lock:
            vx = self._sim_vx
            vyaw = self._sim_vyaw
            self.left_vel = vx - (vyaw * self.wheel_base / 2.0)
            self.right_vel = vx + (vyaw * self.wheel_base / 2.0)
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt
            self.theta += vyaw * dt
        self._publish_odom(self.get_clock().now(), vx, vyaw)

    # --------------------------------------------------------------- callbacks

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            if self.e_stop:
                return
            self.last_cmd_time = time.time()
            vx = max(-self.max_speed, min(self.max_speed, float(msg.linear.x)))
            vyaw = float(msg.angular.z)
            if self.sim_mode:
                self._sim_vx = vx
                self._sim_vyaw = vyaw
                return
            left = vx - (vyaw * self.wheel_base / 2.0)
            right = vx + (vyaw * self.wheel_base / 2.0)
            m = max(abs(left), abs(right))
            if m > self.max_speed:
                left *= self.max_speed / m
                right *= self.max_speed / m
        # Waveshare T:1 drive command; normalise m/s to [-1, 1] range
        self._send_serial({'T': 1,
                           'L': round(left / self.max_speed, 4),
                           'R': round(right / self.max_speed, 4)})

    def _estop_cb(self, msg: Bool):
        with self._lock:
            self.e_stop = bool(msg.data)
        if self.e_stop:
            self._send_serial({'T': 1, 'L': 0, 'R': 0})
            self.get_logger().warn('E-STOP engaged')
        else:
            self.get_logger().info('E-STOP released')

    def _watchdog_timer(self):
        with self._lock:
            if self.e_stop:
                return
            if time.time() - self.last_cmd_time > self.cmd_timeout:
                if self.sim_mode:
                    self._sim_vx = 0.0
                    self._sim_vyaw = 0.0
                else:
                    self._send_serial({'T': 1, 'L': 0, 'R': 0})

    def _heartbeat_timer(self):
        if not self.sim_mode:
            self._send_serial({'T': 6})

    def _diag_timer(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        # Motor status
        ms = DiagnosticStatus()
        ms.name = 'motor_driver'
        ms.hardware_id = 'ugv_beast_esp32'
        with self._lock:
            fault = self.fault_code != 0
            ok = self.serial_ok or self.sim_mode
        ms.level = DiagnosticStatus.ERROR if (not ok or fault) else DiagnosticStatus.OK
        ms.message = (f'ESP32 fault {self.fault_code}: {self.fault_msg}' if fault
                      else ('Serial disconnected' if not ok else 'OK'))
        ms.values = [
            KeyValue(key='serial_ok', value=str(self.serial_ok)),
            KeyValue(key='sim_mode', value=str(self.sim_mode)),
            KeyValue(key='e_stop', value=str(self.e_stop)),
            KeyValue(key='fault_code', value=str(self.fault_code)),
        ]
        arr.status.append(ms)

        # IMU status
        ims = DiagnosticStatus()
        ims.name = 'robot_imu'
        ims.hardware_id = 'ugv_beast_imu'
        with self._lock:
            imu_age = time.time() - self._last_imu_time if self._last_imu_time > 0 else 9999.0
            imu_count = self._imu_count
        if imu_age > 2.0:
            ims.level = DiagnosticStatus.ERROR
            ims.message = f'IMU stale ({imu_age:.1f}s)' if self._last_imu_time > 0 else 'No IMU data received'
        else:
            ims.level = DiagnosticStatus.OK
            ims.message = f'OK ({imu_count} msgs)'
        ims.values = [
            KeyValue(key='msg_count', value=str(imu_count)),
            KeyValue(key='last_age_s', value=f'{imu_age:.2f}'),
            KeyValue(key='has_orientation', value=str(self.imu_has_orientation)),
            KeyValue(key='frame_id', value=self.imu_frame),
        ]
        arr.status.append(ims)

        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
