#!/usr/bin/env python3
"""
Motor driver node for WaveShare UGV Rover.

Pi (high-level) + ESP32 (low-level PID). Communication is UART JSON @ 115200.

PROTOCOL — corrected against a live serial capture from the real firmware.
The ESP32 does NOT send separate odom / IMU / battery frames. It sends ONE
combined feedback frame, type T:1001, e.g.:

  {"T":1001,"L":0,"R":0,"ax":3194,"ay":-196,"az":-7430,
   "gx":13,"gy":41,"gz":64,"mx":-191,"my":334,"mz":1111,
   "odl":-3,"odr":-6,"v":1201}

  L,R       left/right wheel speed (firmware units; ~m/s, 0 at rest)
  odl,odr   cumulative wheel-odometer ticks (signed)
  ax,ay,az  accelerometer RAW counts  (ICM-20948; ~7600 counts = 1 g)
  gx,gy,gz  gyro RAW counts           (ICM-20948)
  mx,my,mz  magnetometer RAW counts   (not used here)
  v         battery voltage in CENTIVOLTS (1201 -> 12.01 V)

  There is NO 'dt' field — dt is computed host-side from arrival time.
  There is NO T:1002 / T:1003 — everything is in the single T:1001 frame.

Pi -> ESP32 commands (unchanged, matches Waveshare base_ctrl wheel_speed):
  Drive:      {"T":1,"L":<float>,"R":<float>}   L/R in [-1,1] (frac of max)
  Stop:       {"T":1,"L":0,"R":0}
  Heartbeat:  {"T":6}

IMU NOTE — scaling and signs:
  accel_scale / gyro_scale convert raw counts to SI. Defaults assume an
  ICM-20948 at +-2 g / +-250 dps. They are ESTIMATES — calibrate:
    * robot level + still  -> linear_acceleration z magnitude ~= 9.81
    * rotate robot 90 deg  -> integral of angular_velocity z ~= 1.57 rad
  If an axis reads with the wrong sign, flip it via the imu_*_invert_*
  parameters in motor_params.yaml — no code change needed.

Publishes:
  /wheel/odom        nav_msgs/Odometry
  /imu/data          sensor_msgs/Imu
  /motor/status      ugv_interfaces/MotorStatus
  /motor/voltage_raw std_msgs/Float32
  /diagnostics       diagnostic_msgs/DiagnosticArray
Subscribes:
  /cmd_vel           geometry_msgs/Twist
  /e_stop            std_msgs/Bool
"""

import json
import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
from ugv_interfaces.msg import MotorStatus

try:
    import serial

    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# Firmware feedback frame type. The real ESP32 firmware emits one combined
# frame; everything (wheels + IMU + battery) arrives under this single code.
FEEDBACK_T = 1001


class MotorDriver(Node):
    """Differential-drive motor driver + IMU publisher for the WaveShare UGV Rover."""

    def __init__(self):
        super().__init__("motor_driver")

        # ---------------------------------------------------------- parameters
        self.declare_parameter("serial_port", "/dev/ttyAMA0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_base_m", 0.295)
        self.declare_parameter("wheel_radius_m", 0.0525)
        self.declare_parameter("ticks_per_rev", 1560)
        self.declare_parameter("max_speed_mps", 0.35)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sim_mode", False)
        self.declare_parameter("invert_drive", False)

        # IMU framing
        self.declare_parameter("imu_frame_id", "imu_link")
        # The real UGV Rover IMU (ICM-20948) sends raw accel+gyro only, no
        # quaternion. Leave False unless your firmware enables the DMP.
        self.declare_parameter("imu_has_orientation", False)

        # IMU raw-count -> SI scale factors. ESTIMATES — see module docstring.
        #   accel: capture showed az ~= -7600 at 1 g  -> 7600 counts/g
        #   gyro : ICM-20948 default FS +-250 dps      -> 131 counts/(deg/s)
        self.declare_parameter("imu_accel_scale", 9.81 / 7600.0)
        self.declare_parameter("imu_gyro_scale", (math.pi / 180.0) / 131.0)

        # IMU axis sign flips. Defaults: no inversion. If, with the robot
        # level and still, linear_acceleration.z comes out NEGATIVE, set
        # imu_accel_invert_z:=true (gravity reaction should read +9.81).
        # If a hand-rotation gives yaw of the wrong sign, flip gyro z.
        self.declare_parameter("imu_accel_invert_x", False)
        self.declare_parameter("imu_accel_invert_y", False)
        self.declare_parameter("imu_accel_invert_z", False)
        self.declare_parameter("imu_gyro_invert_x", False)
        self.declare_parameter("imu_gyro_invert_y", False)
        self.declare_parameter("imu_gyro_invert_z", False)
        self.declare_parameter("imu_gyro_bias_x", 0.0)
        self.declare_parameter("imu_gyro_bias_y", 0.0)
        self.declare_parameter("imu_gyro_bias_z", 0.0)

        # IMU covariances — tune after measuring your IMU's noise floor.
        self.declare_parameter("imu_accel_variance", 0.04)  # (m/s^2)^2
        self.declare_parameter("imu_gyro_variance", 0.001)  # (rad/s)^2
        self.declare_parameter("imu_orientation_variance", 0.01)  # (rad)^2

        p = self.get_parameter
        self.port = p("serial_port").value
        self.baud = p("baud_rate").value
        self.wheel_base = p("wheel_base_m").value
        self.wheel_radius = p("wheel_radius_m").value
        self.ticks_per_rev = p("ticks_per_rev").value
        self.max_speed = p("max_speed_mps").value
        self.cmd_timeout = p("cmd_vel_timeout_s").value
        self.publish_tf = p("publish_tf").value
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.sim_mode = p("sim_mode").value
        self.imu_frame = p("imu_frame_id").value
        self.imu_has_orientation = p("imu_has_orientation").value
        self.accel_scale = p("imu_accel_scale").value
        self.gyro_scale = p("imu_gyro_scale").value
        self.accel_var = p("imu_accel_variance").value
        self.gyro_var = p("imu_gyro_variance").value
        self.orient_var = p("imu_orientation_variance").value

        self._gx_bias = p("imu_gyro_bias_x").value
        self._gy_bias = p("imu_gyro_bias_y").value
        self._gz_bias = p("imu_gyro_bias_z").value

        self._drive_sign = -1.0 if p("invert_drive").value else 1.0

        # Pre-fold sign flips into signed scale factors so the hot path is cheap.
        self._ax_s = self.accel_scale * (-1.0 if p("imu_accel_invert_x").value else 1.0)
        self._ay_s = self.accel_scale * (-1.0 if p("imu_accel_invert_y").value else 1.0)
        self._az_s = self.accel_scale * (-1.0 if p("imu_accel_invert_z").value else 1.0)
        self._gx_s = self.gyro_scale * (-1.0 if p("imu_gyro_invert_x").value else 1.0)
        self._gy_s = self.gyro_scale * (-1.0 if p("imu_gyro_invert_y").value else 1.0)
        self._gz_s = self.gyro_scale * (-1.0 if p("imu_gyro_invert_z").value else 1.0)

        # --------------------------------------------------------------- state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.e_stop = False
        self.last_cmd_time = time.time()
        self.serial_ok = False
        self.fault_code = 0
        self.fault_msg = ""
        self._lock = threading.Lock()
        self._sim_last_time = time.time()
        self._sim_vx = 0.0
        self._sim_vyaw = 0.0
        self._batt_voltage = 0.0
        self._imu_count = 0
        self._last_imu_time = 0.0
        self._last_odom_t = 0.0  # host-side dt source (no 'dt' field on the wire)
        self._ser = None  # shared serial handle; guarded by _lock

        # ---------------------------------------------------------- publishers
        odom_qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", odom_qos)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 50)
        self.motor_status_pub = self.create_publisher(MotorStatus, "/motor/status", 10)
        self.volt_pub = self.create_publisher(Float32, "/motor/voltage_raw", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        if self.publish_tf:
            from tf2_ros import TransformBroadcaster

            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # --------------------------------------------------------- subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, 10
        )
        self.estop_sub = self.create_subscription(
            Bool,
            "/e_stop",
            self._estop_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # --------------------------------------------------------------- timers
        self.create_timer(0.05, self._watchdog_timer)
        self.create_timer(1.0, self._diag_timer)
        # Heartbeat keeps the ESP32 alive; firmware stops motors after ~3 s
        # of silence. If the robot drives then stops dead after ~3 s, the
        # heartbeat type is wrong for your firmware — see _heartbeat_timer.
        self.create_timer(1.0, self._heartbeat_timer)

        # ----------------------------------------------------------- serial/sim
        if not self.sim_mode:
            if not SERIAL_AVAILABLE:
                self.get_logger().error(
                    "pyserial not installed; cannot open ESP32 link. "
                    "Install with: pip install pyserial"
                )
            self._serial_thread = threading.Thread(
                target=self._serial_loop, daemon=True
            )
            self._serial_thread.start()
        else:
            self.get_logger().warn("SIM MODE: no serial; integrating cmd_vel for odom")
            self.create_timer(0.05, self._sim_odom_timer)

        self.get_logger().info(
            f"motor_driver ready | port={self.port} @ {self.baud} | "
            f"sim={self.sim_mode} | imu_has_orientation={self.imu_has_orientation} | "
            f"imu_frame={self.imu_frame}"
        )

    # ====================================================================
    # Serial RX
    # ====================================================================

    def _serial_loop(self):
        reconnect_delay = 2.0
        while rclpy.ok():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1)
                with self._lock:
                    self._ser = ser
                self.serial_ok = True
                self.get_logger().info(f"Serial connected: {self.port}")
                reconnect_delay = 2.0
            except serial.SerialException as e:
                self.serial_ok = False
                self.get_logger().warn(
                    f"Serial open failed ({e}); retry in {reconnect_delay:.0f}s"
                )
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 2, 30.0)
                continue

            buf = b""
            try:
                while rclpy.ok():
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                        # Cap buffer so a missing newline can't grow it forever.
                        if len(buf) > 8192:
                            buf = buf[-1024:]
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            self._parse_esp32(line.strip())
            except serial.SerialException as e:
                self.serial_ok = False
                self.get_logger().warn(f"Serial read error: {e}; reconnecting")
            finally:
                with self._lock:
                    self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass

    def _parse_esp32(self, raw: bytes):
        if not raw:
            return
        try:
            msg = json.loads(raw.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        if not isinstance(msg, dict):
            return
        # The real firmware sends ONE combined frame type.
        if msg.get("T") == FEEDBACK_T:
            self._handle_feedback(msg)

    def _send_serial(self, payload: dict):
        """Write a single compact JSON line to the shared serial handle."""
        if not self.serial_ok or self.sim_mode:
            return
        with self._lock:
            ser = self._ser
        if ser is None:
            return
        try:
            ser.write(
                (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
            )
        except Exception:
            pass

    # ====================================================================
    # Feedback handling — single combined T:1001 frame
    # ====================================================================

    def _handle_feedback(self, msg: dict):
        """Decode one T:1001 frame: wheel speeds + raw IMU + battery."""
        now = self.get_clock().now()

        # ---- battery: 'v' in centivolts (1201 -> 12.01 V) ----
        if "v" in msg:
            try:
                volts = float(msg["v"]) * 0.01
                with self._lock:
                    self._batt_voltage = volts
                self.volt_pub.publish(Float32(data=volts))
            except (TypeError, ValueError):
                pass

        # ---- odometry: host-computed dt + differential-drive integration ----
        if "L" in msg and "R" in msg:
            try:
                left_vel = float(msg["L"])
                right_vel = float(msg["R"])
            except (TypeError, ValueError):
                left_vel = right_vel = 0.0

            t_now = time.time()
            dt = t_now - self._last_odom_t if self._last_odom_t else 0.0
            self._last_odom_t = t_now

            vx = (left_vel + right_vel) / 2.0
            vyaw = (right_vel - left_vel) / self.wheel_base

            with self._lock:
                self.left_vel = left_vel
                self.right_vel = right_vel
                # Ignore the first sample (dt unknown) and stale gaps.
                if 0.0 < dt < 0.5:
                    self.theta += vyaw * dt
                    self.x += vx * math.cos(self.theta) * dt
                    self.y += vx * math.sin(self.theta) * dt

            self._publish_odom(now, vx, vyaw)

        # ---- IMU: raw counts -> SI, with optional per-axis sign flip ----
        if all(k in msg for k in ("ax", "ay", "az", "gx", "gy", "gz")):
            try:
                self._publish_imu(now, msg)
            except (TypeError, ValueError) as e:
                self.get_logger().warn(
                    f"IMU parse error: {e}", throttle_duration_sec=5.0
                )

    def _publish_imu(self, stamp, msg: dict):
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.imu_frame

        # Raw counts -> SI. Sign flips are pre-folded into the *_s factors.
        imu.linear_acceleration.x = float(msg["ax"]) * self._ax_s
        imu.linear_acceleration.y = float(msg["ay"]) * self._ay_s
        imu.linear_acceleration.z = float(msg["az"]) * self._az_s
        imu.linear_acceleration_covariance = [
            self.accel_var,
            0.0,
            0.0,
            0.0,
            self.accel_var,
            0.0,
            0.0,
            0.0,
            self.accel_var,
        ]

        imu.angular_velocity.x = float(msg["gx"]) * self._gx_s
        imu.angular_velocity.y = float(msg["gy"]) * self._gy_s
        imu.angular_velocity.z = float(msg["gz"]) * self._gz_s
        imu.angular_velocity_covariance = [
            self.gyro_var,
            0.0,
            0.0,
            0.0,
            self.gyro_var,
            0.0,
            0.0,
            0.0,
            self.gyro_var,
        ]
        # BIAS FIX
        imu.angular_velocity.x = float(msg["gx"]) * self._gx_s - self._gx_bias
        imu.angular_velocity.y = float(msg["gy"]) * self._gy_s - self._gy_bias
        imu.angular_velocity.z = float(msg["gz"]) * self._gz_s - self._gz_bias

        # Orientation: only if firmware DMP sends a quaternion. The stock
        # UGV Rover firmware does not, so orientation_covariance[0] = -1
        # tells robot_localization to ignore orientation and integrate gyro.
        if self.imu_has_orientation and "qw" in msg:
            imu.orientation.w = float(msg["qw"])
            imu.orientation.x = float(msg["qx"])
            imu.orientation.y = float(msg["qy"])
            imu.orientation.z = float(msg["qz"])
            imu.orientation_covariance = [
                self.orient_var,
                0.0,
                0.0,
                0.0,
                self.orient_var,
                0.0,
                0.0,
                0.0,
                self.orient_var * 5.0,
            ]
        else:
            imu.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu)
        with self._lock:
            self._imu_count += 1
            self._last_imu_time = time.time()

    # ====================================================================
    # Odometry publishing
    # ====================================================================

    def _publish_odom(self, stamp, vx, vyaw):
        with self._lock:
            x, y, theta = self.x, self.y, self.theta
            left_vel, right_vel = self.left_vel, self.right_vel
            fault = self.fault_code != 0

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
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

        # TF: only publish odom->base_link if explicitly enabled. In the dual-
        # EKF stack the LOCAL EKF owns this transform — keep publish_tf:=false
        # there to avoid two publishers fighting over odom->base_link.
        if self.tf_broadcaster is not None:
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
        ms.left_velocity_mps = left_vel
        ms.right_velocity_mps = right_vel
        ms.left_fault = fault
        ms.right_fault = fault
        self.motor_status_pub.publish(ms)

    # ====================================================================
    # Sim mode
    # ====================================================================

    def _sim_odom_timer(self):
        now = time.time()
        dt = now - self._sim_last_time
        self._sim_last_time = now
        with self._lock:
            vx = self._sim_vx
            vyaw = self._sim_vyaw
            self.left_vel = vx - (vyaw * self.wheel_base / 2.0)
            self.right_vel = vx + (vyaw * self.wheel_base / 2.0)
            if 0.0 < dt < 0.5:
                self.theta += vyaw * dt
                self.x += vx * math.cos(self.theta) * dt
                self.y += vx * math.sin(self.theta) * dt
        self._publish_odom(self.get_clock().now(), vx, vyaw)

    # ====================================================================
    # Callbacks
    # ====================================================================

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
        # Waveshare T:1 drive command; normalise m/s to [-1, 1].
        self._send_serial(
            {
                "T": 1,
                "L": round(self._drive_sign * left / self.max_speed, 4),
                "R": round(self._drive_sign * right / self.max_speed, 4),
            }
        )

    def _estop_cb(self, msg: Bool):
        with self._lock:
            self.e_stop = bool(msg.data)
        if self.e_stop:
            self._send_serial({"T": 1, "L": 0, "R": 0})
            self.get_logger().warn("E-STOP engaged")
        else:
            self.get_logger().info("E-STOP released")

    def _watchdog_timer(self):
        with self._lock:
            if self.e_stop:
                return
            stale = time.time() - self.last_cmd_time > self.cmd_timeout
        if stale:
            if self.sim_mode:
                with self._lock:
                    self._sim_vx = 0.0
                    self._sim_vyaw = 0.0
            else:
                self._send_serial({"T": 1, "L": 0, "R": 0})

    def _heartbeat_timer(self):
        # Keep-alive. If the robot drives on a /cmd_vel then stops dead after
        # ~3 s, this frame type is not what the firmware expects — try
        # resending the last drive command instead of {"T":6}.
        if not self.sim_mode:
            self._send_serial({"T": 6})

    def _diag_timer(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        ms = DiagnosticStatus()
        ms.name = "motor_driver"
        ms.hardware_id = "ugv_Rover_esp32"
        with self._lock:
            fault = self.fault_code != 0
            ok = self.serial_ok or self.sim_mode
            volts = self._batt_voltage
        ms.level = DiagnosticStatus.ERROR if (not ok or fault) else DiagnosticStatus.OK
        ms.message = (
            f"ESP32 fault {self.fault_code}: {self.fault_msg}"
            if fault
            else ("Serial disconnected" if not ok else "OK")
        )
        ms.values = [
            KeyValue(key="serial_ok", value=str(self.serial_ok)),
            KeyValue(key="sim_mode", value=str(self.sim_mode)),
            KeyValue(key="e_stop", value=str(self.e_stop)),
            KeyValue(key="fault_code", value=str(self.fault_code)),
            KeyValue(key="battery_v", value=f"{volts:.2f}"),
        ]
        arr.status.append(ms)

        ims = DiagnosticStatus()
        ims.name = "robot_imu"
        ims.hardware_id = "ugv_Rover_imu"
        with self._lock:
            imu_age = (
                time.time() - self._last_imu_time if self._last_imu_time > 0 else 9999.0
            )
            imu_count = self._imu_count
        if imu_age > 2.0:
            ims.level = DiagnosticStatus.ERROR
            ims.message = (
                f"IMU stale ({imu_age:.1f}s)"
                if self._last_imu_time > 0
                else "No IMU data received"
            )
        else:
            ims.level = DiagnosticStatus.OK
            ims.message = f"OK ({imu_count} msgs)"
        ims.values = [
            KeyValue(key="msg_count", value=str(imu_count)),
            KeyValue(key="last_age_s", value=f"{imu_age:.2f}"),
            KeyValue(key="has_orientation", value=str(self.imu_has_orientation)),
            KeyValue(key="frame_id", value=self.imu_frame),
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
