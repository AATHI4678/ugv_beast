"""
vision_server_node.py — ugv_vision package
ROS2 Jazzy node that:
  1. Captures the USB camera via ffmpeg subprocess
  2. Publishes sensor_msgs/CompressedImage on /camera/image/compressed
  3. Runs a Flask HTTP server (non-blocking, in a background thread):
       GET  /video_feed   → MJPEG stream (for rover_ai.py on the laptop)
       POST /control      → {"move": "forward|backward|left|right|stop"}
       GET  /status       → JSON node + motor state
       GET  /health       → liveness probe
  4. Translates HTTP move commands into geometry_msgs/Twist on /cmd_vel/teleop
     so the existing teleop_watchdog → motor_driver → ESP32 stack handles
     all hardware I/O — no direct serial writes here.
  5. Asserts /teleop_override (std_msgs/Bool True) while vision is commanding,
     releasing it (False) 1.5 s after the last non-stop command.
  6. Publishes /vision/status (std_msgs/String JSON) for diagnostics.

ROS2 Parameters (set via vision_params.yaml or CLI):
  camera_device   string   /dev/video0
  flask_port      int      5000
  image_width     int      640
  image_height    int      480
  image_fps       int      15
  jpeg_quality    int      70
  drive_speed     float    0.40   m/s  (linear.x for fwd/back)
  turn_speed      float    0.80   rad/s (angular.z for left/right)
  override_timeout float   1.5    s after last non-stop cmd before releasing override
"""

import json
import subprocess
import threading
import time
import logging

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ugv_interfaces.msg import BatteryState
from ugv_interfaces.srv import EmergencyStop
from flask import Flask, Response, request, jsonify

DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    html, body { width: 100%; height: 100%; background: #000; overflow: hidden; font-family: Arial, sans-serif; }
    #video { position: absolute; top: 0; left: 0; width: 100%; height: 100%; object-fit: cover; }

    #top-bar {
      position: fixed; top: 0; left: 0; right: 0;
      display: flex; align-items: center; gap: 16px;
      padding: 10px 16px; background: rgba(0,0,0,0.55);
      z-index: 10; font-size: 12px; color: #ccc;
    }
    #camera-status { color: #2a2; }
    #camera-status.error { color: #f44; }
    #battery-status { margin-left: auto; }

    #bottom-bar {
      position: fixed; bottom: 0; left: 0; right: 0;
      display: flex; align-items: center; justify-content: center;
      padding: 12px; background: rgba(0,0,0,0.5);
      z-index: 10;
    }

    #estop-btn {
      width: 90px; height: 90px; border-radius: 50%;
      border: 5px solid #333; font-size: 13px; font-weight: bold;
      cursor: pointer; color: #fff;
      background: #222e1a; border-color: #3a5a2a;
      transition: all 0.15s;
      box-shadow: 0 4px 12px rgba(0,0,0,0.6);
      user-select: none;
    }
    #estop-btn.engaged {
      background: #7a1a1a; border-color: #cc2222;
      animation: pulse 0.5s ease-in-out infinite alternate;
    }
    #estop-btn:active { transform: scale(0.95); }
    @keyframes pulse {
      from { box-shadow: 0 0 8px #c22; }
      to   { box-shadow: 0 0 28px #f55; }
    }

    #estop-status {
      position: fixed; bottom: 110px; left: 50%; transform: translateX(-50%);
      font-size: 11px; font-weight: bold; letter-spacing: 0.08em;
      color: #ccc; background: rgba(0,0,0,0.55); padding: 4px 12px;
      border-radius: 4px; z-index: 10;
    }
    #estop-status.engaged { color: #f66; }

    #service-status {
      position: fixed; bottom: 110px; right: 20px;
      font-size: 10px; color: #888; background: rgba(0,0,0,0.55);
      padding: 4px 8px; border-radius: 4px; z-index: 10;
    }
  </style>
</head>
<body>
  <img id="video" src="" />

  <div id="top-bar">
    <span id="camera-status">CAM: --</span>
    <span id="motor-state">MOTOR: STOP</span>
    <span id="battery-status">BAT: --</span>
  </div>

  <div id="estop-status">E-STOP: RELEASED</div>
  <div id="service-status">via: --</div>

  <div id="bottom-bar">
    <button id="estop-btn" onclick="toggleEstop()">DRIVE</button>
  </div>

  <script>
    let estopEngaged = false;
    let serviceVia = null;
    const btn = document.getElementById('estop-btn');
    const statusEl = document.getElementById('estop-status');
    const svcEl = document.getElementById('service-status');
    const camEl = document.getElementById('camera-status');
    const motorEl = document.getElementById('motor-state');
    const battEl = document.getElementById('battery-status');

    document.getElementById('video').src = '/video_feed?' + Date.now();

    function toggleEstop() {
      const action = estopEngaged ? 'release' : 'engage';
      btn.disabled = true;
      fetch('/estop', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action: action, reason: 'dashboard'})
      }).then(r => r.json()).then(data => {
        if (data.success !== undefined) {
          estopEngaged = (action === 'engage');
          serviceVia = data.via_service ? 'service' : 'topic';
          updateUI();
        }
        btn.disabled = false;
      }).catch(() => { btn.disabled = false; });
    }

    function updateUI() {
      if (estopEngaged) {
        btn.textContent = 'E-STOP';
        btn.classList.add('engaged');
        statusEl.textContent = 'E-STOP: ENGAGED';
        statusEl.classList.add('engaged');
      } else {
        btn.textContent = 'DRIVE';
        btn.classList.remove('engaged');
        statusEl.textContent = 'E-STOP: RELEASED';
        statusEl.classList.remove('engaged');
      }
      svcEl.textContent = 'via: ' + (serviceVia || '--');
    }

    function pollStatus() {
      fetch('/status').then(r => r.json()).then(data => {
        camEl.textContent = 'CAM: ' + (data.camera_ok ? 'OK' : 'ERR');
        camEl.className = data.camera_ok ? '' : 'error';
        motorEl.textContent = 'MOTOR: ' + (data.motor_state || 'UNK').toUpperCase();
      }).catch(() => {});
    }

    function pollBattery() {
      fetch('/battery').then(r => r.json()).then(data => {
        const pct = data.percent.toFixed(0);
        let text = 'BAT: ' + pct + '%';
        if (data.critical) {
          text += ' CRIT';
          battEl.style.color = '#f44';
        } else if (data.low) {
          text += ' LOW';
          battEl.style.color = '#fa4';
        } else {
          battEl.style.color = '#ccc';
        }
        battEl.textContent = text;
      }).catch(() => {});
    }

    setInterval(pollStatus, 2000);
    setInterval(pollBattery, 2000);
    pollBattery();
  </script>
</body>
</html>"""

# Suppress Flask's default startup banner in the ROS2 log
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)


class VisionServerNode(Node):

    def __init__(self):
        super().__init__("vision_server")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("camera_device",    "/dev/video0")
        self.declare_parameter("flask_port",       5000)
        self.declare_parameter("image_width",      640)
        self.declare_parameter("image_height",     480)
        self.declare_parameter("image_fps",        15)
        self.declare_parameter("jpeg_quality",     70)
        self.declare_parameter("drive_speed",      0.40)
        self.declare_parameter("turn_speed",       0.80)
        self.declare_parameter("override_timeout", 1.5)

        p = self.get_parameter
        self._cam_dev    = p("camera_device").value
        self._port       = p("flask_port").value
        self._width      = p("image_width").value
        self._height     = p("image_height").value
        self._fps        = p("image_fps").value
        self._jpeg_q     = p("jpeg_quality").value
        self._drive_spd  = p("drive_speed").value
        self._turn_spd   = p("turn_speed").value
        self._ovr_tmout  = p("override_timeout").value

        # ── Publishers ──────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Twist, "/cmd_vel/teleop", 10)

        self._img_pub = self.create_publisher(
            CompressedImage, "/camera/image/compressed", 10)

        latching_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._override_pub = self.create_publisher(
            Bool, "/teleop_override", latching_qos)

        self._estop_pub = self.create_publisher(
            Bool, "/e_stop", latching_qos)

        self._status_pub = self.create_publisher(
            String, "/vision/status", 10)

        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)

        self._estop_svc = self.create_client(EmergencyStop, "/emergency_stop")

        self._batt_sub = self.create_subscription(
            BatteryState, "/battery_state", self._batt_cb, 10)

        # ── State ───────────────────────────────────────────────────────────
        self._frame_lock   = threading.Lock()
        self._latest_frame: bytes | None = None   # JPEG bytes for MJPEG stream
        self._motor_state  = "stop"
        self._last_cmd_t   = 0.0
        self._override_on  = False
        self._cam_ok       = False
        self._frame_count  = 0

        self._estop_state = False
        self._estop_svc_available = False
        self._estop_lock = threading.Lock()
        self._last_estop_response = {"success": False, "message": ""}

        self._battery = {"percent": 0.0, "voltage": 0.0, "low": False, "critical": False}

        # Twist map: move string → (linear_x, angular_z)
        self._twist_map = {
            "forward":  ( self._drive_spd,  0.0),
            "backward": (-self._drive_spd,  0.0),
            "left":     ( 0.0,              self._turn_spd),
            "right":    ( 0.0,             -self._turn_spd),
            "stop":     ( 0.0,              0.0),
        }

        # ── Background threads ──────────────────────────────────────────────
        threading.Thread(target=self._camera_thread,
                         daemon=True, name="cam").start()
        threading.Thread(target=self._flask_thread,
                         daemon=True, name="flask").start()

        # ── ROS2 timers ─────────────────────────────────────────────────────
        self.create_timer(0.067, self._publish_image)      # ~15 Hz image pub
        self.create_timer(0.5,   self._override_watchdog)  # release override
        self.create_timer(1.0,   self._publish_status)
        self.create_timer(2.0,   self._publish_diagnostics)
        self.create_timer(5.0,   self._estop_service_watcher)  # watch for service availability

        self.get_logger().info(
            f"vision_server ready | cam={self._cam_dev} "
            f"{self._width}x{self._height}@{self._fps}fps | "
            f"flask=:{self._port}"
        )

    # ════════════════════════════════════════════════════════════════════════
    # Camera capture (ffmpeg → raw BGR → JPEG)
    # ════════════════════════════════════════════════════════════════════════

    def _camera_thread(self):
        cmd = [
            "ffmpeg", "-loglevel", "error",
            "-f", "v4l2",
            "-input_format", "mjpeg",
            "-video_size", f"{self._width}x{self._height}",
            "-framerate", str(self._fps),
            "-i", self._cam_dev,
            "-f", "rawvideo", "-pix_fmt", "bgr24", "pipe:1",
        ]
        enc = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q]
        frame_bytes = self._width * self._height * 3

        while True:
            try:
                proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.get_logger().info(
                    f"ffmpeg started (pid={proc.pid})")
                self._cam_ok = True

                while True:
                    raw = proc.stdout.read(frame_bytes)
                    if len(raw) != frame_bytes:
                        self.get_logger().warning(
                            "ffmpeg pipe closed, restarting...")
                        self._cam_ok = False
                        break
                    frame = np.frombuffer(
                        raw, dtype=np.uint8).reshape(
                            (self._height, self._width, 3))
                    _, buf = cv2.imencode(".jpg", frame, enc)
                    with self._frame_lock:
                        self._latest_frame = buf.tobytes()
                    self._frame_count += 1

            except Exception as exc:
                self.get_logger().error(f"Camera error: {exc}")
                self._cam_ok = False
            finally:
                try:
                    proc.kill()
                except Exception:
                    pass
            time.sleep(1.0)

    # ════════════════════════════════════════════════════════════════════════
    # ROS2 image publisher
    # ════════════════════════════════════════════════════════════════════════

    def _publish_image(self):
        with self._frame_lock:
            jpg = self._latest_frame
        if jpg is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = list(jpg)
        self._img_pub.publish(msg)

    # ════════════════════════════════════════════════════════════════════════
    # Motor command → ROS2 Twist
    # ════════════════════════════════════════════════════════════════════════

    def _send_twist(self, action: str):
        if action not in self._twist_map:
            return
        lx, az = self._twist_map[action]
        twist = Twist()
        twist.linear.x  = float(lx)
        twist.angular.z = float(az)
        self._cmd_pub.publish(twist)
        self._motor_state = action

        # Activate teleop override for all non-stop commands
        if action != "stop":
            self._last_cmd_t = time.monotonic()
            if not self._override_on:
                self._override_pub.publish(Bool(data=True))
                self._override_on = True
                self.get_logger().info("Teleop override: ON")

    def _override_watchdog(self):
        """Release override after override_timeout seconds of inactivity."""
        if not self._override_on:
            return
        if time.monotonic() - self._last_cmd_t > self._ovr_tmout:
            self._send_twist("stop")
            self._override_pub.publish(Bool(data=False))
            self._override_on = False
            self.get_logger().info("Teleop override: OFF (timeout)")

    def _estop_service_watcher(self):
        """Periodically check if /emergency_stop service is up."""
        was_available = self._estop_svc_available
        self._estop_svc_available = self._estop_svc.service_is_ready()
        if self._estop_svc_available and not was_available:
            self.get_logger().info("Emergency stop service now available")
        elif not self._estop_svc_available and was_available:
            self.get_logger().warn("Emergency stop service lost")

    def _estop_svc_done_callback(self, future):
        """Handle async service call response."""
        try:
            result = future.result()
            with self._estop_lock:
                self._last_estop_response = {
                    "success": bool(result.success),
                    "message": str(result.message),
                }
        except Exception as e:
            with self._estop_lock:
                self._last_estop_response = {
                    "success": False,
                    "message": f"service call failed: {e}",
                }

    def _batt_cb(self, msg: BatteryState):
        """Cache latest battery state for the /battery Flask route."""
        with self._frame_lock:
            self._battery = {
                "percent":  float(msg.percent),
                "voltage":  float(msg.voltage),
                "low":      bool(msg.low_battery_warning),
                "critical": bool(msg.critical_battery),
            }

    # ════════════════════════════════════════════════════════════════════════
    # Flask HTTP server
    # ════════════════════════════════════════════════════════════════════════

    def _flask_thread(self):
        app = Flask("vision_server")

        def mjpeg_generator():
            while True:
                with self._frame_lock:
                    frame = self._latest_frame
                if frame:
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + frame + b"\r\n"
                    )
                else:
                    time.sleep(0.05)

        @app.route("/video_feed")
        def video_feed():
            return Response(
                mjpeg_generator(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/control", methods=["POST"])
        def control():
            data = request.get_json(silent=True)
            if not data or "move" not in data:
                return jsonify({"error": "missing 'move' key"}), 400
            action = str(data["move"]).lower().strip()
            if action not in self._twist_map:
                return jsonify({"error": f"unknown command: {action}"}), 400
            self._send_twist(action)
            return jsonify({
                "status":      "ok",
                "action":      action,
                "motor_state": self._motor_state,
                "override":    self._override_on,
            })

        @app.route("/status")
        def status():
            return jsonify({
                "node":        "vision_server",
                "camera_ok":   self._cam_ok,
                "motor_state": self._motor_state,
                "override":    self._override_on,
                "frames":      self._frame_count,
            })

        @app.route("/health")
        def health():
            return jsonify({"ok": True})

        @app.route("/dashboard")
        def dashboard():
            return Response(
                DASHBOARD_HTML,
                mimetype="text/html",
            )

        @app.route("/battery")
        def battery():
            with self._frame_lock:
                b = self._battery
            return jsonify({
                "percent":  b["percent"],
                "voltage":  round(b["voltage"], 2),
                "low":      b["low"],
                "critical": b["critical"],
            })

        @app.route("/estop", methods=["POST"])
        def estop():
            data = request.get_json(silent=True) or {}
            action = data.get("action", "").lower()
            reason = str(data.get("reason", "operator"))

            if action not in ("engage", "release"):
                return jsonify({
                    "success": False,
                    "message": f"Unknown action: '{action}'. Use 'engage' or 'release'.",
                }), 400

            engage = (action == "engage")

            with self._estop_lock:
                self._estop_state = engage

                if self._estop_svc_available:
                    req = EmergencyStop.Request()
                    req.stop = engage
                    req.reason = reason
                    self._estop_svc.call_async(req).add_done_callback(
                        self._estop_svc_done_callback)
                    via_service = True
                else:
                    self._estop_pub.publish(Bool(data=engage))
                    via_service = False

                response_success = True
                response_message = (
                    f"E-stop {'engaged' if engage else 'released'} via topic"
                    if not via_service
                    else f"E-stop {'engaged' if engage else 'released'} via service"
                )

            return jsonify({
                "success":   response_success,
                "message":   response_message,
                "estop":     self._estop_state,
                "via_service": via_service,
            })

        app.run(host="0.0.0.0", port=self._port,
                threaded=True, debug=False)

    # ════════════════════════════════════════════════════════════════════════
    # Diagnostics / status
    # ════════════════════════════════════════════════════════════════════════

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "camera_ok":   self._cam_ok,
            "motor_state": self._motor_state,
            "override":    self._override_on,
            "frames":      self._frame_count,
        })
        self._status_pub.publish(msg)

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        s = DiagnosticStatus()
        s.name = "vision_server"
        s.hardware_id = "usb_camera"
        s.level = (DiagnosticStatus.OK if self._cam_ok
                   else DiagnosticStatus.ERROR)
        s.message = "Camera OK" if self._cam_ok else "Camera not producing frames"
        s.values = [
            KeyValue(key="camera_device",  value=self._cam_dev),
            KeyValue(key="motor_state",    value=self._motor_state),
            KeyValue(key="override_active",value=str(self._override_on)),
            KeyValue(key="frame_count",    value=str(self._frame_count)),
            KeyValue(key="flask_port",     value=str(self._port)),
        ]
        arr.status.append(s)
        self._diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VisionServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop on shutdown so rover doesn't drive off
        node._send_twist("stop")
        node._override_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
