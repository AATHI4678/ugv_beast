"""
Static TF publishers for UGV Rover sensor mounting.

TF tree:
  base_link
    ├── laser       (RPLIDAR C1, front-mounted)
    ├── imu_link    (robot onboard IMU on ESP32 board, centre of chassis)
    └── gps         (phone GPS antenna, mounted on robot)

IMU MOUNTING NOTES:
  The UGV Rover ESP32 board sits near the centre of the chassis.
  The IMU chip (MPU6050 or similar) is soldered to the ESP32 carrier board.

  Typical UGV Rover orientation when board is flat:
    IMU +X  →  robot forward  (+X in REP-103)
    IMU +Y  →  robot left     (+Y in REP-103)
    IMU +Z  →  robot up       (+Z in REP-103)

  If the axes already align with REP-103 (x-forward, y-left, z-up), the
  rotation is identity (yaw=0, pitch=0, roll=0) and only the translation
  offset needs to be set.

  If the ESP32 board is mounted rotated (e.g. 90° CW from above so that
  the board's silk-screen +X points RIGHT instead of FORWARD), apply the
  appropriate yaw rotation below.

  VERIFY after mounting:
    1. Place robot on flat surface, perfectly still.
    2. ros2 topic echo /imu/data --once
    3. linear_acceleration.z ≈ +9.81 m/s² (gravity pointing down = +Z up)
    4. Rotate robot 90° clockwise (viewed from above) →
       /odometry/local yaw should DECREASE by ~1.57 rad.
    5. Push robot forward → linear_acceleration.x should be positive during accel.

  Adjust the imu_link arguments below if any of the above checks fail.

Arguments to static_transform_publisher:
  x y z yaw pitch roll parent_frame child_frame   (yaw/pitch/roll in radians)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # ── RPLIDAR C1 ──────────────────────────────────────────────────────
            # Front-centre of chassis, 0.20 m forward, 0.15 m above base_link.
            # Laser scan plane is horizontal; frame already REP-103 compliant.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_laser",
                arguments=[
                    "0.20",
                    "0.0",
                    "0.15",  # x y z  (metres)
                    "0",
                    "0",
                    "0",  # yaw pitch roll  (radians)
                    "base_link",
                    "laser",
                ],
            ),
            # ── Robot IMU (ESP32 onboard) ────────────────────────────────────────
            # The ESP32 carrier board sits roughly at the centre of the chassis,
            # approximately 0.05 m above the base plate.
            #
            # Default: axes already aligned (identity rotation).
            # If your board is rotated, change the yaw/pitch/roll below.
            # Common case — board rotated 90° CW from above: yaw = -1.5708
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_imu",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.05",  # x y z: centre, 5 cm above base
                    "0",
                    "0",
                    "0",  # yaw pitch roll: identity (adjust if needed)
                    "base_link",
                    "imu_link",
                ],
            ),
            # ── Phone GPS ────────────────────────────────────────────────────────
            # Phone used only for GPS now (no IMU from phone).
            # Mount the phone where it has clear sky view; update offsets to match.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_gps",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.50",  # x y z: top of robot ~50 cm above base
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "gps",
                ],
            ),
        ]
    )
