# UGV Rover Delivery Robot — Project Notes

A consolidated record of the bring-up of the WaveShare UGV Rover outdoor
delivery robot: ROS 2 Jazzy stack, dual-EKF localization, Nav2, and the
ESP32 motor/IMU driver. Covers only what was actually built and fixed.

> **Note on naming:** earlier revisions of this document and the codebase
> called the robot "Beast." The hardware is actually a **WaveShare UGV
> Rover** — a different product. Several parameters were initially set to
> Beast values and were wrong; see §2 and §4. Treat any remaining "Beast"
> reference in code or config as a bug to correct.

---

## 1. System Architecture

The robot is a dual-computer design:

- **Upper computer** — Raspberry Pi running ROS 2 Jazzy. Owns perception,
  localization, Nav2, and mission logic.
- **Lower computer** — ESP32 on the WaveShare driver board. Runs the motor
  PID loop, reads the IMU (ICM-20948) and battery monitor, and reports
  telemetry. Flashed with WaveShare `ugv_base_ros` firmware.

The two communicate over a **GPIO UART** link, JSON line protocol, 115200 baud.

ROS package layout:
- `ugv_bringup` — top-level `robot.launch.py`, includes all subsystems.
- `ugv_base` — `motor_driver`, `battery_monitor`, `teleop_watchdog`.
- `ugv_perception` — RPLIDAR driver + `laser_filters` chain.
- `ugv_localization` — static TFs, dual EKF, `navsat_transform`, watchdog.
- `ugv_navigation` — Nav2 + `mission_manager`.

Remote visualization runs RViz on a separate machine (Ubuntu in UTM on a Mac).

> **IMU source:** the IMU comes from the **ESP32 / ICM-20948 over UART**
> (`motor_driver`). An earlier prototype streamed IMU from a phone app over
> WiFi — that path is retired. The phone now provides **GPS only**. See
> `PYNAVIGATION.md` for the history of that migration.

---

## 2. Hardware / Device Map

Confirmed by inspection — important because several were initially wrong:

| Device | Port | Detail |
|---|---|---|
| ESP32 motor controller | `/dev/ttyAMA0` (GPIO UART) | 115200 baud, JSON. NOT USB. |
| RPLIDAR C1 | `/dev/ttyUSB0` | CP210x USB-serial (idVendor `10c4`, idProduct `ea60`) |
| Camera | USB | 5 MP, 160-degree wide-angle RGB. No depth. |

**Platform:** WaveShare UGV Rover — **6-wheel 4WD skid-steer**, 80 mm
diameter wheels, ~1.3 m/s max speed. Four of the six wheels carry encoders.
Skid-steer means the platform scrubs on every turn — relevant to odometry
(see §4, §5).

**Key lesson:** the ESP32 is on the Pi's GPIO UART, not USB. A `ttyACM0`
device (CH343, idVendor `1a86`) appears but is a separate item, not the
motor controller. The GPIO UART must be explicitly enabled (see Section 9).

udev rules give stable names so port numbering can't drift:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```

---

## 3. ESP32 Serial Protocol (verified from live capture)

The firmware sends **one combined feedback frame**, type `T:1001`, e.g.:

```json
{"T":1001,"L":0,"R":0,"ax":-54,"ay":-60,"az":8342,
 "gx":29,"gy":10,"gz":15,"mx":-282,"my":326,"mz":1676,
 "odl":-4,"odr":0,"v":1191}
```

- `L`, `R` — left/right wheel speed (~m/s, 0 at rest)
- `odl`, `odr` — cumulative wheel-odometer ticks (signed)
- `ax/ay/az`, `gx/gy/gz` — accel/gyro RAW integer counts (ICM-20948)
- `mx/my/mz` — magnetometer raw counts (unused)
- `v` — battery voltage in CENTIVOLTS (1191 -> 11.91 V)

There is **no** `dt` field, and **no** separate `T:1002`/`T:1003` frames.
An earlier driver assumed a three-frame split with SI-unit IMU values —
that assumption was wrong and produced silent failure.

> **Gyro raw values vary between captures.** Different stationary captures
> have shown `gz` of 15, 17, and 64 counts. This spread is expected — gyro
> bias is nonzero, unit-specific, and temperature-dependent. It is the
> reason an explicit bias correction is required (see §4).

Commands Pi -> ESP32:
- Drive: `{"T":1,"L":<float>,"R":<float>}` — L/R as fraction of max speed
- Stop: `{"T":1,"L":0,"R":0}`
- Heartbeat: `{"T":6}` — firmware stops motors after ~3 s of silence

---

## 4. motor_driver Node

The node opens `/dev/ttyAMA0` @ 115200, reads newline-delimited JSON,
filters `T:1001`, and publishes:

- `/wheel/odom` — `nav_msgs/Odometry`, host-integrated from L/R speeds.
  `dt` is computed host-side from arrival time (not on the wire).
- `/imu/data` — `sensor_msgs/Imu`, raw counts converted to SI.
- `/motor/voltage_raw`, `/motor/status`, `/diagnostics`.

Subscribes `/cmd_vel` and `/e_stop`.

### IMU calibration (from a level, stationary capture)

- Accel: `az` reads ~ +8370 counts at 1 g -> **scale = 9.81 / 8370 = 0.001172**
- `az` is POSITIVE when level, so **no axis inversion** is needed.
- **Gyro bias.** The ESP32 sends RAW gyro counts; it does NOT bias-correct
  them. `motor_driver` must subtract the bias itself. A stationary capture
  of the published `angular_velocity.z` measured a residual of
  **0.00208 rad/s** (~15.6 counts at the 131 counts/dps scale), which
  produced a constant ~0.1°/s heading drift and smeared scans in RViz. The
  fix is the `imu_gyro_bias_*` parameters in `motor_driver`, subtracted
  after scaling. Verify the correction with:
  `ros2 topic echo /imu/data --field angular_velocity.z` — at rest it must
  average ≈ 0.
  - *History:* an earlier version of this section claimed the bias was
    "subtracted before scaling" with a value of ~17 counts. That was
    inaccurate — no subtraction existed in `motor_driver`, and the true
    residual for this unit is larger. Bias drifts with temperature; if
    calibrated cold, re-check after warm-up.
- Gyro **scale** assumes ICM-20948 ±250 dps (131 counts/dps). Still to
  verify by motion: rotate the robot 90° by hand and confirm the integral
  of `angular_velocity.z` ≈ 1.57 rad. (Bias and scale are separate: bias
  fixes drift at rest, scale fixes error during turns.)

All scales, biases, and axis-sign flips are ROS parameters in
`motor_params.yaml` — re-tunable without code changes.

### Geometry parameters

`motor_params.yaml` geometry must reflect the **Rover**, not the Beast:

- `wheel_radius_m: 0.04` — confirmed, Rover has 80 mm wheels. (Earlier 0.05
  was a Beast value.)
- `max_speed_mps: 1.3` — confirmed Rover spec. (Earlier 0.35 was Beast.)
- `wheel_base_m` — **measure your unit.** Left-right wheel centre-to-centre,
  then calibrate empirically with a 360° rotation test. Skid-steer effective
  track width differs from the geometric value.
- `ticks_per_rev` — **calibrate.** Set `wheel_radius_m` first, then drive a
  tape-measured straight line and scale by (measured / reported).

### Drive direction

If the robot moves opposite to commands, add invert parameters. The rule:
command forward -> robot moves forward -> `/wheel/odom twist.linear.x`
positive -> `pose.position.x` increases. Command and encoder signs must be
fixed together, or odometry will lie to the EKF.

---

## 5. Localization — Dual EKF

Two `robot_localization` EKF instances:

- **Local EKF** (`ekf_filter_node_odom`) — fuses wheel odom + IMU.
  `world_frame: odom`. Publishes `odom -> base_link`. GPS-independent.
- **Global EKF** (`ekf_filter_node_map`) — adds GPS via `navsat_transform`.
  `world_frame: map`. Publishes `map -> odom`.

`navsat_transform` converts `/gps/fix` (phone GPS over WebSocket) to
`/odometry/gps`. With the phone off, the global EKF still runs but `map`
just drifts with odometry — for RViz, use Fixed Frame `odom` in that case.

> **Skid-steer caveat.** Wheel-odometry *yaw* on a 6-wheel skid-steer is
> unreliable — the wheels scrub on every turn. Do not fuse wheel-odom yaw
> rate: set the `odom0` `vyaw` slot to `false` in both EKFs and let the
> gyro own heading rate. Likewise, the IMU has no working magnetometer, so
> do not fuse IMU absolute yaw — fuse gyro *rate* only (`imu0` orientation
> yaw slot `false`, `vyaw` slot `true`).

**TF ownership rule:** the local EKF owns `odom -> base_link`. `motor_driver`
must therefore run with `publish_tf: false`, or two publishers fight over
that transform and the pose oscillates.

---

## 6. Perception — LIDAR + Filter Chain

Pipeline: `/scan` -> range -> shadows -> speckle -> `/scan_filtered`.

- RPLIDAR C1 supports scan modes **Standard** and **DenseBoost** only.
  `Boost` is unsupported and causes a start-scan failure — use `Standard`.
- `/scan_filtered` is the observation source for both Nav2 costmaps and
  the collision monitor.

---

## 7. Nav2 Configuration

Outdoor configuration, no static map:

- Both costmaps `rolling_window: true`.
- MPPI controller (`FollowPath`) for smooth path following.
- NavFn global planner.
- Goal tolerance widened for GPS-grade accuracy.

### The cmd_vel safety chain (critical)

The velocity pipeline must be a strict linear chain, with the collision
monitor as the **sole** final gate before the motors:

```
controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
   -> collision_monitor -> /cmd_vel/nav2 -> teleop_watchdog mux -> /cmd_vel -> motor_driver
```

Required remappings in the navigation launch file:
- `controller_server`: `("cmd_vel", "cmd_vel_nav")`
- `velocity_smoother`: `("cmd_vel", "cmd_vel_nav")` (publishes `cmd_vel_smoothed`)
- `collision_monitor` (yaml): `cmd_vel_in_topic: cmd_vel_smoothed`,
  `cmd_vel_out_topic: /cmd_vel/nav2`

If the controller or smoother publishes directly to `/cmd_vel/nav2`, the
collision monitor is bypassed and its stop command is overwritten — the
robot will not stop for obstacles. This was the cause of the robot
crashing into things.

Collision monitor has `PolygonStop` and `PolygonSlow` zones reading
`/scan_filtered`. Verify with `ros2 topic echo /collision_monitor_state`.

### Footprint and transform_tolerance

- The Rover footprint must be **measured** (body ≈ 0.23 × 0.25 m, plus wheel
  overhang — measure the outer extremes). Use the same footprint in both
  costmaps. With a polygon footprint, MPPI's `CostCritic` needs
  `consider_footprint: true`; with a circular `robot_radius`, `false`.
- `transform_tolerance` should be `0.3`, not `0.1` — the Pi 4B plus a remote
  RViz VM make 0.1 s too tight (see §10).

---

## 8. Common Failure Patterns & Fixes

Recurring issues, with the lessons that generalize:

- **`LaunchConfiguration` mistakes.** `LaunchConfiguration` takes no
  `value_type`; that kwarg belongs on `ParameterValue`. To coerce a launch
  arg to a typed parameter use `ParameterValue(LaunchConfiguration(...),
  value_type=bool)`. A launch argument must also be declared with
  `DeclareLaunchArgument` AND added to the returned `LaunchDescription`.

- **Launch include helper.** A `LaunchConfiguration` is iterable, so a
  helper testing `hasattr(v, '__iter__')` mis-routes it. `launch_arguments`
  accepts substitutions natively — pass `list(kwargs.items())` directly.

- **YAML "sequence should be of same type" parse errors.** The `rcl` params
  parser requires homogeneous sequences. In `robot_localization` covariance
  arrays, every element must be a float — write `0.0`, never bare `0`. In
  `laser_filters` count parameters (`neighbors`, `window`, `filter_window`)
  must be bare integers.

- **`laser_filters` ROS 2 config format.** The filter chain is a MAP of
  `filter1:`, `filter2:`, ... keys — NOT a YAML list with `- name:`. The
  list form is ROS 1 and fails to parse under `rcl`.

- **MPPI "controller period more than model dt".** `model_dt` must equal
  `1 / controller_frequency` (e.g. 10 Hz -> `model_dt: 0.1`).

- **MPPI footprint error.** With `robot_radius` set (circular), the
  `CostCritic` must have `consider_footprint: false`. `true` requires an
  explicit footprint polygon.

- **`bt_navigator` "Empty Tree" / segfault.** Do not set
  `default_nav_to_pose_bt_xml: ""` — an empty string is taken literally in
  Jazzy. Delete the line so the built-in default BT is used. Likewise do
  not specify a full `plugin_lib_names` list of standard nodes; Jazzy loads
  them by default and a duplicate registration crashes the node.

- **Stale installed files.** `ros2 launch` and node params load from
  `install/`, not `src/`. YAML files in `share/` are COPIED, not symlinked,
  even with `--symlink-install` — always `colcon build` after editing a
  config and verify the installed copy.

- **Node not getting its params.** A `Node(...)` must explicitly list its
  params file in `parameters=[...]`. If omitted, every parameter falls back
  to its in-code `declare_parameter` default. (Check this for
  `motor_driver` — its in-code geometry defaults are stale Beast values, so
  a missing params file would silently run wrong geometry.)

---

## 9. Raspberry Pi GPIO UART Setup

The ESP32 link needs the GPIO UART explicitly enabled.

In `/boot/firmware/config.txt`:
```
enable_uart=1
dtoverlay=disable-bt
```

`disable-bt` routes the stable PL011 UART to the GPIO pins (`/dev/ttyAMA0`)
instead of the jitter-prone mini-UART.

In `/boot/firmware/cmdline.txt`: remove any `console=serial0,...` /
`console=ttyAMA0,...` token.

Disable the serial login console:
```
sudo systemctl disable --now serial-getty@ttyAMA0.service
sudo systemctl disable --now hciuart
```

Reboot. The user must be in the `dialout` group to open the port.

**Note:** consistent garbage on a serial read means a baud mismatch, not a
wiring fault — force the rate with `stty -F /dev/ttyAMA0 115200 raw -echo`
before testing. Total silence means no power / wrong device / no signal.
On an assembled UGV, "ESP32 silent" usually means the driver board is not
on main battery power (the ESP32 is not powered by the Pi).

---

## 10. Remote RViz (UTM VM on Mac)

- An `.rviz` config file is fully portable — copy it to the VM.
- The IMU display needs `ros-jazzy-rviz-imu-plugin` installed on the VM.
- Set RViz Fixed Frame to `odom` when there is no GPS, or `map` will drift.
- For UTM, the VM network should be Bridged so it shares the LAN.

**TF over the network.** `/tf_static` is published `TRANSIENT_LOCAL`
(latched once); `/tf` is `VOLATILE` and high-rate. Across a VM boundary one
or the other can fail to arrive while topics still appear in
`ros2 topic list`. If the TF tree is incomplete on the VM, force unicast
DDS discovery — Cyclone DDS with explicit `<Peer>` entries and
`<AllowMulticast>false</AllowMulticast>`, the same `RMW_IMPLEMENTATION` on
both machines. Nav2 itself runs on the Pi, so a VM-only TF gap is a
visualization problem, not a navigation one.

**Clock skew — "TF extrapolation into future" errors.** If RViz logs
`Lookup would require extrapolation into the future`, the Pi and the VM
clocks are not synchronized. A 10–50 ms offset is typical of an unsynced VM
(the hypervisor pauses/resumes the guest). Fix by running NTP on both
machines — install `chrony` on the Pi and in the VM, and confirm with
`timedatectl` that the offset is down to single-digit milliseconds. Do NOT
use `use_sim_time` to mask this. Raising `transform_tolerance` to 0.3 (see
§7) gives additional slack.

---

## 11. Diagnostic Quick Reference

```bash
# ESP32 link — should stream T:1001 JSON
timeout 3 cat /dev/ttyAMA0

# Driver output
ros2 topic hz /wheel/odom
ros2 topic echo /imu/data --once          # az should be ~ +9.81 level & still
ros2 topic echo /imu/data --field angular_velocity.z   # ~0 at rest after bias fix

# TF tree
#   NOTE: confirm the real lidar frame name with `tf2_echo --frames`.
#   It is NOT necessarily `laser`; substitute the actual name below.
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link <laser_frame>
ros2 topic hz /tf /tf_static

# Perception
ros2 topic hz /scan /scan_filtered

# Nav2 state
ros2 lifecycle get /bt_navigator          # must be 'active'
ros2 action list | grep navigate

# Safety chain — obstacle in front, watch for stop
ros2 topic echo /collision_monitor_state
ros2 topic echo /cmd_vel/nav2

# Plan-only (preview a path without driving)
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

---

## 12. Known Open Items

Carried forward, not yet resolved at time of writing:

- **Gyro scale unverified.** `imu_gyro_scale` (131 counts/dps) assumes
  ±250 dps mode — confirm with a hand-rotation test (rotate 90°, integral
  of `angular_velocity.z` ≈ 1.57 rad). *Gyro bias is separately handled —
  see §4 — and is no longer an open scale-vs-bias ambiguity.*
- **Gyro bias temperature drift.** The static bias correction in §4 is valid
  at the temperature it was measured. Re-check `angular_velocity.z` at rest
  after warm-up; if the residual returns, consider online bias estimation.
- **Robot geometry.** `wheel_base_m` and `ticks_per_rev` still need physical
  measurement / calibration for the Rover (see §4).
- **Drive direction.** Verify forward command -> forward motion -> positive
  `/wheel/odom` linear.x; add invert parameters if mirrored.
- **Heartbeat type.** `{"T":6}` keep-alive is unconfirmed; if the robot
  stops dead ~3 s after a command, the firmware wants a different keepalive.
- **Lidar frame name.** Confirm the C1 driver's published `frame_id` and
  propagate the correct name into RViz, the costmap `observation_sources`,
  and §11.
- **Speed tuning.** Lower MPPI `vx_max` and `velocity_smoother` max_velocity
  to ~0.15 m/s while tuning obstacle response; raise `CostCritic`
  `cost_weight` and `local_costmap` `update_frequency` as needed.

### Resolved

- **EKF update rate.** Both EKFs previously warned "failed to meet update
  rate" at 30 Hz on the Pi 4B. `frequency` in `ekf.yaml` is now 15 Hz.
  Resolved.
