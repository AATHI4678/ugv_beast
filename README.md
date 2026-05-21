# UGV Rover Delivery Robot — ROS 2 Jazzy Workspace

**Hardware:** WaveShare UGV Rover (6-wheel 4WD skid-steer) · Raspberry Pi 4B · RPLIDAR C1 · ESP32/ICM-20948 IMU · Android phone (GPS only, via WebSocket)
**Software:** ROS 2 Jazzy on Ubuntu 24.04 Server
**Target accuracy:** 1–3 m outdoor position · obstacle avoidance via 2D lidar

> **Architecture note:** the IMU is the **ESP32's ICM-20948**, read over GPIO
> UART by `motor_driver`. The phone provides **GPS only**. An earlier prototype
> streamed IMU from the phone as well; that path is retired. See
> `PYNAVIGATION.md` for the migration history and `PYPROGRESS.md` for the
> current architecture.

---

## Package Map

```
ugv_ws/src/
├── ugv_interfaces/        Custom msgs, srvs, actions
├── ugv_base/              Motor driver (UART→ESP32, incl. IMU), battery monitor, teleop watchdog
├── phone_sensor_bridge/   WebSocket client (phone GPS) → /gps/fix /gps/vel
├── ugv_localization/      Dual EKF + navsat_transform + TF + localization watchdog
├── ugv_perception/        RPLIDAR C1 driver + outdoor laser filter chain
├── ugv_navigation/        Nav2 outdoor config + mission manager + waypoint converter
└── ugv_bringup/           Master launch, RViz config, systemd, setup scripts
```

## Topic Map

| Topic | Type | Source | Consumer |
|---|---|---|---|
| `/scan` | LaserScan | sllidar_ros2 | laser_filters |
| `/scan_filtered` | LaserScan | laser_filters | Nav2 costmaps |
| `/imu/data` | Imu | motor_driver (ESP32/UART) | both EKFs |
| `/wheel/odom` | Odometry | motor_driver (ESP32 encoders) | both EKFs |
| `/gps/fix` | NavSatFix | phone_sensor_bridge | navsat_transform |
| `/gps/vel` | TwistWithCovarianceStamped | phone_sensor_bridge | global EKF |
| `/odometry/gps` | Odometry | navsat_transform | global EKF |
| `/odometry/local` | Odometry | local EKF | Nav2, debugging |
| `/odometry/global` | Odometry | global EKF | Nav2 global planner |
| `/cmd_vel/nav2` | Twist | Nav2 (via velocity_smoother) | teleop_watchdog |
| `/cmd_vel/teleop` | Twist | joystick/keyboard | teleop_watchdog |
| `/cmd_vel` | Twist | teleop_watchdog (mux) | motor_driver |
| `/delivery_status` | DeliveryStatus | mission_manager | operator/dashboard |
| `/battery_state` | BatteryState | battery_monitor | mission_manager |
| `/e_stop` | Bool | localization_watchdog / operator | motor_driver |
| `/localization_ok` | Bool | localization_watchdog | mission_manager |

> Note: `navsat_transform` consumes `/imu/data` for the GPS heading transform
> only if `use_odometry_yaw: false`. With the EKF providing fused yaw, prefer
> `use_odometry_yaw: true` — see `PYPROGRESS.md § 5`.

---

## Build Instructions

### 1. Flash Pi and install ROS 2

```bash
# On the Pi (Ubuntu 24.04):
bash ~/ugv_ws/src/ugv_bringup/scripts/setup_pi.sh
```

Or manually follow the steps in `PYNAVIGATION.md` Part A (A0).

### 2. Clone the RPLIDAR driver

```bash
cd ~/ugv_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### 3. Install Python dependencies

```bash
pip3 install websockets    # phone GPS WebSocket bridge
sudo apt install python3-pyserial   # ESP32 UART link
```

### 4. Install ROS dependencies

```bash
cd ~/ugv_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build (sequential to avoid OOM on Pi 4B)

```bash
cd ~/ugv_ws
colcon build --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 6. Environment setup

Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
# Optional: CycloneDDS tuning for hotspot WiFi
# export CYCLONEDDS_URI=file://$(ros2 pkg prefix ugv_bringup)/share/ugv_bringup/config/cyclonedds.xml
```

> The ESP32 link also needs the Pi's **GPIO UART** explicitly enabled
> (`enable_uart=1`, `dtoverlay=disable-bt`, serial console disabled). See
> `PYPROGRESS.md § 9`.

---

## Deployment Instructions

### One-time calibration steps

#### A. Set magnetic declination

Look up your location at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
Convert degrees to radians and edit `ugv_localization/config/ekf.yaml`:
```yaml
navsat_transform_node:
  ros__parameters:
    magnetic_declination_radians: <your value>   # radians, location-specific
```

#### B. Measure robot geometry

Verify `ugv_base/config/motor_params.yaml` against the **Rover** (not Beast):
```yaml
wheel_radius_m: 0.04       # confirmed: Rover has 80 mm wheels
max_speed_mps: 1.3         # confirmed Rover spec
wheel_base_m: <measured>   # measure left-right wheel centre-to-centre
ticks_per_rev: <calibrated>
```

**Encoder calibration:**
```bash
# Set wheel_radius_m first. Then drive a tape-measured straight line
# (e.g. 2 m) at low speed and check /wheel/odom:
ros2 topic echo /wheel/odom --once
# Scale ticks_per_rev by (measured_distance / reported_distance).
```

**Track-width calibration:** rotate the robot exactly 360° in place and
compare odom yaw; adjust `wheel_base_m` until they match. Skid-steer scrub
makes the effective track width differ from the geometric one — calibrate
empirically.

#### C. Verify IMU mounting and calibration

The IMU is the ESP32's ICM-20948. Adjust the `base_link → imu_link`
transform in `ugv_localization/launch/static_tf.launch.py` to where the
ESP32 board physically sits.

```bash
# Gyro bias — robot perfectly still and level:
ros2 topic echo /imu/data --field angular_velocity.z
# Must average ≈ 0. A nonzero average is uncorrected gyro bias;
# set imu_gyro_bias_z in motor_params.yaml. See PYPROGRESS.md § 4.

# Gyro scale — rotate the robot 90° by hand:
# the integral of angular_velocity.z should be ≈ 1.57 rad.
```

#### D. GPS velocity frame walk test (CRITICAL)

```bash
ros2 topic echo /gps/vel
# Walk due north at ~1 m/s for 10 seconds
# linear.y ≈ +1.0 = ENU frame (correct, use as-is)
# linear.x ≈ +1.0 = NED frame (edit bridge to swap axes)
```

#### E. Record home position

Stand the robot at its charging/docking location:
```bash
ros2 topic echo /gps/fix --once
# Copy latitude and longitude to ugv_robot.service home_latitude/home_longitude args
```

### Start the robot

**Manual (for testing):**
```bash
ros2 launch ugv_bringup robot.launch.py \
  phone_ip:=192.168.4.2 \
  home_latitude:=44.305488 \
  home_longitude:=-79.574232
```

**Auto-start on boot:**
```bash
sudo systemctl enable --now ugv_hotspot.service
sudo systemctl enable --now ugv_robot.service
```

**Desktop dev mode (no hardware):**
```bash
ros2 launch ugv_bringup robot_dev.launch.py
```

---

## Testing Checklist

### Stage 1: Individual subsystems

- [ ] RPLIDAR C1 visible: `ros2 topic hz /scan` → ~8 Hz
- [ ] Laser filter working: `ros2 topic hz /scan_filtered` → same rate, fewer points
- [ ] IMU publishing: `ros2 topic hz /imu/data` (rate set by ESP32 firmware — confirm actual value)
- [ ] IMU bias OK: `ros2 topic echo /imu/data --field angular_velocity.z` ≈ 0 at rest
- [ ] Wheel odom publishing: `ros2 topic hz /wheel/odom`
- [ ] GPS publishing: `ros2 topic hz /gps/fix` → ~1 Hz
- [ ] Motor driver serial connected (check `/diagnostics`)
- [ ] Battery monitor publishing: `ros2 topic echo /battery_state --once`

### Stage 2: TF tree

```bash
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link → {<laser_frame>, imu_link, gps}
# Confirm the real lidar frame name with: ros2 run tf2_ros tf2_echo --frames
# (it is NOT necessarily "laser")
```

### Stage 3: EKF convergence

```bash
ros2 topic echo /odometry/local --once   # odom-frame smooth odom
ros2 topic echo /odometry/global --once  # GPS-corrected global pose
# After ~30s outdoors with GPS fix: covariance[0] and [7] should be < 5.0
```

### Stage 4: Outdoor walk test

```bash
# Move robot by hand (or teleop) and check:
ros2 topic echo /odometry/global
# Position should track movement, yaw should match actual heading
```

### Stage 5: Teleop

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
# Publishes to /cmd_vel/teleop — watchdog muxes to /cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel/teleop
```

### Stage 6: Waypoint mission

```bash
ros2 run ugv_navigation waypoint_converter \
  --ros-args -p mission_file:=$(ros2 pkg prefix ugv_navigation)/share/ugv_navigation/missions/example_mission.yaml
ros2 topic echo /delivery_status
```

---

## Troubleshooting

### "TF extrapolation into future" errors
- Most common with remote RViz: **clock skew** between the Pi and the
  visualization VM. Sync both with `chrony`/NTP; confirm with `timedatectl`.
- Verify `use_sim_time: false` in all Nav2 params.
- Raise `transform_tolerance` to 0.3 in Nav2 configs (0.1 is too tight on a
  Pi 4B with a remote viewer).

### Robot drives in circles / heading wrong
1. **Gyro bias** not subtracted — check `angular_velocity.z` ≈ 0 at rest;
   set `imu_gyro_bias_z` in `motor_params.yaml`.
2. **Gyro scale** wrong — run the 90° rotation test (integral ≈ 1.57 rad).
3. Wrong magnetic declination — recompute and update `ekf.yaml`.
4. Wrong `imu_link` static transform — verify the ESP32 board mounting.
5. On skid-steer, ensure the EKF is not fusing wheel-odom yaw — gyro owns
   heading rate. See `PYPROGRESS.md § 5`.

### Scans smeared / doubled in RViz
- Set the LaserScan display Decay Time to 0.
- Usually accumulating heading error — see the gyro bias/scale items above.
- Check Fixed Frame: `odom` when no GPS, `map` with GPS.

### Position jumps on GPS update
- GPS covariance too optimistic → inflate GPS position variance in global EKF
  ```yaml
  odom1_pose_rejection_threshold: 5.0  # reject outliers > 5 m
  ```
- Increase `navsat_transform_node.delay` to 5–8s.

### Phantom obstacles in sunlight
- Tighten range filter: `upper_threshold: 6.0`
- Add a physical sunshade hood over the RPLIDAR.

### Motors don't move
1. Check serial connection: `ls -la /dev/rplidar` and the ESP32 UART device.
2. Check udev rules applied: `sudo udevadm trigger`
3. Check e-stop status: `ros2 topic echo /e_stop --once`
4. Test in sim_mode first: `ros2 launch ugv_base ugv_base.launch.py sim_mode:=true`
5. ESP32 silent on an assembled UGV usually means the driver board is not on
   main battery power (the ESP32 is not powered by the Pi).

### Nav2 won't start / lifecycle error
- EKF must publish `/odometry/local` and TF `odom→base_link` before Nav2 starts.
- The startup delay in `robot.launch.py` handles this; increase if needed.
- Check: `ros2 lifecycle get /bt_navigator`

### WiFi dropout causes robot to stop
- Expected behaviour (watchdog stops on sensor timeout). Note: WiFi now
  carries GPS only — an IMU/odometry-driven local EKF still runs through a
  dropout, but GPS corrections pause.
- Increase `cmd_vel_timeout_s` in `motor_params.yaml` to 1.5–2.0s for looser
  recovery.
- Use Pi as hotspot (`ugv_hotspot.service`) for a shorter WiFi path.

### Out of memory during build
```bash
colcon build --executor sequential --parallel-workers 1
```
Or increase swap to 4GB in `/etc/dphys-swapfile`.

---

## Example Waypoint Mission

```bash
cat > /tmp/my_mission.yaml << 'EOF'
mission:
  id: "test_001"
  arrival_tolerance_m: 2.0
  waypoints:
    - id: "point_a"
      lat: 44.305612
      lon: -79.574109
      alt: 234.0
    - id: "point_b"
      lat: 44.305540
      lon: -79.574350
      alt: 234.0
EOF

ros2 run ugv_navigation waypoint_converter \
  --ros-args -p mission_file:=/tmp/my_mission.yaml

ros2 topic echo /delivery_status

# Emergency stop
ros2 service call /emergency_stop ugv_interfaces/srv/EmergencyStop \
  "{stop: true, reason: 'manual stop'}"

# Release e-stop
ros2 service call /emergency_stop ugv_interfaces/srv/EmergencyStop \
  "{stop: false, reason: 'cleared'}"
```

---

## Recommended Future Upgrades

| Upgrade | Benefit | Complexity |
|---|---|---|
| RTK GPS (u-blox F9P) | Sub-metre localization | Medium |
| OAK-D Lite depth camera | 3D obstacle detection, curb awareness | Medium |
| 3D LiDAR (Livox MID-360) | Full 3D scene understanding | High |
| LTE modem (Sixfab) | Outdoor remote operations without hotspot | Low |
| Hardware E-stop relay | Safety compliance for pedestrian areas | Low |
| Encoders on all six wheels | Better odometry (only four are encoded) | Low |
| Jetson Orin Nano | Replace Pi for faster Nav2 + depth processing | Medium |
| SLAM cartographer | Indoor capability | Medium |

---

## Architecture Notes

### Why dual EKF?

The local EKF (odom frame) gives Nav2's local planner a smooth, continuous
odometry stream. GPS jumps would cause lurching if fed directly to the local
planner. The global EKF (map frame) absorbs GPS updates and provides the
slowly-corrected global pose Nav2's global planner needs. This is the
standard outdoor Nav2 pattern.

### Why rolling global costmap?

There is no pre-built map. The global costmap grows around the robot's
current position, updated by lidar returns. The robot relies on GPS for
global positioning, not the costmap.

### Why MPPI controller?

MPPI (Model Predictive Path Integral) generates smooth trajectories suitable
for skid-steer robots on sidewalks. It handles the large goal tolerance (2 m
GPS accuracy) better than DWB. Batch size reduced from 2000→1000 for Pi 4B
CPU budget.

### Skid-steer odometry

The Rover is a 6-wheel skid-steer platform: it turns by driving the left and
right wheel banks at different speeds, scrubbing the wheels sideways on every
turn. Wheel-odometry *yaw* is therefore unreliable — heading should come from
the gyro, not the wheels. Only four of the six wheels carry encoders. See
`PYPROGRESS.md § 4–5`.

### IMU — ESP32 / ICM-20948

The IMU is on the ESP32 driver board and sends **raw** accel/gyro counts over
UART. `motor_driver` converts counts to SI, removes gravity, subtracts gyro
bias, and publishes `/imu/data`. The ICM-20948 magnetometer is unused
(magnetic interference from the motors), so there is no absolute yaw — the
EKF integrates gyro rate for heading. Gyro bias and scale must both be
calibrated; see the deployment calibration steps and `PYPROGRESS.md § 4`.

### Why CycloneDDS?

FastDDS (default) is significantly heavier on ARM (Pi 4B). CycloneDDS reduces
CPU usage by ~20–30% at idle, which matters on a 4-core 1.8 GHz Pi.

---

## Documentation

- `PYPROGRESS.md` — current-architecture project notes (source of truth):
  hardware map, ESP32 protocol, driver, dual EKF, Nav2, failure patterns.
- `PYNAVIGATION.md` — outdoor navigation guide; split into still-valid,
  needs-editing, and obsolete (phone-IMU) parts.
