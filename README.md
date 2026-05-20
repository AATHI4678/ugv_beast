# UGV Beast Delivery Robot — ROS 2 Jazzy Workspace

**Hardware:** WaveShare UGV Beast · Raspberry Pi 4B · RPLIDAR C1 · Android phone (GPS + IMU via WebSocket)  
**Software:** ROS 2 Jazzy on Ubuntu 24.04 Server  
**Target accuracy:** 1–3 m outdoor position · obstacle avoidance via 2D lidar

---

## Package Map

```
ugv_ws/src/
├── ugv_interfaces/        Custom msgs, srvs, actions
├── ugv_base/              Motor driver (UART→ESP32), battery monitor, teleop watchdog
├── phone_sensor_bridge/   WebSocket client (phone app IMU+GPS) → /imu/data /gps/fix /gps/vel
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
| `/imu/data` | Imu | phone_sensor_bridge | both EKFs, navsat_transform |
| `/gps/fix` | NavSatFix | phone_sensor_bridge | navsat_transform |
| `/gps/vel` | TwistWithCovarianceStamped | phone_sensor_bridge | global EKF |
| `/wheel/odom` | Odometry | motor_driver | both EKFs |
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

---

## Build Instructions

### 1. Flash Pi and install ROS 2

```bash
# On the Pi (Ubuntu 24.04):
bash ~/ugv_ws/src/ugv_bringup/scripts/setup_pi.sh
```

Or manually follow the steps in `PYNAVIGATION.md § Stage 0`.

### 2. Clone the RPLIDAR driver

```bash
cd ~/ugv_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### 3. Install Python dependencies

```bash
pip3 install websockets
sudo apt install python3-pyserial
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

---

## Deployment Instructions

### One-time calibration steps

#### A. Set magnetic declination

Look up your location at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml  
Convert degrees to radians and edit `ugv_localization/config/ekf.yaml`:
```yaml
navsat_transform_node:
  ros__parameters:
    magnetic_declination_radians: -0.1833   # ← your value here
```

#### B. Measure robot geometry

Verify `ugv_base/config/motor_params.yaml`:
```yaml
wheel_base_m: 0.295        # measure track width (centre-to-centre of wheels)
wheel_radius_m: 0.0525     # measure actual wheel radius
ticks_per_rev: 1560        # verify with a calibration spin (see below)
```

**Encoder calibration:**
```bash
# Mark a starting point, drive exactly 1m forward, check /wheel/odom
ros2 topic echo /wheel/odom --once
# pose.pose.position.x should read ~1.0
# Adjust ticks_per_rev if off.
```

#### C. Verify phone IMU mounting

Adjust `ugv_localization/launch/static_tf.launch.py` `imu_link` transform:
```bash
ros2 launch ugv_localization static_tf.launch.py
# Rotate robot 90° clockwise (from above) → yaw in /odometry/local should decrease ~1.57 rad
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
- [ ] Phone bridge connected: `ros2 topic hz /imu/data` → 40-60 Hz
- [ ] GPS publishing: `ros2 topic hz /gps/fix` → ~1 Hz
- [ ] Motor driver serial connected (check `/diagnostics`)
- [ ] Battery monitor publishing: `ros2 topic echo /battery_state --once`

### Stage 2: TF tree

```bash
ros2 run tf2_tools view_frames
# Should show: map → odom → base_link → {laser, imu_link, gps}
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
# Install teleop
sudo apt install ros-jazzy-teleop-twist-keyboard
# Run (note: publishes to /cmd_vel/teleop — watchdog muxes to /cmd_vel)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel/teleop
```

### Stage 6: Waypoint mission

```bash
# Send example mission
ros2 run ugv_navigation waypoint_converter \
  --ros-args -p mission_file:=$(ros2 pkg prefix ugv_navigation)/share/ugv_navigation/missions/example_mission.yaml

# Monitor
ros2 topic echo /delivery_status
```

---

## Troubleshooting

### "TF extrapolation into future" errors
- Set `use_phone_timestamps: false` in `bridge_params.yaml` (default)
- Verify `use_sim_time: false` in all Nav2 params

### Robot drives in circles / heading wrong
1. Wrong magnetic declination — recompute and update `ekf.yaml`
2. Phone IMU yaw sign flip — the bridge negates yaw; check `euler_deg_to_quaternion`
3. Wrong `imu_link` static transform — verify mounting and re-measure transform angles

### Position jumps on GPS update
- GPS covariance too optimistic → inflate GPS position variance in global EKF
  ```yaml
  odom1_pose_rejection_threshold: 5.0  # reject outliers > 5 m
  ```
- Increase `navsat_transform_node.delay` to 5–8s

### Phantom obstacles in sunlight
- Tighten range filter: `upper_threshold: 6.0`
- Add a physical sunshade hood over the RPLIDAR

### Phone bridge "DroppedQuality" messages
- Phone IMU quality reporting may be conservative
- Lower `min_data_quality: 0.1` if the data looks good visually

### Motors don't move
1. Check serial connection: `ls -la /dev/rplidar /dev/esp32`
2. Check udev rules applied: `sudo udevadm trigger`
3. Check e-stop status: `ros2 topic echo /e_stop --once`
4. Test in sim_mode first: `ros2 launch ugv_base ugv_base.launch.py sim_mode:=true`

### Nav2 won't start / lifecycle error
- EKF must publish `/odometry/local` and TF `odom→base_link` before Nav2 starts
- The 8-second delay in `robot.launch.py` handles this; increase if needed
- Check: `ros2 lifecycle get /bt_navigator`

### WiFi dropout causes robot to stop
- Expected behaviour (watchdog stops on sensor timeout)
- Increase `cmd_vel_timeout_s` in `motor_params.yaml` to 1.5-2.0s for looser recovery
- Use Pi as hotspot (`ugv_hotspot.service`) for shorter WiFi path

### Out of memory during build
```bash
colcon build --executor sequential --parallel-workers 1
```
Or increase swap to 4GB in `/etc/dphys-swapfile`.

---

## Example Waypoint Mission

Edit coordinates for your location, then:

```bash
# Create a mission file
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

# Send to robot
ros2 run ugv_navigation waypoint_converter \
  --ros-args -p mission_file:=/tmp/my_mission.yaml

# Watch progress
ros2 topic echo /delivery_status

# Emergency stop if needed
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
| Additional wheel encoders | Better odometry on slippery surfaces | Low |
| Jetson Orin Nano | Replace Pi for faster Nav2 + depth processing | Medium |
| SLAM cartographer | Indoor capability | Medium |

---

## Architecture Notes

### Why dual EKF?

The local EKF (odom frame) gives Nav2's local planner a smooth, continuous
odometry stream. GPS jumps would cause lurching if fed directly to the local planner.
The global EKF (map frame) absorbs GPS updates and provides the slowly-corrected
global pose Nav2's global planner needs. This is the standard outdoor nav2 pattern.

### Why rolling global costmap?

There is no pre-built map. The global costmap grows around the robot's current
position, updated by lidar returns. The robot relies on GPS for global positioning,
not the costmap.

### Why MPPI controller?

MPPI (Model Predictive Path Integral) generates smooth trajectories suitable for
differential-drive robots on sidewalks. It handles the large goal tolerance (2m GPS
accuracy) better than DWB. Batch size reduced from 2000→1000 for Pi 4B CPU budget.

### Why CycloneDDS?

FastDDS (default) is significantly heavier on ARM (Pi 4B). CycloneDDS reduces
CPU usage by ~20-30% at idle, which matters on a 4-core 1.8 GHz Pi.

### Phone IMU trust level

Phone 9-axis sensor fusion (typically Qualcomm/MediaTek fusion stack) is excellent
for roll and pitch (~0.01 rad² covariance) but yaw is magnetometer-dependent.
If the phone is near motors or power cables, magnetic interference will corrupt yaw.
Mount the phone as far from current-carrying traces as possible.
