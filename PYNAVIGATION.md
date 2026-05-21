# Outdoor Navigation Guide

> **⚠️ ARCHITECTURE NOTICE — READ FIRST**
>
> This document was originally written for a **phone-IMU prototype**: a phone
> app streamed *both* GPS and IMU over WiFi. **The robot has since migrated.**
>
> **Current architecture:** the IMU is the **ESP32 / ICM-20948** on the
> WaveShare driver board, read over **GPIO UART** by `motor_driver`. The phone
> now provides **GPS only**. Wheel odometry comes from the ESP32 encoders.
>
> The robot is a **WaveShare UGV Rover** — 6-wheel 4WD skid-steer, 80 mm
> wheels, ~1.3 m/s max. (Earlier drafts said "Beast"; that was wrong.)
>
> This guide is therefore split into three parts:
>
> - **Part A — Still Valid.** Architecture-neutral; use as-is.
> - **Part B — Needs Editing.** Salvageable, but contains values/config that
>   must change for the ESP32-IMU Rover. Each item says what to change.
> - **Part C — Obsolete (Phone-IMU).** Describes the old sensor path. Kept
>   for history only. **Do not follow for the current robot.** For the
>   current IMU/EKF/driver setup, `PYPROGRESS.md` is the source of truth.

---

## Table of Contents

- [Part A — Still Valid](#part-a--still-valid)
  - [A0: OS and ROS 2 Installation](#a0-os-and-ros-2-installation)
  - [A1: RPLIDAR C1 Setup](#a1-rplidar-c1-setup)
  - [A2: Lidar Filtering for Outdoor Use](#a2-lidar-filtering-for-outdoor-use)
  - [A3: GPS Velocity Walk Test](#a3-gps-velocity-walk-test)
  - [A4: Nav2 Outdoor Concepts](#a4-nav2-outdoor-concepts)
  - [A5: Power and Reliability](#a5-power-and-reliability)
- [Part B — Needs Editing](#part-b--needs-editing)
  - [B1: TF Tree](#b1-tf-tree)
  - [B2: Dual EKF Configuration](#b2-dual-ekf-configuration)
  - [B3: Nav2 Config Values](#b3-nav2-config-values)
  - [B4: Tuning Checklist](#b4-tuning-checklist)
  - [B5: Topic Map](#b5-topic-map)
- [Part C — Obsolete (Phone-IMU)](#part-c--obsolete-phone-imu)
- [Useful References](#useful-references)

---

# Part A — Still Valid

Architecture-neutral material. None of it depends on where the IMU comes
from, so it carried through the migration unchanged.

## A0: OS and ROS 2 Installation

### Flash Ubuntu 24.04

Jazzy requires **Ubuntu 24.04 (Noble)**. Flash Ubuntu Server 24.04 for
Raspberry Pi 4 with Raspberry Pi Imager. Pre-configure SSH, WiFi, and your
username.

**Storage tip:** Use an SSD over USB 3.0 if possible. The Pi 4B's SD card
slot is a bottleneck and SD cards die from ROS log writes.

### Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe

# New key/repo setup for Jazzy (uses ros2-apt-source package)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install ros-jazzy-ros-base ros-dev-tools -y
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install required packages

```bash
sudo apt install -y \
  ros-jazzy-rplidar-ros \
  ros-jazzy-robot-localization \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-laser-filters \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2
```

### Performance tuning for Pi 4B

Jazzy on a Pi 4B is heavier than Humble. Do these upfront:

**Switch to CycloneDDS** (FastDDS is too CPU-hungry):

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

**Increase swap** (RAM gets tight when colcon builds nav2):

```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile   # set CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

If you have a 2GB Pi 4B, prefer the 8GB version.

### Create workspace

```bash
mkdir -p ~/nav_ws/src
cd ~/nav_ws
colcon build
echo "source ~/nav_ws/install/setup.bash" >> ~/.bashrc
```

**First-build tip:** Build packages sequentially the first time to avoid OOM:

```bash
colcon build --executor sequential --parallel-workers 1
```

> **Note:** The ESP32 link also needs the Pi's **GPIO UART enabled**
> (`enable_uart=1`, `dtoverlay=disable-bt`, serial console disabled). That
> setup is hardware-specific to the current architecture — see
> `PYPROGRESS.md § 9`. It is not part of this (originally phone-only) guide.

## A1: RPLIDAR C1 Setup

### Hardware connection

Plug the C1 into a USB port. Find its device path:

```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0   # quick fix
```

### Permanent udev rule (recommended)

```bash
sudo nano /etc/udev/rules.d/rplidar.rules
```

Add:

```
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"
```

Reload:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Install the SLAMTEC driver

```bash
cd ~/nav_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select sllidar_ros2
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
```

### Verify

```bash
ros2 topic echo /scan --once
ros2 run rviz2 rviz2
```

> **⚠️ Frame name:** the lidar's `frame_id` is whatever the C1 driver
> publishes — confirm it with `ros2 run tf2_ros tf2_echo --frames`. It is
> **not** necessarily `laser`. Earlier drafts of this guide and the
> diagnostics in `PYPROGRESS.md § 11` assumed `laser`; a live `tf2_echo`
> showed that frame does not exist. Use the real name in the RViz Fixed
> Frame and in every costmap `observation_sources` entry.

### Outdoor mounting tips

- Mount the lidar **low and tilted very slightly downward** so it looks at
  ground-level obstacles rather than the sky.
- Add a **small sunshade hood** above the lidar housing.
- Expect sporadic dropouts and noise on bright days.

> **Scan modes:** the RPLIDAR C1 supports **Standard** and **DenseBoost**
> only. `Boost` is unsupported and causes a start-scan failure — use
> `Standard`. (Carried in from `PYPROGRESS.md § 6`.)

## A2: Lidar Filtering for Outdoor Use

Outdoor scans need cleaning. This filter chain is unchanged by the migration.

`config/scan_filter.yaml`:

```yaml
scan_filter_chain:
  - name: range
    type: laser_filters/LaserScanRangeFilter
    params:
      use_message_range_limits: false
      lower_threshold: 0.15
      upper_threshold: 8.0   # ignore returns past 8m outdoors
      lower_replacement_value: .inf
      upper_replacement_value: .inf
  - name: shadows
    type: laser_filters/ScanShadowsFilter
    params:
      min_angle: 10.0
      max_angle: 170.0
      neighbors: 1
      window: 1
  - name: speckle
    type: laser_filters/LaserScanSpeckleFilter
    params:
      filter_type: 0
      max_range: 8.0
      max_range_difference: 0.5
      filter_window: 2
```

Run with `scan_to_scan_filter_chain` from `laser_filters`. Output to
`/scan_filtered`. Use that as Nav2's input.

> **YAML format reminders** (from `PYPROGRESS.md § 8`): the filter chain is a
> **map** of `filter1:`, `filter2:` keys under `rcl`, not a ROS 1 `- name:`
> list. Count params (`neighbors`, `window`, `filter_window`) must be bare
> integers. Note the snippet above uses the ROS 1 list form — convert it to
> the map form before use under ROS 2 Jazzy.

## A3: GPS Velocity Walk Test

GPS **velocity** still comes from the phone, so this test remains valid.
(GPS *position* and velocity are the only things the phone still provides.)

GPS-derived velocity could be in ENU world frame, NED world frame, or
phone-local frame. Confirm which before fusing it.

**Procedure:**

1. Stand outside, get a clear GPS fix.
2. `ros2 topic echo /gps/vel`
3. Walk **due north** at a steady pace (~1 m/s) for 10 seconds.
4. Note the velocity components.

**Interpretation:**

| Behavior | Frame | Action |
|----------|-------|--------|
| `linear.y` ≈ +1.0, `linear.x` ≈ 0 | **ENU** (East-North-Up) | Most common; fuse directly as world velocity |
| `linear.x` ≈ +1.0, `linear.y` ≈ 0 | **NED** | Negate Y, swap axes if needed |
| Both nonzero, change as you turn | **Phone-local** | Rotate via orientation before fusing |
| Random / chaotic | Below GPS noise floor | Walk faster (≥ 0.5 m/s) |

The EKF wiring assumes ENU. If your test shows otherwise, convert in the
bridge before publishing.

## A4: Nav2 Outdoor Concepts

The *concepts* here are architecture-independent. Specific YAML values are
in Part B.

- **Don't use AMCL or SLAM-based localization outdoors.** The `map` frame
  comes from the global EKF, not a pre-built map.
- **Rolling global costmap.** There is no static map; the global costmap
  grows around the robot's current position. `rolling_window: true` on
  *both* costmaps.
- **Lidar is for obstacle detection, not localization.** It feeds the Nav2
  costmaps; it does not localize the robot.
- **GPS waypoint demo.** `ros-navigation/navigation2_tutorials` (jazzy
  branch) contains `nav2_gps_waypoint_follower_demo` — a working
  `logged_waypoint_follower` to adapt.
- **Start from the Jazzy base config**, not Humble tutorials:
  `cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`. Jazzy has
  param-format changes (BT XML defaults, plugin namespaces, MPPI default).

## A5: Power and Reliability

- **Quality 5V/3A+ supply or LiFePO4 BMS** — undervolts kill USB devices
  silently and the lidar misbehaves.
- **Powered USB hub** if you add peripherals beyond the lidar.
- **`systemd` services** for launch files so the stack auto-starts on boot.
- **Log to tmpfs ramdisk** and rotate aggressively — SD writes corrupt the
  filesystem over time.
- **Status indicators** — LEDs/LCD for GPS-fix, lidar-publishing,
  EKF-converged.
- **Pi as WiFi AP** — running the Pi as a hotspot for the phone removes
  router dependency and reduces latency.
- **Phone power** — the phone (still the GPS source) should be on a
  powerbank or the robot's 5V rail; streaming drains battery fast.
- **Keep an SSH session ready** during testing.

---

# Part B — Needs Editing

Salvageable material that contains values or config wrong for the
ESP32-IMU Rover. Each subsection states the required change.

## B1: TF Tree

The tree shape is unchanged:

```
map → odom → base_link → {<laser_frame>, imu_link, gps}
```

**What must change:**

- **`base_link → imu_link`.** The old transform described a *phone* lying on
  the robot roof (Android axes, 90° yaw rotation). The IMU is now the
  **ESP32 board's ICM-20948** — `imu_link` must describe where that *board*
  physically sits. Re-measure the translation and rotation from `base_link`
  to the ESP32's IMU. The phone-axis commentary no longer applies.
- **`base_link → <laser_frame>`.** Use the real lidar frame name (see A1),
  not `laser`.
- **`base_link → gps`.** Still the phone's GPS antenna — valid, but update
  the offset to where the phone is actually mounted now.

The `imu_link` transform is the root for the gyro scale/bias tests, so an
incorrect transform here corrupts those — fix it before calibrating.

## B2: Dual EKF Configuration

> **The original Stage 5 EKF in this guide is the phone-IMU EKF and is wrong
> for the current robot.** Do not copy it. `PYPROGRESS.md` and the live
> `ekf.yaml` are authoritative. The differences that matter:

| Setting | Old (phone) value | Current (ESP32) value | Why |
|---|---|---|---|
| IMU source | `phone_sensor_bridge` | `motor_driver` (UART) | Migration |
| `imu0_remove_gravitational_acceleration` | `false` | **`true`** | ESP32 sends raw counts *with* gravity; phone pre-removed it |
| IMU yaw (orientation) slot | `true` | **`false`** | ICM-20948 magnetometer unused → no valid absolute yaw; fuse gyro *rate* only |
| `imu0_relative` | `true` | `false` | ESP32 IMU is rigidly mounted |
| `frequency` | `30.0` | **`15.0`** | Pi 4B misses 30 Hz deadlines |
| `sensor_timeout` | `0.5` | **`0.1`** | UART is reliable; WiFi tolerance no longer needed |
| `imu0_queue_size` | `25` | `10` | No WiFi jitter to buffer |
| Wheel odometry | commented out | **primary sensor** | ESP32 encoders now feed both EKFs |
| Wheel-odom `vyaw` slot | n/a | **`false`** | Skid-steer scrub makes wheel yaw unreliable; gyro owns heading rate |

> **Migration hazard — the gyro bias.** The phone app internally
> bias-corrected its `angularVelocity` before sending it. The ESP32 sends
> **raw gyro counts**, and that bias subtraction did **not** transfer to
> `motor_driver` automatically. A residual ~0.00208 rad/s was measured at
> rest, producing a constant ~0.1°/s heading drift. The fix is an explicit
> gyro-bias subtraction in `motor_driver` (`imu_gyro_bias_z` parameter).
> Verify at rest: `ros2 topic echo /imu/data --field angular_velocity.z`
> must average ≈ 0.

**navsat_transform:** still needed (phone GPS → map XY). Update
`magnetic_declination_radians` for your actual location (look up at
ngdc.noaa.gov/geomag). Once IMU absolute yaw is disabled, set
`use_odometry_yaw: true` so navsat uses the EKF's fused yaw.

## B3: Nav2 Config Values

The Stage 7 costmap YAML in the original guide is a **generic starting
point** and has been superseded by the project's tuned Nav2 config. If
referring to it, note:

- **Footprint.** The old `0.25 m` square is a guess. The Rover footprint
  must be measured including wheel overhang (body ≈ 0.23 × 0.25 m, wheels
  proud → measure outer extremes). Use the same footprint in *both*
  costmaps. With a polygon footprint, MPPI's `CostCritic` needs
  `consider_footprint: true`; with a circular `robot_radius`, it needs
  `false`.
- **`transform_tolerance`.** Raise from `0.1` to `0.3` — the Pi 4B plus a
  remote-RViz VM boundary makes 0.1 s too tight.
- The project's real Nav2 config also adds the **collision-monitor safety
  chain** (controller → `cmd_vel_nav` → smoother → `cmd_vel_smoothed` →
  collision_monitor → `/cmd_vel/nav2`), MPPI critic tuning, and tuned
  inflation. See `PYPROGRESS.md § 7`.

## B4: Tuning Checklist

> **The original Stage 12 checklist blames phone-IMU causes that no longer
> apply.** Corrected mapping for the current robot:

| Symptom | Old (phone) diagnosis — **stale** | Current diagnosis |
|---|---|---|
| Heading drifts / robot circles | Wrong declination; phone near motors | **Gyro bias** not subtracted in `motor_driver`; also verify gyro *scale* (90° test) |
| TF "extrapolation into future" | Phone timestamps; set `use_app_timestamps:false` | **Pi↔VM clock skew** — sync both with `chrony`/NTP; raise `transform_tolerance` |
| Yaw wrong direction on rotation | Phone bridge `euler_deg_to_quaternion` sign | Not applicable — that function is not in the signal path. Check `imu_gyro_invert_z` and the `imu_link` TF |
| Position jumps on GPS update | (still valid) | Inflate GPS position covariance / `odom1` rejection threshold |
| Phantom obstacles in sun | (still valid) | Tighten range filter; hood the lidar |
| Robot lurches when GPS reacquires | (still valid) | Increase `navsat_transform` `delay` |

## B5: Topic Map

Update the source of `/imu/data`:

| Topic | Type | Source | Consumer |
|-------|------|--------|----------|
| `/scan` | LaserScan | sllidar_ros2 | laser_filters |
| `/scan_filtered` | LaserScan | laser_filters | Nav2 costmaps |
| `/imu/data` | Imu | **`motor_driver` (ESP32/UART)** — *was phone_sensor_bridge* | both EKFs, navsat_transform |
| `/wheel/odom` | Odometry | **`motor_driver` (ESP32 encoders)** | both EKFs |
| `/gps/fix` | NavSatFix | phone_sensor_bridge | navsat_transform |
| `/gps/vel` | TwistWithCovarianceStamped | phone_sensor_bridge | global EKF |
| `/odometry/gps` | Odometry | navsat_transform | global EKF |
| `/odometry/local` | Odometry | local EKF | Nav2, debugging |
| `/odometry/global` | Odometry | global EKF | Nav2 global planner |

---

# Part C — Obsolete (Phone-IMU)

> **Do not follow this section for the current robot.** It documents the
> pre-migration prototype where a phone app streamed **both** IMU and GPS
> over WiFi. It is retained only as a record of the earlier design and to
> explain where migrated bugs (e.g. the dropped gyro-bias correction)
> originated. For the current IMU path, use `PYPROGRESS.md`.

The following original stages are obsolete in whole or in part:

- **Stage 2 — Phone App Sensor Bridge.** The `phone_sensor_bridge` node read
  IMU from `ws://<ip>:2343/IMU`. The IMU half is dead; only the GPS route
  (`/GPS`) remains relevant. The `euler_deg_to_quaternion` helper, the IMU
  message format, `dataQuality` covariance scaling, and the IMU WebSocket
  thread no longer apply.
- **Stage 3 — Verify Phone Sensor Data**, IMU portions. The phone IMU rate
  check, the IMU-display orientation test, and the stationary-acceleration
  "gravity already removed" check are all phone-specific. (The GPS velocity
  walk test survives — promoted to **A3**.)
- **Stage 4 — Phone IMU frame caveat.** The Android-axis discussion and the
  phone-on-roof `imu_link` transform are obsolete. (The TF tree itself
  survives, with edits — see **B1**.)
- **Stage 5 — phone-IMU EKF.** `imu0_remove_gravitational_acceleration:
  false`, IMU absolute-yaw fusion, `imu0_relative: true`, `frequency: 30`,
  `sensor_timeout: 0.5`, `queue_size: 25`, wheel odom commented out — all
  wrong for the ESP32 robot. (See **B2** for the corrected table.)
- **"Phone IMU trust level"** architecture note — obsolete.

**Why this section is kept:** the phone architecture pre-removed gravity and
bias-corrected the gyro *inside the phone*. When the IMU moved to the ESP32,
those processing steps had no equivalent and were silently lost — which is
the documented origin of the gyro-bias drift. Keeping Part C makes that
history traceable.

The full original phone-IMU text (bridge node source, launch files, message
formats) is preserved in version control / the original `PYNAVIGATION.md`
revision if needed.

---

## Useful References

- ROS 2 Jazzy docs: https://docs.ros.org/en/jazzy/
- robot_localization: https://docs.ros.org/en/jazzy/p/robot_localization/
- Nav2 GPS demo: https://github.com/ros-navigation/navigation2_tutorials (jazzy branch)
- SLAMTEC RPLIDAR ROS 2 driver: https://github.com/Slamtec/sllidar_ros2
- Magnetic declination calculator: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
- REP-103 (ROS coordinate conventions): https://www.ros.org/reps/rep-0103.html
- **`PYPROGRESS.md`** — current-architecture source of truth (ESP32 IMU,
  dual EKF, Nav2 safety chain, hardware map).
