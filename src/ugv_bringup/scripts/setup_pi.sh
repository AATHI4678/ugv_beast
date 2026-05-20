#!/usr/bin/env bash
# setup_pi.sh — one-time setup on a fresh Pi 4B running Ubuntu 24.04
# Run as the robot user (ubuntu), not root.

set -e

echo "=== Installing ROS 2 Jazzy ==="
sudo apt update && sudo apt upgrade -y
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

export ROS_APT_SOURCE_VERSION=$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-robot-localization \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-laser-filters \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2 \
  python3-websockets \
  python3-pyserial \
  python3-colcon-common-extensions

echo "=== CycloneDDS configuration ==="
cat >> ~/.bashrc << 'EOF'
source /opt/ros/jazzy/setup.bash
source ~/ugv_ws/install/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
EOF

echo "=== Swap configuration (2GB for build) ==="
sudo dphys-swapfile swapoff || true
sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

echo "=== udev rules ==="
sudo tee /etc/udev/rules.d/99-rplidar.rules << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"
EOF
# ESP32 UART (adjust vendor/product ID for your CH340/CP2102 chip)
sudo tee -a /etc/udev/rules.d/99-rplidar.rules << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="esp32"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "=== RPLIDAR sllidar_ros2 driver ==="
mkdir -p ~/ugv_ws/src
cd ~/ugv_ws/src
if [ ! -d sllidar_ros2 ]; then
  git clone https://github.com/Slamtec/sllidar_ros2.git
fi

echo "=== Workspace initial build ==="
cd ~/ugv_ws
colcon build --executor sequential --parallel-workers 1

echo "=== systemd services ==="
SHARE=$(ros2 pkg prefix ugv_bringup 2>/dev/null)/share/ugv_bringup/systemd

sudo cp "$SHARE/ugv_hotspot.service" /etc/systemd/system/
sudo cp "$SHARE/ugv_robot.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ugv_hotspot.service
# Enable robot service only after first successful manual test
# sudo systemctl enable ugv_robot.service

echo ""
echo "=== Setup complete! ==="
echo "Next steps:"
echo "  1. Edit /etc/systemd/system/ugv_robot.service with your home GPS coords"
echo "  2. Test manually: ros2 launch ugv_bringup robot.launch.py"
echo "  3. Then: sudo systemctl enable --now ugv_robot"
