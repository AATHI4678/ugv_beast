#!/usr/bin/env bash
# deploy.sh — rsync workspace to Pi and rebuild
# Usage: ./scripts/deploy.sh [pi_ip]
# Example: ./scripts/deploy.sh ubuntu@192.168.4.1

PI=${1:-"ubuntu@192.168.4.1"}
REMOTE_WS="~/ugv_ws"

set -e
echo "=== Syncing workspace to $PI ==="
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  "$(dirname "$0")/../../../" "$PI:$REMOTE_WS/src/"

echo "=== Building on Pi ==="
ssh "$PI" "
  source /opt/ros/jazzy/setup.bash &&
  cd ~/ugv_ws &&
  colcon build --executor sequential --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
"

echo "=== Deploy complete. Restart service: ==="
echo "ssh $PI 'sudo systemctl restart ugv_robot'"
