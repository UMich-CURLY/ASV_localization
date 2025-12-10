#!/bin/bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ROS2/drift_ros2"

if [[ ! -d "${WS_DIR}" ]]; then
  echo "Unable to find ROS2/drift_ros2 workspace."
  exit 1
fi

echo "[drift_ros2] Building custom_sensor_msgs..."
pushd "${WS_DIR}" >/dev/null
colcon build --packages-select custom_sensor_msgs
source install/setup.bash

echo "[drift_ros2] Building full workspace..."
colcon build --symlink-install
echo "[drift_ros2] Build complete. Source ${WS_DIR}/install/setup.bash to use the nodes."
popd >/dev/null
