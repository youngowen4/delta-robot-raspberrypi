#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
fi

ros2 run delta_robot_ros orin_controller
