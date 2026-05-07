#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
SERVO_BACKEND="${DELTA_SERVO_BACKEND:-dry-run}"
ARM1_PIN="${DELTA_SERVO_PIN_ARM1:-12}"
ARM2_PIN="${DELTA_SERVO_PIN_ARM2:-13}"
ARM3_PIN="${DELTA_SERVO_PIN_ARM3:-18}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
  source "${SCRIPT_DIR}/install/setup.bash"
fi

export DELTA_SERVO_BACKEND="${SERVO_BACKEND}"
export DELTA_SERVO_PIN_ARM1="${ARM1_PIN}"
export DELTA_SERVO_PIN_ARM2="${ARM2_PIN}"
export DELTA_SERVO_PIN_ARM3="${ARM3_PIN}"

ros2 run delta_robot_ros rpi_robot_node
