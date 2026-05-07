# Delta Robot ROS 2 Package

This directory now contains a ROS 2 Python package named `delta_robot_ros`.

## Nodes

- `orin_controller`: `pygame` control UI intended for the Jetson Orin Nano.
- `rpi_robot_node`: Raspberry Pi robot-side node that replaces the ESP32 firmware behavior with ROS 2 subscriptions, kinematics, trajectory handling, and servo output.

## Default Pin Mapping

The original ESP32 logical arm mapping was:

- Arm 1: ESP32 GPIO 26
- Arm 2: ESP32 GPIO 25
- Arm 3: ESP32 GPIO 33

The Raspberry Pi node keeps that arm ordering, but uses Raspberry Pi BCM GPIO numbers for actual output. The current defaults are:

- Arm 1: BCM 12
- Arm 2: BCM 13
- Arm 3: BCM 18

Change them with:

```bash
export DELTA_SERVO_PIN_ARM1=12
export DELTA_SERVO_PIN_ARM2=13
export DELTA_SERVO_PIN_ARM3=18
```

## Servo Backend

- `dry-run`: safe logging-only mode
- `pigpio`: real Raspberry Pi servo output

Enable hardware control with:

```bash
sudo pigpiod
export DELTA_SERVO_BACKEND=pigpio
```

## Build

From the workspace root that contains this package:

```bash
colcon build --packages-select delta_robot_ros
source install/setup.bash
```

## Run

Jetson Orin Nano:

```bash
ros2 run delta_robot_ros orin_controller
```

Raspberry Pi:

```bash
ros2 run delta_robot_ros rpi_robot_node
```

Or launch the Pi node with environment-configurable pins:

```bash
ros2 launch delta_robot_ros delta_robot.launch.py servo_backend:=dry-run arm1_pin:=12 arm2_pin:=13 arm3_pin:=18
```
