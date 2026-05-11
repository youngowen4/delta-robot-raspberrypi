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

- `auto`: prefer `lgpio`, then `pigpio`, then fall back to `dry-run`
- `dry-run`: safe logging-only mode
- `lgpio`: native GPIO backend for Ubuntu/Raspberry Pi setups with `python3-lgpio`
- `pigpio`: real Raspberry Pi servo output

Enable `lgpio` hardware control with:

```bash
sudo apt update
sudo apt install -y python3-lgpio liblgpio1
export DELTA_SERVO_BACKEND=lgpio
```

Enable `pigpio` hardware control with:

```bash
sudo pigpiod
export DELTA_SERVO_BACKEND=pigpio
```

Servo pulse width defaults to `1000us` to `2000us`, which is a safer starting range for many hobby servos than `500us` to `2000us`. Override if needed:

```bash
export DELTA_SERVO_PULSE_MIN_US=1000
export DELTA_SERVO_PULSE_MAX_US=2000
```

Motion deadband can be tuned to suppress small target corrections and joint chatter:

```bash
export DELTA_POSITION_DEADBAND_MM=0.75
export DELTA_THETA_DEADBAND_DEG=0.8
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
ros2 launch delta_robot_ros delta_robot.launch.py servo_backend:=auto arm1_pin:=12 arm2_pin:=13 arm3_pin:=18
```

On Ubuntu Server for Raspberry Pi, `lgpio` is typically the better default if `pigpiod` is unavailable from apt.
