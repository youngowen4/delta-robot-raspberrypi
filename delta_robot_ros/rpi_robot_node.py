import math
import os
import threading
from dataclasses import dataclass, field
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String

try:
    import lgpio
except ImportError:
    lgpio = None

try:
    import pigpio
except ImportError:
    pigpio = None


NODE_NAME = "delta_raspberry_pi_robot"
DEVICE_NAME = "Raspberry Pi 4 Model B"

COMMAND_TOPIC = os.getenv("DELTA_COMMAND_TOPIC", "/delta/control/command")
MODE_TOPIC = os.getenv("DELTA_MODE_TOPIC", "/delta/control/mode")
TARGET_TOPIC = os.getenv("DELTA_TARGET_TOPIC", "/delta/control/target")
STATUS_TOPIC = os.getenv("DELTA_STATUS_TOPIC", "/delta/control/status")

MODE_HOMING = 0
MODE_AUTOMATIC = 1
MODE_MANUAL = 2
MODE_KEYBOARD = 3

HOME_THETA = (0.0, 0.0, 0.0)
TRAJECTORY_DT = 0.02
HOMING_STEPS = 50
AUTO_LINE_STEPS = 50
AUTO_CIRCLE_STEPS = 100
AUTO_RADIUS = 130.0
AUTO_Z = -270.0
LOW_PASS_FACTOR = 0.4
LOW_PASS_MAX_STEP = 9.0

SERVO_PULSE_MIN_US = 500.0
SERVO_PULSE_MAX_US = 2000.0
SERVO_DEGREES_MIN = 0.0
SERVO_DEGREES_MAX = 180.0

DEFAULT_GPIO_PINS = (12, 13, 18)
SERVO_PINS = tuple(
    int(os.getenv(name, default))
    for name, default in zip(
        ("DELTA_SERVO_PIN_ARM1", "DELTA_SERVO_PIN_ARM2", "DELTA_SERVO_PIN_ARM3"),
        DEFAULT_GPIO_PINS,
    )
)

SERVO_BACKEND = os.getenv("DELTA_SERVO_BACKEND", "dry-run").strip().lower()
STATUS_PERIOD_SEC = float(os.getenv("DELTA_STATUS_PERIOD_SEC", "0.5"))


@dataclass
class PointTarget:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    mode: int = MODE_HOMING


@dataclass
class ThetaTarget:
    arm_1: float = 0.0
    arm_2: float = 0.0
    arm_3: float = 0.0


@dataclass
class RobotState:
    a: float = 60.0
    rf: float = 120.0
    re: float = 260.0
    z_min: float = -335.0
    z_max: float = -268.0
    r_max: float = 135.0
    end_effector_current: PointTarget = field(default_factory=PointTarget)
    theta_current: ThetaTarget = field(default_factory=ThetaTarget)
    end_effector_target: PointTarget = field(default_factory=PointTarget)
    theta_target: ThetaTarget = field(default_factory=ThetaTarget)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def lerp_point(start: PointTarget, end: PointTarget, t: float) -> PointTarget:
    return PointTarget(
        x=start.x + (end.x - start.x) * t,
        y=start.y + (end.y - start.y) * t,
        z=start.z + (end.z - start.z) * t,
        mode=end.mode,
    )


def low_pass_filter(current: PointTarget, target: PointTarget) -> PointTarget:
    dx = target.x - current.x
    dy = target.y - current.y
    dz = target.z - current.z

    step_x = dx * LOW_PASS_FACTOR
    step_y = dy * LOW_PASS_FACTOR
    step_z = dz * LOW_PASS_FACTOR

    step_length = math.sqrt(step_x * step_x + step_y * step_y + step_z * step_z)
    if step_length > LOW_PASS_MAX_STEP and step_length > 0.0:
        scale = LOW_PASS_MAX_STEP / step_length
        step_x *= scale
        step_y *= scale
        step_z *= scale

    return PointTarget(
        x=current.x + step_x,
        y=current.y + step_y,
        z=current.z + step_z,
        mode=target.mode,
    )


def parabolic_arc_point(start: PointTarget, end: PointTarget, height: float, t: float) -> PointTarget:
    point = lerp_point(start, end, t)
    point.z += 4.0 * height * t * (1.0 - t)
    return point


def passive_rotation(point: PointTarget, phi_degrees: int) -> PointTarget:
    if phi_degrees == 0:
        return PointTarget(point.x, point.y, point.z, point.mode)

    cosine = -0.5
    sine_abs = 0.86602540378
    sine = sine_abs if phi_degrees > 0 else -sine_abs

    return PointTarget(
        x=point.x * cosine + point.y * sine,
        y=-point.x * sine + point.y * cosine,
        z=point.z,
        mode=point.mode,
    )


def is_in_workspace(robot: RobotState, point: PointTarget) -> bool:
    if point.z > robot.z_max or point.z < robot.z_min:
        return False
    return point.x * point.x + point.y * point.y <= robot.r_max * robot.r_max


def calculate_inverse_kinematics(robot: RobotState, point: PointTarget) -> Optional[ThetaTarget]:
    phi_values = (0, 120, -120)
    rf2 = robot.rf * robot.rf
    re2 = robot.re * robot.re
    theta_values = []

    for phi in phi_values:
        rotated = passive_rotation(point, phi)
        rotated.x += robot.a

        distance = math.sqrt((rotated.x - robot.a) ** 2 + rotated.z**2)
        if distance < 1e-6:
            return None

        p_value = (rf2 - re2 + rotated.y * rotated.y + distance * distance) / (2.0 * distance)
        if p_value > robot.rf or p_value < 0.0:
            return None

        h_value_sq = rf2 - p_value * p_value
        if h_value_sq < 0.0:
            return None
        h_value = math.sqrt(h_value_sq)

        p_x = robot.a + p_value * (rotated.x - robot.a) / distance
        p_z = p_value * rotated.z / distance
        vector_x = -rotated.z
        vector_z = rotated.x - robot.a

        candidates = (
            (p_x + (h_value / distance) * vector_x, p_z + (h_value / distance) * vector_z),
            (p_x - (h_value / distance) * vector_x, p_z - (h_value / distance) * vector_z),
        )

        theta = None
        for candidate_x, candidate_z in candidates:
            if candidate_x >= robot.a and candidate_z <= 0.0:
                theta = math.atan2(candidate_z, candidate_x - robot.a)
                break

        if theta is None:
            return None
        theta_values.append(theta)

    return ThetaTarget(*theta_values)


def calculate_forward_kinematics(robot: RobotState, theta: ThetaTarget) -> Optional[PointTarget]:
    theta_values = (theta.arm_1, theta.arm_2, theta.arm_3)
    phi_values = (0, 120, -120)
    joints_global = []

    for theta_value, phi in zip(theta_values, phi_values):
        sin_value = math.sin(theta_value)
        cos_value = math.cos(theta_value)
        joint_local = PointTarget(x=robot.rf * cos_value, y=0.0, z=robot.rf * sin_value)
        joints_global.append(passive_rotation(joint_local, -phi))

    a1 = 2.0 * (joints_global[0].x - joints_global[1].x)
    b1 = 2.0 * joints_global[1].y
    c1 = 2.0 * (joints_global[0].z - joints_global[1].z)
    d1 = (
        joints_global[0].x * joints_global[0].x
        - joints_global[1].x * joints_global[1].x
        - joints_global[1].y * joints_global[1].y
        + joints_global[0].z * joints_global[0].z
        - joints_global[1].z * joints_global[1].z
    )

    a2 = 2.0 * (joints_global[0].x - joints_global[2].x)
    b2 = 2.0 * joints_global[2].y
    c2 = 2.0 * (joints_global[0].z - joints_global[2].z)
    d2 = (
        joints_global[0].x * joints_global[0].x
        - joints_global[2].x * joints_global[2].x
        - joints_global[2].y * joints_global[2].y
        + joints_global[0].z * joints_global[0].z
        - joints_global[2].z * joints_global[2].z
    )

    denominator = a1 * b2 - a2 * b1
    if abs(denominator) < 1e-6:
        return None

    a_value = (-d1 * b2 + d2 * b1) / denominator
    c_value = (a1 * d2 - a2 * d1) / denominator
    b_value = (-c1 * b2 + c2 * b1) / denominator
    d_value = (a1 * c2 - a2 * c1) / denominator
    a_shift = a_value - joints_global[0].x

    quadratic_b = b_value * b_value + d_value * d_value + 1.0
    quadratic_c = 2.0 * b_value * a_shift + 2.0 * c_value * d_value - 2.0 * joints_global[0].z
    quadratic_d = a_shift * a_shift + c_value * c_value + joints_global[0].z * joints_global[0].z - robot.re * robot.re

    delta = quadratic_c * quadratic_c - 4.0 * quadratic_b * quadratic_d
    if delta < 0.0:
        return None

    z0 = min(
        (-quadratic_c + math.sqrt(delta)) / (2.0 * quadratic_b),
        (-quadratic_c - math.sqrt(delta)) / (2.0 * quadratic_b),
    )

    return PointTarget(
        x=a_value + b_value * z0,
        y=c_value + d_value * z0,
        z=z0,
    )


def theta_to_tuple(theta: ThetaTarget) -> Tuple[float, float, float]:
    return theta.arm_1, theta.arm_2, theta.arm_3


class ServoBackend:
    def apply(self, theta: ThetaTarget) -> None:
        raise NotImplementedError

    def shutdown(self) -> None:
        return None


class DryRunServoBackend(ServoBackend):
    def __init__(self, logger, pins: Tuple[int, int, int]):
        self._logger = logger
        self._pins = pins

    def apply(self, theta: ThetaTarget) -> None:
        details = []
        for pin, angle_rad in zip(self._pins, theta_to_tuple(theta)):
            servo_angle = clamp(90.0 - math.degrees(angle_rad), SERVO_DEGREES_MIN, SERVO_DEGREES_MAX)
            pulse_width = SERVO_PULSE_MIN_US + (
                (servo_angle / SERVO_DEGREES_MAX) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)
            )
            details.append(f"GPIO {pin}: theta={angle_rad:.3f} rad servo={servo_angle:.2f} deg pulse={pulse_width:.0f}us")
        self._logger.info("Dry-run servo update | " + " | ".join(details))


class PigpioServoBackend(ServoBackend):
    def __init__(self, logger, pins: Tuple[int, int, int]):
        if pigpio is None:
            raise RuntimeError("pigpio Python package is not installed.")

        self._logger = logger
        self._pins = pins
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("Unable to connect to pigpio daemon. Start it with 'sudo pigpiod'.")

        for pin in pins:
            self._pi.set_mode(pin, pigpio.OUTPUT)

    def apply(self, theta: ThetaTarget) -> None:
        for pin, angle_rad in zip(self._pins, theta_to_tuple(theta)):
            servo_angle = clamp(90.0 - math.degrees(angle_rad), SERVO_DEGREES_MIN, SERVO_DEGREES_MAX)
            pulse_width = SERVO_PULSE_MIN_US + (
                (servo_angle / SERVO_DEGREES_MAX) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)
            )
            self._pi.set_servo_pulsewidth(pin, pulse_width)
            self._logger.debug(
                f"Applied servo output on GPIO {pin}: theta={angle_rad:.3f} rad servo={servo_angle:.2f} deg pulse={pulse_width:.0f}us"
            )

    def shutdown(self) -> None:
        for pin in self._pins:
            self._pi.set_servo_pulsewidth(pin, 0)
        self._pi.stop()


class LgpioServoBackend(ServoBackend):
    def __init__(self, logger, pins: Tuple[int, int, int]):
        if lgpio is None:
            raise RuntimeError("lgpio Python package is not installed.")
        if not hasattr(lgpio, "gpiochip_open"):
            raise RuntimeError("lgpio is installed, but gpiochip support is unavailable.")
        if not hasattr(lgpio, "tx_servo"):
            raise RuntimeError("lgpio is installed, but tx_servo support is unavailable.")

        self._logger = logger
        self._pins = pins
        self._handle = lgpio.gpiochip_open(0)

        for pin in pins:
            lgpio.gpio_claim_output(self._handle, pin)

    def apply(self, theta: ThetaTarget) -> None:
        for pin, angle_rad in zip(self._pins, theta_to_tuple(theta)):
            servo_angle = clamp(90.0 - math.degrees(angle_rad), SERVO_DEGREES_MIN, SERVO_DEGREES_MAX)
            pulse_width = int(
                SERVO_PULSE_MIN_US
                + ((servo_angle / SERVO_DEGREES_MAX) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US))
            )
            lgpio.tx_servo(self._handle, pin, pulse_width)
            self._logger.debug(
                f"Applied lgpio servo output on GPIO {pin}: theta={angle_rad:.3f} rad servo={servo_angle:.2f} deg pulse={pulse_width}us"
            )

    def shutdown(self) -> None:
        for pin in self._pins:
            try:
                lgpio.tx_servo(self._handle, pin, 0)
            except Exception:
                pass
            try:
                lgpio.gpio_free(self._handle, pin)
            except Exception:
                pass
        try:
            lgpio.gpiochip_close(self._handle)
        except Exception:
            pass


class DeltaRobotNode(Node):
    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        self.robot = RobotState()
        self.lock = threading.Lock()
        self.current_mode = MODE_HOMING
        self.command_target = PointTarget(z=-300.0, mode=MODE_HOMING)
        self.command_source = "startup"
        self.latest_status = "Initializing robot controller."
        self._status_dirty = True
        self._automatic_initialized = False
        self._automatic_step = 0
        self._automatic_type = 0
        self._automatic_start = PointTarget()
        self._automatic_end = PointTarget()
        self._homing_active = False
        self._homing_step = 0
        self._homing_start = ThetaTarget()
        self._home_point = PointTarget()

        self.mode_subscription = self.create_subscription(Int32, MODE_TOPIC, self._on_mode, 10)
        self.target_subscription = self.create_subscription(Point, TARGET_TOPIC, self._on_target, 10)
        self.command_subscription = self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self.control_timer = self.create_timer(TRAJECTORY_DT, self._control_step)
        self.status_timer = self.create_timer(STATUS_PERIOD_SEC, self._publish_status)

        self.servo_backend = self._create_servo_backend()
        self._initialize_home_state()

    def _create_servo_backend(self) -> ServoBackend:
        if SERVO_BACKEND == "pigpio":
            self.get_logger().info(f"Using pigpio servo backend on GPIO pins {SERVO_PINS}.")
            return PigpioServoBackend(self.get_logger(), SERVO_PINS)
        if SERVO_BACKEND == "lgpio":
            self.get_logger().info(f"Using lgpio servo backend on GPIO pins {SERVO_PINS}.")
            return LgpioServoBackend(self.get_logger(), SERVO_PINS)

        self.get_logger().info(
            f"Using dry-run servo backend on GPIO pins {SERVO_PINS}. Set DELTA_SERVO_BACKEND=lgpio or pigpio for hardware output."
        )
        return DryRunServoBackend(self.get_logger(), SERVO_PINS)

    def _initialize_home_state(self) -> None:
        home_theta = ThetaTarget(*HOME_THETA)
        home_point = calculate_forward_kinematics(self.robot, home_theta)
        if home_point is None:
            raise RuntimeError("Unable to calculate the robot home point from HOME_THETA.")

        self.get_logger().info(
            "Workspace limits configured as "
            f"R_MAX={self.robot.r_max:.2f}, Z_MIN={self.robot.z_min:.2f}, Z_MAX={self.robot.z_max:.2f}"
        )
        self.get_logger().info(
            "Computed home point from HOME_THETA="
            f"({HOME_THETA[0]:.3f}, {HOME_THETA[1]:.3f}, {HOME_THETA[2]:.3f}) "
            f"as X={home_point.x:.2f}, Y={home_point.y:.2f}, Z={home_point.z:.2f}"
        )

        with self.lock:
            self.robot.theta_current = home_theta
            self.robot.theta_target = home_theta
            self.robot.end_effector_current = home_point
            self.robot.end_effector_target = home_point
            self.command_target = PointTarget(home_point.x, home_point.y, home_point.z, MODE_HOMING)
            self._home_point = home_point
            self.latest_status = (
                f"{DEVICE_NAME} ready. Home point X={home_point.x:.2f} Y={home_point.y:.2f} Z={home_point.z:.2f}."
            )
            self._status_dirty = True

        self.servo_backend.apply(home_theta)

    def _on_mode(self, msg: Int32) -> None:
        with self.lock:
            previous_mode = self.current_mode
            self.current_mode = int(msg.data)
            self.command_target.mode = self.current_mode
            self.command_source = "mode topic"

            if self.current_mode == MODE_HOMING:
                self._start_homing()
                self.latest_status = "Homing requested."
            elif self.current_mode == MODE_AUTOMATIC:
                self._automatic_initialized = False
                self.latest_status = "Automatic trajectory enabled."
            elif self.current_mode == MODE_MANUAL:
                self.latest_status = "Manual target tracking enabled."
            elif self.current_mode == MODE_KEYBOARD:
                self.latest_status = "Keyboard coordinate target tracking enabled."
            else:
                self.latest_status = f"Unsupported mode {self.current_mode}; waiting for a valid command."
            if previous_mode == MODE_HOMING and self.current_mode != MODE_HOMING:
                self._homing_active = False
            self._status_dirty = True

    def _on_target(self, msg: Point) -> None:
        with self.lock:
            self.command_target.x = float(msg.x)
            self.command_target.y = float(msg.y)
            self.command_target.z = float(msg.z)
            self.command_target.mode = self.current_mode
            self.command_source = "target topic"
            self.latest_status = (
                f"Target updated from topic: X={self.command_target.x:.2f} Y={self.command_target.y:.2f} Z={self.command_target.z:.2f}."
            )
            self._status_dirty = True

    def _on_command(self, msg: String) -> None:
        command = (msg.data or "").strip()
        if not command:
            return

        parts = [part.strip() for part in command.split(",")]
        try:
            mode = int(parts[0])
        except ValueError:
            self.get_logger().warning(f"Ignoring invalid command payload: {command}")
            return

        with self.lock:
            self.current_mode = mode
            self.command_target.mode = mode
            if len(parts) >= 4:
                try:
                    self.command_target.x = float(parts[1])
                    self.command_target.y = float(parts[2])
                    self.command_target.z = float(parts[3])
                except ValueError:
                    self.get_logger().warning(f"Ignoring invalid numeric command payload: {command}")
                    return
            self.command_source = "command topic"
            if mode == MODE_HOMING:
                self._start_homing()
            elif mode == MODE_AUTOMATIC:
                self._automatic_initialized = False
            self.latest_status = f"Command accepted from command topic: {command}"
            self._status_dirty = True

    def _start_homing(self) -> None:
        self._homing_active = True
        self._homing_step = 0
        self._homing_start = ThetaTarget(
            self.robot.theta_current.arm_1,
            self.robot.theta_current.arm_2,
            self.robot.theta_current.arm_3,
        )

    def _control_step(self) -> None:
        with self.lock:
            mode = self.current_mode
            if mode == MODE_HOMING:
                next_point, next_theta, status = self._step_homing()
            elif mode == MODE_AUTOMATIC:
                next_point, next_theta, status = self._step_automatic()
            elif mode in (MODE_MANUAL, MODE_KEYBOARD):
                next_point, next_theta, status = self._step_manual()
            else:
                next_point = None
                next_theta = None
                status = f"Unsupported mode {mode}; no motion applied."

            if next_point is not None and next_theta is not None:
                self.robot.end_effector_current = next_point
                self.robot.end_effector_target = next_point
                self.robot.theta_current = next_theta
                self.robot.theta_target = next_theta
                self.latest_status = status
                self._status_dirty = True
            elif status and status != self.latest_status:
                self.latest_status = status
                self._status_dirty = True
                return
            else:
                return

        self.servo_backend.apply(next_theta)

    def _step_homing(self) -> Tuple[Optional[PointTarget], Optional[ThetaTarget], str]:
        if not self._homing_active:
            self._start_homing()

        self._homing_step += 1
        t = clamp(self._homing_step / HOMING_STEPS, 0.0, 1.0)
        theta = ThetaTarget(
            arm_1=self._homing_start.arm_1 + (HOME_THETA[0] - self._homing_start.arm_1) * t,
            arm_2=self._homing_start.arm_2 + (HOME_THETA[1] - self._homing_start.arm_2) * t,
            arm_3=self._homing_start.arm_3 + (HOME_THETA[2] - self._homing_start.arm_3) * t,
        )
        point = calculate_forward_kinematics(self.robot, theta)
        if point is None:
            self._homing_active = False
            return None, None, "Error: unable to calculate home point during homing."

        if self._homing_step >= HOMING_STEPS:
            self._homing_active = False
            return point, theta, "Homing completed."

        return point, theta, f"Homing in progress ({self._homing_step}/{HOMING_STEPS})."

    def _step_automatic(self) -> Tuple[Optional[PointTarget], Optional[ThetaTarget], str]:
        if not self._automatic_initialized:
            self._automatic_start = PointTarget(
                self.robot.end_effector_current.x,
                self.robot.end_effector_current.y,
                self.robot.end_effector_current.z,
                MODE_AUTOMATIC,
            )
            self._automatic_end = PointTarget(AUTO_RADIUS, 0.0, AUTO_Z, MODE_AUTOMATIC)
            self._automatic_step = 0
            self._automatic_type = 1
            self._automatic_initialized = True

        if self._automatic_type == 1:
            t = clamp(self._automatic_step / AUTO_LINE_STEPS, 0.0, 1.0)
            point = lerp_point(self._automatic_start, self._automatic_end, t)
            self._automatic_step += 1
            if self._automatic_step > AUTO_LINE_STEPS:
                self._automatic_type = 2
                self._automatic_step = 0
        else:
            t = clamp(self._automatic_step / AUTO_CIRCLE_STEPS, 0.0, 1.0)
            angle = t * 2.0 * math.pi
            point = PointTarget(
                x=AUTO_RADIUS * math.cos(angle),
                y=AUTO_RADIUS * math.sin(angle),
                z=AUTO_Z,
                mode=MODE_AUTOMATIC,
            )
            self._automatic_step += 1
            if self._automatic_step > AUTO_CIRCLE_STEPS:
                self._automatic_step = 0

        theta = self._point_to_theta(point)
        if theta is None:
            self.get_logger().warning(
                "Automatic trajectory rejected at "
                f"X={point.x:.2f} Y={point.y:.2f} Z={point.z:.2f} "
                f"from current X={self.robot.end_effector_current.x:.2f} "
                f"Y={self.robot.end_effector_current.y:.2f} "
                f"Z={self.robot.end_effector_current.z:.2f}"
            )
            return None, None, "Error: automatic trajectory point is outside reachable workspace."
        return point, theta, f"Automatic motion active at X={point.x:.2f} Y={point.y:.2f} Z={point.z:.2f}."

    def _step_manual(self) -> Tuple[Optional[PointTarget], Optional[ThetaTarget], str]:
        target = PointTarget(
            self.command_target.x,
            self.command_target.y,
            self.command_target.z,
            self.current_mode,
        )
        point = low_pass_filter(self.robot.end_effector_current, target)
        theta = self._point_to_theta(point)
        if theta is None:
            mode_name = "keyboard" if self.current_mode == MODE_KEYBOARD else "manual"
            self.get_logger().warning(
                f"{mode_name.capitalize()} target rejected after filtering at "
                f"X={point.x:.2f} Y={point.y:.2f} Z={point.z:.2f}; "
                f"requested X={target.x:.2f} Y={target.y:.2f} Z={target.z:.2f}; "
                f"current X={self.robot.end_effector_current.x:.2f} "
                f"Y={self.robot.end_effector_current.y:.2f} "
                f"Z={self.robot.end_effector_current.z:.2f}"
            )
            return None, None, f"Error: {mode_name} target is outside reachable workspace."

        if self.current_mode == MODE_KEYBOARD:
            return point, theta, f"Keyboard target tracking from {self.command_source}."
        return point, theta, f"Manual control tracking target from {self.command_source}."

    def _point_to_theta(self, point: PointTarget) -> Optional[ThetaTarget]:
        if not is_in_workspace(self.robot, point):
            self.get_logger().warning(
                f"Workspace bounds rejection at X={point.x:.2f} Y={point.y:.2f} Z={point.z:.2f}"
            )
            return None
        theta = calculate_inverse_kinematics(self.robot, point)
        if theta is None:
            self.get_logger().warning(
                f"Inverse kinematics rejection at X={point.x:.2f} Y={point.y:.2f} Z={point.z:.2f}"
            )
        return theta

    def _publish_status(self) -> None:
        with self.lock:
            if not self._status_dirty:
                return
            status_text = self.latest_status
            current_point = self.robot.end_effector_current
            current_theta = self.robot.theta_current
            mode = self.current_mode
            self._status_dirty = False

        message = String()
        message.data = (
            f"{status_text} | mode={mode} | "
            f"xyz=({current_point.x:.2f}, {current_point.y:.2f}, {current_point.z:.2f}) | "
            f"theta=({current_theta.arm_1:.3f}, {current_theta.arm_2:.3f}, {current_theta.arm_3:.3f})"
        )
        self.status_publisher.publish(message)

    def destroy_node(self) -> bool:
        self.servo_backend.shutdown()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = DeltaRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
