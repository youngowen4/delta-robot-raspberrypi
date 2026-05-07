import math
import os

import pygame
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String


DEVICE_NAME = "Jetson Orin Nano"
NODE_NAME = "delta_orin_controller"
COMMAND_TOPIC = os.getenv("DELTA_COMMAND_TOPIC", "/delta/control/command")
MODE_TOPIC = os.getenv("DELTA_MODE_TOPIC", "/delta/control/mode")
TARGET_TOPIC = os.getenv("DELTA_TARGET_TOPIC", "/delta/control/target")
STATUS_TOPIC = os.getenv("DELTA_STATUS_TOPIC", "/delta/control/status")


R_MAX = 135.0
Z_MIN = -335.0
Z_MAX = -268.0
DEFAULT_Z = -300.0
Z_STEP = 5.0


WIDTH, HEIGHT = 700, 700
CENTER = (WIDTH // 2, HEIGHT // 2)
DRAW_RADIUS = int((min(WIDTH, HEIGHT) // 2) * 0.8)
FRAME_RATE = 50
BACKGROUND_COLOR = (30, 30, 30)
GRID_COLOR = (80, 80, 80)
BOUNDARY_COLOR = (100, 100, 100)
TARGET_ACTIVE_COLOR = (0, 255, 255)
TARGET_IDLE_COLOR = (150, 150, 150)
SUCCESS_COLOR = (0, 255, 0)
ERROR_COLOR = (255, 0, 0)
INFO_COLOR = (200, 200, 200)


class DeltaControlBridge(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.mode_publisher = self.create_publisher(Int32, MODE_TOPIC, 10)
        self.target_publisher = self.create_publisher(Point, TARGET_TOPIC, 10)
        self.status_subscription = self.create_subscription(
            String,
            STATUS_TOPIC,
            self._on_status,
            10,
        )
        self.latest_status = "Waiting for ROS 2 controller feedback..."

    def _on_status(self, msg):
        self.latest_status = msg.data or "Controller status topic is empty."

    def publish_state(self, mode, x_value, y_value, z_value):
        mode_msg = Int32()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)

        target_msg = Point()
        target_msg.x = float(x_value)
        target_msg.y = float(y_value)
        target_msg.z = float(z_value)
        self.target_publisher.publish(target_msg)

        command_msg = String()
        if mode in (0, 1):
            command_msg.data = f"{mode}"
        else:
            command_msg.data = f"{mode},{x_value:.7f},{y_value:.7f},{z_value:.7f}"
        self.command_publisher.publish(command_msg)


def clamp(value, low, high):
    return max(low, min(value, high))


def within_workspace(x_value, y_value, z_value):
    return math.hypot(x_value, y_value) <= R_MAX and Z_MIN <= z_value <= Z_MAX


def map_mouse_to_robot(mouse_x, mouse_y):
    dx = mouse_x - CENTER[0]
    dy = mouse_y - CENTER[1]
    robot_target_x = (dx / DRAW_RADIUS) * R_MAX
    robot_target_y = -(dy / DRAW_RADIUS) * R_MAX

    distance = math.hypot(robot_target_x, robot_target_y)
    if distance > R_MAX:
        scale = R_MAX / distance
        robot_target_x *= scale
        robot_target_y *= scale

    return robot_target_x, robot_target_y


def draw_ui(
    screen,
    font,
    font_large,
    mode,
    robot_x,
    robot_y,
    robot_z,
    is_typing,
    input_text,
    local_status,
    ros_status,
):
    screen.fill(BACKGROUND_COLOR)

    pygame.draw.circle(screen, BOUNDARY_COLOR, CENTER, DRAW_RADIUS, 2)
    pygame.draw.line(
        screen,
        GRID_COLOR,
        (CENTER[0] - DRAW_RADIUS, CENTER[1]),
        (CENTER[0] + DRAW_RADIUS, CENTER[1]),
        1,
    )
    pygame.draw.line(
        screen,
        GRID_COLOR,
        (CENTER[0], CENTER[1] - DRAW_RADIUS),
        (CENTER[0], CENTER[1] + DRAW_RADIUS),
        1,
    )

    target_color = TARGET_ACTIVE_COLOR if mode in (2, 3) else TARGET_IDLE_COLOR
    draw_target_x = int(CENTER[0] + (robot_x / R_MAX) * DRAW_RADIUS)
    draw_target_y = int(CENTER[1] - (robot_y / R_MAX) * DRAW_RADIUS)
    pygame.draw.circle(screen, target_color, (draw_target_x, draw_target_y), 8)
    pygame.draw.circle(screen, (255, 255, 255), (draw_target_x, draw_target_y), 15, 1)

    if mode == 3:
        mode_text = "KEYBOARD INPUT"
        mode_color = (255, 165, 0)
        target_display = f"X: {robot_x:.2f} | Y: {robot_y:.2f} | Z: {robot_z:.2f}"
    elif mode == 2:
        mode_text = "MANUAL MOUSE CONTROL"
        mode_color = SUCCESS_COLOR
        target_display = f"X: {robot_x:.2f} | Y: {robot_y:.2f} | Z: {robot_z:.2f}"
    elif mode == 1:
        mode_text = "AUTO MODE"
        mode_color = (255, 100, 100)
        target_display = "Automated control mode selected."
    else:
        mode_text = "HOME MODE"
        mode_color = (255, 100, 100)
        target_display = "Return-to-home command selected."

    txt_device = font.render(f"HOST: {DEVICE_NAME}", True, INFO_COLOR)
    txt_mode = font.render(f"MODE: {mode_text} (Press 0,1,2,3 to switch)", True, mode_color)
    txt_target = font.render(f"TARGET: {target_display}", True, TARGET_ACTIVE_COLOR)
    txt_topics = font.render(
        f"ROS TOPICS: {MODE_TOPIC} | {TARGET_TOPIC} | {COMMAND_TOPIC}",
        True,
        INFO_COLOR,
    )

    screen.blit(txt_device, (20, 20))
    screen.blit(txt_mode, (20, 50))
    screen.blit(txt_target, (20, 80))
    screen.blit(txt_topics, (20, 110))

    if mode == 3:
        if is_typing:
            pygame.draw.rect(screen, (50, 50, 50), (20, HEIGHT - 100, WIDTH - 40, 40))
            pygame.draw.rect(screen, SUCCESS_COLOR, (20, HEIGHT - 100, WIDTH - 40, 40), 2)
            cursor = "|" if pygame.time.get_ticks() % 1000 < 500 else ""
            txt_input = font_large.render(f"INPUT X Y Z: {input_text}{cursor}", True, (255, 255, 0))
            screen.blit(txt_input, (30, HEIGHT - 95))
            txt_guide = font.render("Press ENTER to confirm, ESC to cancel.", True, INFO_COLOR)
            screen.blit(txt_guide, (20, HEIGHT - 50))
        else:
            txt_guide = font.render(
                "Press ENTER for X Y Z input. Use the mouse wheel to change Z.",
                True,
                INFO_COLOR,
            )
            screen.blit(txt_guide, (20, HEIGHT - 50))
    elif mode == 2:
        txt_guide = font.render("Move mouse for X/Y. Scroll wheel for Z.", True, INFO_COLOR)
        screen.blit(txt_guide, (20, HEIGHT - 50))

    local_color = ERROR_COLOR if local_status.startswith("Error") else SUCCESS_COLOR
    ros_color = ERROR_COLOR if "error" in ros_status.lower() else INFO_COLOR
    txt_local_status = font.render(local_status, True, local_color)
    txt_ros_status = font.render(f"Controller: {ros_status}", True, ros_color)
    screen.blit(txt_local_status, (20, HEIGHT - 155))
    screen.blit(txt_ros_status, (20, HEIGHT - 130))

    pygame.display.flip()


def main():
    rclpy.init()
    ros_bridge = DeltaControlBridge()

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"Delta Robot Control Panel - {DEVICE_NAME}")
    clock = pygame.time.Clock()

    try:
        font = pygame.font.SysFont("tahoma, segoeui", 20)
        font_large = pygame.font.SysFont("tahoma, segoeui", 24, bold=True)
    except Exception:
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_large = pygame.font.Font(pygame.font.get_default_font(), 24)

    mode = 0
    robot_x, robot_y, robot_z = 0.0, 0.0, DEFAULT_Z
    is_typing = False
    input_text = ""
    local_status = "ROS 2 bridge ready. Publishing controller state."

    running = True
    while running:
        rclpy.spin_once(ros_bridge, timeout_sec=0.0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if mode == 3 and is_typing:
                    if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        try:
                            parts = input_text.strip().split()
                            if len(parts) < 2:
                                local_status = "Error: Enter at least X and Y."
                            else:
                                new_x = float(parts[0])
                                new_y = float(parts[1])
                                new_z = float(parts[2]) if len(parts) >= 3 else robot_z
                                if within_workspace(new_x, new_y, new_z):
                                    robot_x, robot_y, robot_z = new_x, new_y, new_z
                                    input_text = ""
                                    is_typing = False
                                    local_status = "Coordinates accepted and published."
                                else:
                                    local_status = f"Error: Out of range (R<={R_MAX}, {Z_MIN}<=Z<={Z_MAX})."
                        except ValueError:
                            local_status = "Error: Only numeric values are allowed."
                    elif event.key == pygame.K_BACKSPACE:
                        input_text = input_text[:-1]
                        local_status = "Editing input."
                    elif event.key == pygame.K_ESCAPE:
                        is_typing = False
                        input_text = ""
                        local_status = "Input cancelled."
                    elif event.unicode in "0123456789- .":
                        input_text += event.unicode
                        local_status = "Editing input."
                else:
                    if event.key in (pygame.K_0, pygame.K_KP0):
                        mode = 0
                        is_typing = False
                        local_status = "Home mode selected."
                    elif event.key in (pygame.K_1, pygame.K_KP1):
                        mode = 1
                        is_typing = False
                        local_status = "Auto mode selected."
                    elif event.key in (pygame.K_2, pygame.K_KP2):
                        mode = 2
                        is_typing = False
                        local_status = "Manual mouse control enabled."
                    elif event.key in (pygame.K_3, pygame.K_KP3):
                        mode = 3
                        is_typing = False
                        local_status = "Keyboard coordinate mode enabled."
                    elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER) and mode == 3:
                        is_typing = True
                        input_text = ""
                        local_status = "Enter X Y Z coordinates."

            elif event.type == pygame.MOUSEWHEEL and mode in (2, 3):
                robot_z = clamp(robot_z + event.y * Z_STEP, Z_MIN, Z_MAX)

        if mode == 2:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            robot_x, robot_y = map_mouse_to_robot(mouse_x, mouse_y)

        ros_bridge.publish_state(mode, robot_x, robot_y, robot_z)
        draw_ui(
            screen,
            font,
            font_large,
            mode,
            robot_x,
            robot_y,
            robot_z,
            is_typing,
            input_text,
            local_status,
            ros_bridge.latest_status,
        )
        clock.tick(FRAME_RATE)

    ros_bridge.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == "__main__":
    main()
