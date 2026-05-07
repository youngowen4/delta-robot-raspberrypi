from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_backend = LaunchConfiguration("servo_backend")
    arm1_pin = LaunchConfiguration("arm1_pin")
    arm2_pin = LaunchConfiguration("arm2_pin")
    arm3_pin = LaunchConfiguration("arm3_pin")

    return LaunchDescription(
        [
            DeclareLaunchArgument("servo_backend", default_value="dry-run"),
            DeclareLaunchArgument("arm1_pin", default_value="12"),
            DeclareLaunchArgument("arm2_pin", default_value="13"),
            DeclareLaunchArgument("arm3_pin", default_value="18"),
            Node(
                package="delta_robot_ros",
                executable="rpi_robot_node",
                name="delta_raspberry_pi_robot",
                output="screen",
                parameters=[],
                additional_env={
                    "DELTA_SERVO_BACKEND": servo_backend,
                    "DELTA_SERVO_PIN_ARM1": arm1_pin,
                    "DELTA_SERVO_PIN_ARM2": arm2_pin,
                    "DELTA_SERVO_PIN_ARM3": arm3_pin,
                },
            ),
        ]
    )
