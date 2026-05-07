from setuptools import find_packages, setup


package_name = "delta_robot_ros"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/delta_robot.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="young",
    maintainer_email="young@example.com",
    description="ROS 2 Python nodes for an Orin-controlled Delta robot with Raspberry Pi actuation.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orin_controller = delta_robot_ros.orin_controller:main",
            "rpi_robot_node = delta_robot_ros.rpi_robot_node:main",
        ],
    },
)
