import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ugv_base"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pP][yY]")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UGV Team",
    maintainer_email="robot@delivery.local",
    description="Hardware abstraction layer for WaveShare UGV Rover",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_driver    = ugv_base.motor_driver:main",
            "battery_monitor = ugv_base.battery_monitor:main",
            "teleop_watchdog = ugv_base.teleop_watchdog:main",
        ],
    },
)
