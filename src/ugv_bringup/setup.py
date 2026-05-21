import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ugv_bringup"

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
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "systemd"),
            glob(os.path.join("systemd", "*.service")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UGV Team",
    maintainer_email="robot@delivery.local",
    description="Master bringup for UGV Rover delivery robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
