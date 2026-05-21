from setuptools import setup
import os
from glob import glob

package_name = "ugv_vision"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@ugv",
    description="Vision obstacle-avoidance bridge for UGV Rover",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vision_server = ugv_vision.vision_server_node:main",
        ],
    },
)
