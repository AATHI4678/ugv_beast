from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'phone_sensor_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pP][yY]'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UGV Team',
    maintainer_email='robot@delivery.local',
    description='WebSocket bridge: phone GPS + IMU to ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge     = phone_sensor_bridge.bridge:main',
            'mock_phone = phone_sensor_bridge.mock_phone:main',
        ],
    },
)
