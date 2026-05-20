from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_navigation'

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
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.xml'))),
        (os.path.join('share', package_name, 'missions'),
            glob(os.path.join('missions', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UGV Team',
    maintainer_email='robot@delivery.local',
    description='Nav2 outdoor configuration and delivery mission executor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager  = ugv_navigation.mission_manager:main',
            'waypoint_converter = ugv_navigation.waypoint_converter:main',
        ],
    },
)
