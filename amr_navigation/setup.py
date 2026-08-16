import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scinolimits',
    maintainer_email='prazwaldutta7@gmail.com',
    description='Waypoint navigation for the autonomous mobile robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_server = amr_navigation.waypoint_server:main',
            'patrol_server = amr_navigation.patrol_server:main',
            'obstacle_avoidance = amr_navigation.obstacle_avoidance:main',
            'cmd_arbitrator = amr_navigation.cmd_arbitrator:main',
            'tf_pose_monitor = amr_navigation.tf_pose_monitor:main',
        ],
    },
)
