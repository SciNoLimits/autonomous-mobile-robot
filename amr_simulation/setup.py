from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'amr_simulation'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        # World
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')
        ),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        # Environment model files
        (
            os.path.join(
                'share',
                package_name,
                'worlds',
                'models',
                'amr_environment'
            ),
            [
                'worlds/models/amr_environment/model.config',
                'worlds/models/amr_environment/model.sdf',
            ]
        ),

        # Environment mesh files
        (
            os.path.join(
                'share',
                package_name,
                'worlds',
                'models',
                'amr_environment',
                'meshes'
            ),
            [
                'worlds/models/amr_environment/meshes/environment.obj',
                'worlds/models/amr_environment/meshes/environment.mtl',
            ]
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='scinolimits',
    maintainer_email='prazwaldutta7@gmail.com',

    description='AMR Gazebo simulation environment',

    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [],
    },
)