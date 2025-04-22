from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_localisation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'numpy', 'transforms3d'],
    zip_safe=True,
    maintainer='Mario Martinez',
    maintainer_email='mario.mtz@manchester-robotics.com',
    description='Localization, control and navigation package for the Puzzlebot mobile robot using ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_odometry = puzzlebot_localisation.puzzlebot_odometry:main',
            'puzzlebot_pose_controller = puzzlebot_localisation.puzzlebot_pose_controller:main',
            'puzzlebot_path_generator = puzzlebot_localisation.puzzlebot_path_generator:main',
        ],
    },
)
