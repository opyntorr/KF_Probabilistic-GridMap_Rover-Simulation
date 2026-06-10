import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tello_control_pos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_controller = tello_control_pos.controller:main',
            'pose_fuser = tello_control_pos.pose_fuser:main',
            'pose_fuser_optitrack = tello_control_pos.pose_fuser_optitrack:main',
            'plotter = tello_control_pos.plotter:main',
            'optitrack_simulator = tello_control_pos.optitrack_simulator:main',
            'drift_simulator = tello_control_pos.drift_simulator:main',
            'tello_joy_teleop = tello_control_pos.tello_joy_teleop:main',
            'camera_info_publisher = tello_control_pos.camera_info_publisher:main'
        ],
    },
)
