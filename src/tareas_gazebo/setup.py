import os
from glob import glob
from setuptools import setup

package_name = 'tareas_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='opyntorr',
    maintainer_email='a00344869@tec.mx',
    description='KF continuo + control y mapa de ocupacion probabilistico en Gazebo (JetAuto).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'kf_control = tareas_gazebo.kf_control_node:main',
            'gridmap = tareas_gazebo.gridmap_node:main',
        ],
    },
)
