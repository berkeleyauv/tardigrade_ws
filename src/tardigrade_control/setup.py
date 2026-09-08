import os
from glob import glob

from setuptools import setup


package_name = 'tardigrade_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Backend-agnostic Tardigrade control and allocation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_attitude_controller = '
            'tardigrade_control.depth_attitude_controller:main',
            'thruster_mixer = tardigrade_control.thruster_mixer:main',
            'velocity_wrench_controller = '
            'tardigrade_control.velocity_wrench_controller:main',
            'velocity_setpoint_mux = '
            'tardigrade_control.velocity_setpoint_mux:main',
            'pose_velocity_controller = '
            'tardigrade_control.pose_velocity_controller:main',
            'thruster_allocator = tardigrade_control.thruster_allocator:main',
            'thruster_actuator_mapper = '
            'tardigrade_control.thruster_actuator_mapper:main',
        ],
    },
)
