from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'tardigrade'

setup(
    name=package_name,
    version='0.1.0',
    packages=['controls', 'tasks'],
    package_dir={'':'.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'imageio'],
    zip_safe=True,
    maintainer='kabilan',
    maintainer_email='kabilanvaikunthan@berkeley.edu',
    description='Tardigrade robot controller and URDF package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orientation_control = controls.orientation_control:main',
            'velocity_transformer = controls.velocity_transformer:main',
            'gate_approach = tasks.gate_approach:main',
        ],
    },
)
