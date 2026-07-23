import os
from glob import glob

from setuptools import setup


package_name = 'tardigrade_esp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ESP32 thruster bridge for Tardigrade AUV.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp_bridge = tardigrade_esp.esp_bridge:main',
            'fake_esp_state = tardigrade_esp.fake_esp_state:main',
            'thruster_mixer = tardigrade_esp.thruster_mixer:main',
            'esp_thruster_bridge = tardigrade_esp.esp_thruster_bridge:main',
            'esp_thruster_test = tardigrade_esp.esp_thruster_test:main',
            'depth_attitude_controller = tardigrade_esp.depth_attitude_controller:main',
        ],
    },
)
