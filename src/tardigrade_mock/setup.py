import os
from glob import glob

from setuptools import setup


package_name = 'tardigrade_mock'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Development-only ROS interface mocks for Tardigrade.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_cameras = tardigrade_mock.mock_cameras:main',
            'mock_controller = tardigrade_mock.mock_controller:main',
            'mock_esp_bridge = tardigrade_mock.mock_esp_bridge:main',
            'mock_robot_state = tardigrade_mock.mock_robot_state:main',
        ],
    },
)
