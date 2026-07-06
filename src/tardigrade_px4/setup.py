from setuptools import setup

package_name = 'tardigrade_px4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        # Each console_script becomes a `ros2 run tardigrade_px4 ...`
        # executable after colcon build.
        'console_scripts': [
            'pixhawk_interface = tardigrade_px4.pixhawk_interface:main',
            'mock_px4_status = tardigrade_px4.mock_px4_status:main',
            'odometry_to_px4 = tardigrade_px4.odometry_to_px4:main',
            'mavlink_odometry_to_px4 = tardigrade_px4.mavlink_odometry_to_px4:main',
            'mavlink_pixhawk_interface = tardigrade_px4.mavlink_pixhawk_interface:main',
            'keyboard_cmd_vel = tardigrade_px4.keyboard_cmd_vel:main',
        ],
    },
)
