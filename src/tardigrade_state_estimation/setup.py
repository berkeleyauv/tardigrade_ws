from setuptools import setup

package_name = 'tardigrade_state_estimation'

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
        # Each console_script becomes a `ros2 run tardigrade_state_estimation ...`
        # executable after colcon build.
        'console_scripts': [
            'vectornav_odometry = tardigrade_state_estimation.vectornav_odometry:main',
            'zed_odometry = tardigrade_state_estimation.zed_odometry:main',
            'zed_vectornav_odometry = tardigrade_state_estimation.zed_vectornav_odometry:main',
        ],
    },
)
