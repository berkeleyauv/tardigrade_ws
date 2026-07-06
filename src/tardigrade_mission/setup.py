from setuptools import setup

package_name = 'tardigrade_mission'

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
    maintainer='Berkeley AUV',
    maintainer_email='devajgupta@berkeley.edu',
    description='Mission state machines for the Tardigrade AUV.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gate_mission = tardigrade_mission.gate_mission:main',
        ],
    },
)
