package_name = 'tardigrade_recording'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raymond Tong',
    maintainer_email='raymond.tong@berkeley.edu',
    description='Camera recording scripts for tardigrade_ws',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed_recorder = tardigrade_recording.zed_recorder:main'
        ],
    },
)