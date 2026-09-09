import os
from glob import glob

from setuptools import setup


package_name = 'tardigrade_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Berkeley AUV',
    maintainer_email='software@berkeleyauv.org',
    description='Canonical vehicle configuration and export tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'export_vehicle_config = '
            'tardigrade_description.export_vehicle_config:main',
            'validate_vehicle_config = '
            'tardigrade_description.export_vehicle_config:validate_main',
        ],
    },
)
