from setuptools import setup

package_name = 'tardigrade'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.controls', f'{package_name}.controls.controllers'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'imageio'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Tardigrade robot controller and URDF package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orientation_control = tardigrade.controls.orientation_control:main',
            'velocity_transformer = tardigrade.controls.velocity_transformer:main',
        ],
    },
)