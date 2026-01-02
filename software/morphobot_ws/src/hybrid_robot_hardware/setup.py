from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hybrid_robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eisan',
    maintainer_email='hnineisan547@gmail.com',
    description='Hardware interface for ST3215 servos',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_driver = hybrid_robot_hardware.motor_driver:main',
            'test_servos = hybrid_robot_hardware.test_servo_detection:main',
        ],
        'hardware_interface': [
            'ST3215HardwareInterface = hybrid_robot_hardware.st3215_hardware_interface:create_st3215_hardware_interface',
        ],
    },
)
