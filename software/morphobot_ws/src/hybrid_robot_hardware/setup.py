from setuptools import find_packages, setup

package_name = 'hybrid_robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/controllers.yaml']),
        ('share/' + package_name + '/launch', ['launch/robot_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eisan',
    maintainer_email='hnineisan547@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver = hybrid_robot_hardware.motor_driver:main'
        ],
    },
)
