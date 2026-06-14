from glob import glob

from setuptools import find_packages, setup

package_name = 'ur_rtde_torque_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HAOYU LUO',
    maintainer_email='haoyu.luo@ensam.eu',
    description='Standalone, pluggable RTDE receive-only client that publishes '
                'UR joint torques (N.m) on /joint_torques as a standard '
                'sensor_msgs/JointState.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'torque_publisher = '
                'ur_rtde_torque_bridge.torque_publisher_node:main'
            ),
        ],
    },
)
