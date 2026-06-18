from glob import glob

from setuptools import find_packages, setup

package_name = 'digital_twin_on_unity_and_ros2'

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
    description='ROS 2 nodes bridging UR robot state to Unity for a digital '
                'twin (joint state and UR state bridges plus a supervisor '
                'manager).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'joint_state_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'joint_state_bridge_ros2unity:main'
            ),
            (
                'joint_torque_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'joint_torque_bridge_ros2unity:main'
            ),
            (
                'ur_state_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'ur_state_bridge_ros2unity:main'
            ),
            (
                'io_states_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'io_states_bridge_ros2unity:main'
            ),
            (
                'tool_data_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'tool_data_bridge_ros2unity:main'
            ),
            (
                'controller_status_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'controller_status_bridge_ros2unity:main'
            ),
            (
                'robot_info_bridge_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'robot_info_bridge_ros2unity:main'
            ),
            (
                'command_supervisor_ros2unity = '
                'digital_twin_on_unity_and_ros2.'
                'command_supervisor_ros2unity:main'
            ),
            'digital_twin_manager = manager.digital_twin_manager_node:main',
        ],
    },
)
