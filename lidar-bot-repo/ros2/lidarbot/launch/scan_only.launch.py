"""
Step 1: lidar + frames + Foxglove, and nothing else.

Usage:
    ros2 launch lidarbot scan_only.launch.py
    ros2 launch lidarbot scan_only.launch.py lidar_port:=/dev/ttyUSB1
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lidarbot')
    with open(os.path.join(pkg, 'urdf', 'lidarbot.urdf')) as f:
        robot_desc = f.read()
    port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('bridge', default_value='foxglove'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='fake_odom',
             arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']),

        Node(package='lidarbot', executable='ldlidar_node',
             name='ldlidar', output='screen',
             parameters=[{
                 'port': port,
                 'baud': 230400,
                 'frame_id': 'base_laser',
                 'topic': 'scan',
                 'bins': 450,
                 'invert': True,
                 'angle_offset': 0.0,
                 'range_max': 12.0,
             }]),

        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge', output='screen',
             condition=IfCondition(PythonExpression(
                 ["'", LaunchConfiguration('bridge'), "' == 'foxglove'"])),
             parameters=[{'port': 8765, 'address': '0.0.0.0'}]),
    ])
