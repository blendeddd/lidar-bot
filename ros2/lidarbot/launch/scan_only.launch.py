"""Step 1: lidar + frames + Foxglove, and nothing else.

No odometry, no SLAM, no motors. The point is to prove three things in one go:
the lidar driver runs, the TF frames line up with the scan's frame_id, and
Foxglove on the laptop can reach the Pi. If this looks right, everything after
it is incremental.

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
        # 'foxglove' needs ros-<distro>-foxglove-bridge installed;
        # 'none' lets you run any bridge yourself instead of this
        # launch file dying because the package is absent
        DeclareLaunchArgument('bridge', default_value='foxglove'),

        # publishes base_footprint -> base_link -> base_laser, so the scan has
        # somewhere to live even with no odometry running
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}], output='screen'),

        # odom -> base_footprint is normally the bridge's job; park the robot at
        # the origin so Foxglove has a complete tree for this test only
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
