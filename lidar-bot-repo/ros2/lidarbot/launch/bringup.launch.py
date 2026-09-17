"""
lidar-bot hardware bringup: description, base bridge, lidar, joystick teleop,
and the Foxglove bridge. No SLAM here - see slam.launch.py.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('lidarbot')
    urdf = os.path.join(pkg, 'urdf', 'lidarbot.urdf')
    with open(urdf) as f:
        robot_desc = f.read()

    use_joy = LaunchConfiguration('joy')
    use_foxglove = LaunchConfiguration('foxglove')
    lidar_port = LaunchConfiguration('lidar_port')
    gyro_scale = ParameterValue(LaunchConfiguration('gyro_scale'),
                               value_type=float)
    gyro_bias = ParameterValue(LaunchConfiguration('gyro_bias'),
                               value_type=float)
    I = lambda n: ParameterValue(LaunchConfiguration(n), value_type=int)
    F = lambda n: ParameterValue(LaunchConfiguration(n), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('gyro_scale', default_value='1.0'),
        DeclareLaunchArgument('gyro_bias', default_value='0.0'),
        DeclareLaunchArgument('gyro_invert', default_value='false'),
        DeclareLaunchArgument('joy_axis_lin', default_value='5'),
        DeclareLaunchArgument('joy_axis_ang', default_value='0'),
        DeclareLaunchArgument('joy_enable', default_value='4'),
        DeclareLaunchArgument('joy_turbo', default_value='5'),
        DeclareLaunchArgument('joy_scale_lin', default_value='0.25'),
        DeclareLaunchArgument('joy_scale_ang', default_value='1.2'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('cam_fps', default_value='10.0'),
        DeclareLaunchArgument('cam_quality', default_value='50'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}], output='screen'),

        Node(package='lidarbot', executable='base_bridge', name='base_bridge',
             output='screen',
             parameters=[{
                 'port': '/dev/ttyAMA0',
                 'baud': 115200,
                 'use_gyro_yaw': True,
                 'cmd_scale': 1.0,
                 'cmd_max': 0.35,
                 'gyro_scale': gyro_scale,
                 'gyro_bias': gyro_bias,
                 'gyro_invert': ParameterValue(
                     LaunchConfiguration('gyro_invert'), value_type=bool),
             }]),

        Node(package='lidarbot', executable='ldlidar_node',
             name='ldlidar', output='screen',
             parameters=[{
                 'port': lidar_port,
                 'baud': 230400,
                 'frame_id': 'base_laser',
                 'topic': 'scan',
                 'bins': 450,
                 'invert': True,
                 'angle_offset': 0.0,
                 'range_max': 12.0,
             }]),

        Node(package='lidarbot', executable='camera_node', name='camera',
             output='screen',
             condition=IfCondition(LaunchConfiguration('camera')),
             parameters=[{
                 'index': -1,            # probe: the metadata node opens but
                 'width': 640,           #   never reads, so prove it yields
                 'height': 480,
                 'fps': F('cam_fps'),
                 'quality': I('cam_quality'),
                 'frame_id': 'camera_link',
             }]),

        Node(package='joy', executable='joy_node', name='joy',
             condition=IfCondition(use_joy),
             parameters=[{'deadzone': 0.12, 'autorepeat_rate': 20.0}]),

        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop',
             condition=IfCondition(use_joy),
             parameters=[{
                 'axis_linear.x': I('joy_axis_lin'),
                 'scale_linear.x': F('joy_scale_lin'),
                 'scale_linear_turbo.x': F('joy_scale_lin'),
                 'axis_angular.yaw': I('joy_axis_ang'),
                 'scale_angular.yaw': F('joy_scale_ang'),
                 'enable_button': I('joy_enable'),
                 'enable_turbo_button': I('joy_turbo'),
                 'publish_stamped_twist': True,
             }]),

        Node(package='foxglove_bridge', executable='foxglove_bridge',
             condition=IfCondition(use_foxglove),
             parameters=[{'port': 8765, 'address': '0.0.0.0'}]),
    ])
