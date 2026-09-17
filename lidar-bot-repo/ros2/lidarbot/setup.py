from setuptools import setup
from glob import glob

package_name = 'lidarbot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaz',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_bridge = lidarbot.base_bridge:main',
            'ldlidar_node = lidarbot.ldlidar_node:main',
            'odom_check = lidarbot.odom_check:main',
            'pad_teleop = lidarbot.pad_teleop:main',
            'camera_node = lidarbot.camera_node:main',
            'explorer = lidarbot.explorer:main',
        ],
    },
)
