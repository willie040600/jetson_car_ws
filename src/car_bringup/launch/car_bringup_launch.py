import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_params = os.path.join(
        get_package_share_directory('ydlidar_scan_cpp'), 'params', 'ydlidar.yaml')

    motor_port = LaunchConfiguration('motor_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        # ttyUSB 編號會隨插拔順序翻轉，接反時兩顆都是「開得起來但不回話」
        DeclareLaunchArgument('motor_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),

        Node(
            package='zlac706_driver_cpp',
            executable='zlac706_diffdrive_node',
            name='zlac706_diffdrive_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'port': motor_port}],
        ),
        Node(
            package='ydlidar_scan_cpp',
            executable='ydlidar_scan_node',
            name='ydlidar_scan_node',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_params, {'port': lidar_port}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_frame',
            arguments=['0.0', '0.0', '0.107', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],
        ),
    ])
