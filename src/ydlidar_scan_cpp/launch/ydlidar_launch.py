import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ydlidar_scan_cpp'), 'params', 'ydlidar.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='ydlidar_scan_cpp',
            executable='ydlidar_scan_node',
            name='ydlidar_scan_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_frame',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame'],
        ),
    ])
