import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 不用 slam_toolbox 內建的 online_async_launch.py：它預設 use_sim_time=true，
    # 且預設參數檔的 base_frame 是 base_footprint，兩者都會讓 odom 查詢失敗。
    slam_params = os.path.join(
        get_package_share_directory('zlac706_driver_cpp'),
        'config', 'mapper_params_online_async.yaml')

    if not os.path.isfile(slam_params):
        raise FileNotFoundError(f'找不到 SLAM 參數檔: {slam_params}')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            emulate_tty=True,
            parameters=[slam_params, {'use_sim_time': False}],
        ),
    ])
