import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_navigation2_dir = get_package_share_directory('car_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 實體車沒有 /clock，use_sim_time 只要是 true，AMCL 與 costmap 的 TF 查詢就會全部逾時
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    nav2_param_path = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='實體車固定 false'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(car_navigation2_dir, 'maps', 'room.yaml'),
            description='要載入的地圖 yaml 完整路徑'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(car_navigation2_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 參數檔完整路徑'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='自動把 Nav2 lifecycle 節點帶到 active'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否同時開 RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                'autostart': autostart}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
