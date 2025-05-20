import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 获取配置文件路径
    filter_config = os.path.join(
        get_package_share_directory('fishbot_description'),
        'config',
        'laser_filter_config.yaml'
    )
    
    # 声明LaserScan过滤节点
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[
            {'use_sim_time': use_sim_time},
            filter_config  # 使用外部配置文件
        ],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        laser_filter_node
    ])
