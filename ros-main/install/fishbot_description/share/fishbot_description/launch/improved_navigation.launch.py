import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')
    
    # 声明参数
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('fishbot_navigation2'), 'maps', 'room.yaml'),
        description='全局地图文件的路径')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('fishbot_navigation2'), 'config', 'nav2_params.yaml'),
        description='Nav2参数文件的路径')
    
    # 启动雷达滤波器
    include_laser_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fishbot_description'), 'launch', 'laser_filter.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 启动TF稳定器节点
    tf_stabilizer_node = Node(
        package='fishbot_description',
        executable='tf_stabilizer.py',
        name='tf_stabilizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 启动Nav2导航堆栈 - 直接修改参数指向过滤后的雷达话题
    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('fishbot_navigation2'), 'launch', 'navigation2.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'scan_topic': '/scan_filtered'  # 使用过滤后的激光扫描话题
        }.items()
    )
    
    return LaunchDescription([
        declare_map_file_cmd,
        declare_params_file_cmd,
        include_laser_filter,
        tf_stabilizer_node,
        include_nav2
    ])
