from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_merger',
            executable='pointcloud_merger_node',  # 你的点云合并节点
            name='pointcloud_merger',
        ),
        Node(
            package='pointcloud_merger',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
           
        ),
    ])
