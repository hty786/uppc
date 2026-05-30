from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                # 左边是节点输入，右边是 FAST-LIVO2 发布的点云话题
                ('cloud_in', '/cloud_registered'), 
                # 左边是节点输出，右边是 Nav2 要听的话题
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link', # 切片基于哪个坐标系
                'transform_tolerance': 0.01,
                'min_height': 0.1,  # 这里的切片是用于“实时避障”和“实时定位”的
                'max_height': 1.0,
                'angle_min': -3.14159, 
                'angle_max': 3.14159, 
                'angle_increment': 0.0087, 
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True
            }]
        )
    ])