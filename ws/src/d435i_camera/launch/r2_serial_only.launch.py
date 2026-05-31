import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    mid360_config = os.path.join(
        get_package_share_directory('d435i_camera'),
        'config', 'MID360_config.json')

    mid360 = Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                  name='livox_lidar_publisher', output='screen',
                  parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                               'publish_freq': 10.0, 'output_data_type': 0,
                               'frame_id': 'livox_frame',
                               'user_config_path': mid360_config,
                               'cmdline_input_bd_code': 'livox0000000001'}])

    serial = Node(package='r2_controller', executable='serial_bridge.py',
                  name='serial_bridge', output='screen', on_exit=Shutdown(),
                  parameters=[{'port': 'auto', 'baudrate': 115200}])

    r2_main = Node(package='r2_controller', executable='r2_main.py',
                   name='r2_main', output='screen')

    fast_livo = Node(package='fast_livo', executable='fastlivo_mapping',
                     name='laserMapping', output='screen',
                     parameters=[os.path.join(get_package_share_directory('fast_livo'),
                                              'config', 'mid360.yaml'),
                                 os.path.join(get_package_share_directory('fast_livo'),
                                              'config', 'camera_mid360.yaml')])

    yaw_plotter = Node(package='r2_controller', executable='yaw_plotter.py',
                       name='yaw_plotter', output='screen')

    dashboard = Node(package='r2_controller', executable='r2_dashboard.py',
                       name='r2_dashboard', output='screen')

    return LaunchDescription([mid360, fast_livo, serial, r2_main,
                              yaw_plotter, dashboard])
