import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py')

    mid360_config = os.path.join(
        get_package_share_directory('d435i_camera'),
        'config', 'MID360_config.json')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'camera_name': 'd435i',
            'camera_namespace': 'd435i',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_motion': 'false',
            'enable_sync': 'true',
            'initial_reset': 'true',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'colorizer.enable': 'false',
            'decimation_filter.enable': 'false',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'hole_filling_filter.enable': 'true',
        }.items(),
    )

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

    kfs = Node(package='d435i_camera', executable='kfs_detector.py',
               name='kfs_detector', output='screen')

    r2_main = Node(package='r2_controller', executable='r2_main.py',
                   name='r2_main', output='screen')

    fast_livo = Node(package='fast_livo', executable='fastlivo_mapping',
                     name='laserMapping', output='screen',
                     parameters=[os.path.join(get_package_share_directory('fast_livo'),
                                              'config', 'mid360.yaml'),
                                 os.path.join(get_package_share_directory('fast_livo'),
                                              'config', 'camera_mid360.yaml')])

    pid_plotter = Node(package='r2_controller', executable='pid_plotter.py',
                       name='pid_plotter', output='screen')

    nav_plotter = Node(package='r2_controller', executable='nav_plotter.py',
                       name='nav_plotter', output='screen')

    return LaunchDescription([camera, mid360, fast_livo, serial, kfs, r2_main,
                              pid_plotter, nav_plotter])
