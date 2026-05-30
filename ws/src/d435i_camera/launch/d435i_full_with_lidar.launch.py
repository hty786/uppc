import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )
    d435i_share = get_package_share_directory('d435i_camera')

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
            'enable_sync': 'false',
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

    detection = Node(
        package='d435i_camera',
        executable='blue_detection_node.py',
        name='blue_detection_node',
        output='screen',
        parameters=[{
            'image_topic': '/d435i/d435i/color/image_raw',
            'hue_low': 100,
            'hue_high': 130,
            'sat_low': 80,
            'sat_high': 255,
            'val_low': 80,
            'val_high': 255,
            'min_area': 500,
            'min_aspect_ratio': 2.0,
        }],
    )

    mid360_config = os.path.join(d435i_share, 'config', 'MID360_config.json')
    mid360 = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': mid360_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }]
    )

    rviz_config = os.path.join(d435i_share, 'rviz', 'd435i.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config]
    )

    return LaunchDescription([camera, detection, mid360, rviz])
