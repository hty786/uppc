from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )

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
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
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

    return LaunchDescription([camera, detection])
