from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )

    realsense_launch = IncludeLaunchDescription(
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
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'hole_filling_filter.enable': 'false',
        }.items(),
    )

    return LaunchDescription([realsense_launch])
