"""aruco_marker_viz_cpp marker_viz_node 실행 런치."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_marker_viz_cpp')
    default_params = os.path.join(pkg_share, 'config', 'marker_viz.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw/compressed')
    calib_arg = DeclareLaunchArgument('calib_file', default_value='')

    node = Node(
        package='aruco_marker_viz_cpp',
        executable='marker_viz_node',
        name='aruco_marker_viz',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'calib_file': LaunchConfiguration('calib_file'),
            },
        ],
    )
    return LaunchDescription([params_arg, image_topic_arg, calib_arg, node])
