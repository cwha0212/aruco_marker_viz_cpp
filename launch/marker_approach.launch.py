"""aruco_marker_viz_cpp marker_approach_node 실행 런치.

접근 시작/정지 서비스:
    ros2 service call /marker_approach/start std_srvs/srv/Trigger
    ros2 service call /marker_approach/stop  std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_marker_viz_cpp')
    default_params = os.path.join(pkg_share, 'config', 'marker_approach.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw/compressed')
    calib_arg = DeclareLaunchArgument('calib_file', default_value='')
    target_arg = DeclareLaunchArgument('target_marker_id', default_value='1')
    cmd_vel_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')

    node = Node(
        package='aruco_marker_viz_cpp',
        executable='marker_approach_node',
        name='marker_approach',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'calib_file': LaunchConfiguration('calib_file'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'target_marker_id': ParameterValue(
                    LaunchConfiguration('target_marker_id'), value_type=int),
            },
        ],
    )
    return LaunchDescription(
        [params_arg, image_topic_arg, calib_arg, target_arg, cmd_vel_arg, node])
