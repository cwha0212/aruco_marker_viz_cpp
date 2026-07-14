"""aruco_marker_viz_cpp init_pose_from_marker_node 실행 런치.

서비스 호출:
    ros2 service call /init_pose_from_marker/estimate_init_pose std_srvs/srv/Trigger
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
    default_params = os.path.join(pkg_share, 'config', 'init_pose.yaml')
    default_marker_map = os.path.join(pkg_share, 'config', 'marker_map.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    marker_map_arg = DeclareLaunchArgument(
        'marker_map_file', default_value=default_marker_map,
        description='마커 map pose 파라미터 파일(ROS2 params)')
    calib_arg = DeclareLaunchArgument('calib_file', default_value='')
    target_arg = DeclareLaunchArgument('target_marker_id', default_value='1')

    node = Node(
        package='aruco_marker_viz_cpp',
        executable='init_pose_from_marker_node',
        name='init_pose_from_marker',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            LaunchConfiguration('marker_map_file'),  # 마커 map 을 params 파일로 병합
            {
                'calib_file': LaunchConfiguration('calib_file'),
                'target_marker_id': ParameterValue(
                    LaunchConfiguration('target_marker_id'), value_type=int),
            },
        ],
    )
    return LaunchDescription([params_arg, marker_map_arg, calib_arg, target_arg, node])
