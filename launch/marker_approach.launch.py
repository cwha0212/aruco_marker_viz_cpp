"""aruco_marker_viz_cpp marker_approach_node 실행 런치.

파라미터는 config/marker_approach.yaml 을 단일 소스로 사용한다(런치가 덮어쓰지 않음).
target_marker_id 등은 config 파일에서 수정하면 그대로 반영된다.
다른 파일을 쓰려면: ros2 launch ... params_file:=/path/to.yaml

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


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_marker_viz_cpp')
    default_params = os.path.join(pkg_share, 'config', 'marker_approach.yaml')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    node = Node(
        package='aruco_marker_viz_cpp',
        executable='marker_approach_node',
        name='marker_approach',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )
    return LaunchDescription([params_arg, node])
