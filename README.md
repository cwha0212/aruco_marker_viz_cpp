# aruco_marker_viz_cpp

ArUco 마커 검출·표시 및 마커 기반 초기위치 추정 (C++ / ROS2 Humble).

Python 패키지 `aruco_marker_viz`를 C++(`rclcpp` + OpenCV)로 포팅한 버전입니다.

## 구성

- `src/marker_viz_node.cpp` — 이미지 구독, ArUco 마커 검출·표시, 마커 거리 및 맵 좌표 계산
- `src/init_pose_from_marker_node.cpp` — 검출된 마커로부터 초기위치(pose) 추정
- `include/aruco_marker_viz_cpp/aruco_common.hpp` — 공용 헬퍼
- `config/` — `marker_viz.yaml`, `marker_map.yaml`, `init_pose.yaml`
- `launch/` — `marker_viz.launch.py`, `init_pose_from_marker.launch.py`

## 의존성

`rclcpp`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs`, `cv_bridge`,
`tf2`, `tf2_geometry_msgs`, `yaml-cpp`, `libopencv-dev`

## 빌드

```bash
cd aruco_ws
colcon build --packages-select aruco_marker_viz_cpp
source install/setup.bash
```

## 실행

```bash
ros2 launch aruco_marker_viz_cpp marker_viz.launch.py
ros2 launch aruco_marker_viz_cpp init_pose_from_marker.launch.py
```
