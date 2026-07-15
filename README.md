# aruco_marker_viz_cpp

ArUco 마커 검출·표시 및 마커 기반 초기위치 추정 (C++ / ROS2 Humble).

Python 패키지 `aruco_marker_viz`를 C++(`rclcpp` + OpenCV)로 포팅한 버전입니다.

## 구성

- `src/marker_viz_node.cpp` — 이미지 구독, ArUco 마커 검출·표시, 마커 거리 및 맵 좌표 계산
- `src/init_pose_from_marker_node.cpp` — 검출된 마커로부터 초기위치(pose) 추정
- `src/marker_approach_node.cpp` — 마커를 인식해 **정면 10cm 앞까지 자동 접근**(`/cmd_vel` 발행)
- `include/aruco_marker_viz_cpp/aruco_common.hpp` — 공용 헬퍼
- `config/` — `marker_viz.yaml`, `marker_map.yaml`, `init_pose.yaml`, `marker_approach.yaml`
- `launch/` — `marker_viz.launch.py`, `init_pose_from_marker.launch.py`, `marker_approach.launch.py`

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
ros2 launch aruco_marker_viz_cpp marker_approach.launch.py
```

## 서비스 호출 (Service Calls)

### `init_pose_from_marker_node`

검출된 타깃 마커로 로봇 초기위치를 계산해 `/initialpose` 로 1회 발행.

```bash
# 마커를 카메라에 보이게 한 뒤 호출
ros2 service call /init_pose_from_marker/estimate_init_pose std_srvs/srv/Trigger
```

### `marker_approach_node`

마커 정면 10cm 앞까지 자동 접근. **안전을 위해 실행 직후에는 대기(IDLE)** 하며,
아래 `start` 서비스를 호출해야 이동을 시작합니다(`auto_start: true` 로 바꾸면 즉시 시작).

```bash
# 접근 시작 (IDLE → 정렬(ALIGN) → 전진(APPROACH) → 도착(ARRIVED))
ros2 service call /marker_approach/start std_srvs/srv/Trigger

# 즉시 정지 (cmd_vel=0, IDLE 로 전환) — 비상 정지용
ros2 service call /marker_approach/stop std_srvs/srv/Trigger

# 발행되는 속도명령 확인
ros2 topic echo /cmd_vel
```

동작 개요:

1. **ALIGN** — 전진 없이 횡이동(`linear.y`)·회전(`angular.z`)으로 마커를 화면 중앙에
   맞추고 마커 정면(z축)으로 정렬. (허용오차: 중앙 `lateral_tolerance`, z축 `yaw_tolerance_deg`)
2. **APPROACH** — 정면을 유지하며 전진(`linear.x`), 카메라 z거리가 `final_distance`(기본 0.10m)가
   되면 정지·`ARRIVED`.
3. 근접에서 마커가 화면을 벗어나 소실되면 `openloop_final_approach` 로 마지막 구간을
   무피드백 저속 직진으로 마무리(odom `/aft_mapped_to_init` 로 이동거리 측정).
4. 마커가 `max_pose_age_sec` 이상 안 보이면 즉시 정지(안전).

> 최대 선속도는 `config/marker_approach.yaml` 의 `max_linear_speed`(기본 0.3 m/s)로 제한하며,
> 게인·허용오차 등 제어 파라미터는 같은 파일에 주석과 함께 정리되어 있습니다.
> 실제 로봇 적용 전, 낮은 속도에서 `/cmd_vel` 의 `y`·`angular.z` 부호가 로봇 규약과
> 맞는지 먼저 확인하세요.

런치 인자 예:

```bash
ros2 launch aruco_marker_viz_cpp marker_approach.launch.py \
  target_marker_id:=1 cmd_vel_topic:=/cmd_vel image_topic:=/camera/image_raw/compressed
```
