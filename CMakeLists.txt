cmake_minimum_required(VERSION 3.8)
project(aruco_marker_viz_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPS
  rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs
  cv_bridge tf2 tf2_geometry_msgs OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(marker_viz_node src/marker_viz_node.cpp)
ament_target_dependencies(marker_viz_node ${DEPS})
target_link_libraries(marker_viz_node ${OpenCV_LIBS} yaml-cpp)

add_executable(init_pose_from_marker_node src/init_pose_from_marker_node.cpp)
ament_target_dependencies(init_pose_from_marker_node ${DEPS})
target_link_libraries(init_pose_from_marker_node ${OpenCV_LIBS} yaml-cpp)

install(TARGETS marker_viz_node init_pose_from_marker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
