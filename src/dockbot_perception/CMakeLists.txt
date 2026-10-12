# Copyright 2026 Pavan Patil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(dockbot_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dockbot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)

add_library(pose_estimation STATIC src/pose_estimation.cpp)
target_include_directories(pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(pose_estimation SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pose_estimation PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)
set_target_properties(pose_estimation PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(aruco_detector_component SHARED src/aruco_detector.cpp)
target_include_directories(aruco_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(aruco_detector_component PUBLIC pose_estimation)
ament_target_dependencies(aruco_detector_component PUBLIC
  rclcpp rclcpp_components cv_bridge tf2_ros sensor_msgs geometry_msgs dockbot_interfaces)
rclcpp_components_register_nodes(aruco_detector_component "dockbot_perception::ArucoDetector")

add_executable(aruco_detector src/aruco_detector_main.cpp)
target_link_libraries(aruco_detector aruco_detector_component)

install(TARGETS aruco_detector_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS aruco_detector DESTINATION lib/${PROJECT_NAME})

install(TARGETS pose_estimation EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_estimation test/test_pose_estimation.cpp)
  target_include_directories(test_pose_estimation PRIVATE test)
  target_link_libraries(test_pose_estimation pose_estimation)

  ament_add_gtest(test_detector_node test/test_detector_node.cpp)
  target_include_directories(test_detector_node PRIVATE test)
  target_link_libraries(test_detector_node aruco_detector_component)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
