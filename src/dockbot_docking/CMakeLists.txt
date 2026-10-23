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
project(dockbot_docking)

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
find_package(geometry_msgs REQUIRED)
find_package(dockbot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(control_law STATIC src/control_law.cpp)
target_link_libraries(control_law PUBLIC Eigen3::Eigen)
target_include_directories(control_law PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(state_machine STATIC src/state_machine.cpp)
target_link_libraries(state_machine PUBLIC control_law)
target_include_directories(state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_library(docking_controller_component SHARED src/docking_controller.cpp)
target_link_libraries(docking_controller_component PUBLIC state_machine)
ament_target_dependencies(docking_controller_component PUBLIC
  rclcpp rclcpp_action rclcpp_components geometry_msgs dockbot_interfaces)
set_target_properties(control_law state_machine PROPERTIES POSITION_INDEPENDENT_CODE ON)
rclcpp_components_register_nodes(docking_controller_component "dockbot_docking::DockingController")

add_executable(docking_controller src/docking_controller_main.cpp)
target_link_libraries(docking_controller docking_controller_component)

install(TARGETS docking_controller_component
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS docking_controller DESTINATION lib/${PROJECT_NAME})

install(TARGETS control_law state_machine EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(test_control_law test/test_control_law.cpp)
  target_include_directories(test_control_law PRIVATE test)
  target_link_libraries(test_control_law control_law)

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PRIVATE test)
  target_link_libraries(test_state_machine state_machine)

  ament_add_gtest(test_controller_node test/test_controller_node.cpp)
  target_link_libraries(test_controller_node docking_controller_component)
  ament_target_dependencies(test_controller_node rclcpp rclcpp_action dockbot_interfaces
    rosgraph_msgs)

  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp)
  target_include_directories(test_closed_loop PRIVATE test)
  target_link_libraries(test_closed_loop state_machine)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
