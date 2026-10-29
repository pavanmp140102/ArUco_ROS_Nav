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
project(dockbot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scoring test/test_scoring.py
    PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_launch_test(test/test_e10_detection.py
    TARGET test_e10_detection
    TIMEOUT 240
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  # One launch test per scenario, each with a fresh simulator. They are slow, so they carry the
  # "e2e" label: run only these with `ctest -L e2e`, or skip them with `ctest -LE e2e`. Every
  # test starts a full simulator on the same ROS domain, so none may run at the same time as
  # another (RUN_SERIAL), even under `ctest -j`.
  foreach(scenario
      e01_straight e02_side_offset e03_facing_away e04_all_ids e05_two_in_view
      e06_missing_marker e07_occluded e08_cancel e09_repeatability e11_camera_noise
      e12_two_goals)
    add_launch_test(test/test_e2e.py
      TARGET test_${scenario}
      TIMEOUT 1800
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test SCENARIO=${scenario})
    set_tests_properties(test_${scenario} PROPERTIES LABELS e2e RUN_SERIAL TRUE)
  endforeach()
  set_tests_properties(test_e10_detection PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
