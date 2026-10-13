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

"""Launch the ArUco detector with its parameter file: camera image in, marker poses out."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start aruco_detector with config/detector.yaml and simulation time."""
    config = os.path.join(
        get_package_share_directory('dockbot_perception'), 'config', 'detector.yaml')
    return LaunchDescription([
        Node(
            package='dockbot_perception',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[config],
            output='screen'),
    ])
