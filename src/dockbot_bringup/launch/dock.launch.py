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


"""Launch the whole docking demo: Gazebo, the robot, the detector, the controller, optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Assemble sim.launch.py, aruco_detector, docking_controller and optionally RViz.

    With marker_id of zero or more the controller starts docking at that marker as soon as it
    sees it; with the default of -1 it waits for a goal on the /dock action.
    """
    gazebo_share = get_package_share_directory('dockbot_gazebo')
    perception_share = get_package_share_directory('dockbot_perception')
    docking_share = get_package_share_directory('dockbot_docking')
    description_share = get_package_share_directory('dockbot_description')

    # Arguments that sim.launch.py already declares are passed straight through.
    passthrough = ['world', 'x', 'y', 'yaw', 'headless', 'gpu', 'camera_rate']

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'sim.launch.py')),
        launch_arguments={name: LaunchConfiguration(name) for name in passthrough}.items())

    detector = Node(
        package='dockbot_perception',
        executable='aruco_detector',
        name='aruco_detector',
        parameters=[os.path.join(perception_share, 'config', 'detector.yaml')],
        output='screen')

    controller = Node(
        package='dockbot_docking',
        executable='docking_controller',
        name='docking_controller',
        parameters=[
            os.path.join(docking_share, 'config', 'controller.yaml'),
            {'autostart_marker_id': ParameterValue(
                LaunchConfiguration('marker_id'), value_type=int)},
        ],
        output='screen')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(description_share, 'rviz', 'dockbot.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='docking_world',
                              description='World name in dockbot_gazebo/worlds (no extension)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server only, without its GUI'),
        DeclareLaunchArgument('gpu', default_value='intel',
                              description='Renderer: intel, nvidia, or software'),
        DeclareLaunchArgument('camera_rate', default_value='30',
                              description='Camera frame rate [Hz]'),
        DeclareLaunchArgument('marker_id', default_value='-1',
                              description='Dock to this marker ID at launch; -1 waits for /dock'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also start RViz with the dockbot configuration'),
        sim,
        detector,
        controller,
        rviz,
    ])
