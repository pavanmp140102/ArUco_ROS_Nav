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

"""Launch Gazebo, spawn dockbot, and bridge its topics to ROS: the simulated half of docking."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nvidia_usable():
    """
    Return True when this host has an NVIDIA GPU with a loaded driver and is not a container.

    Lets the default choose the NVIDIA GPU on a workstation and fall back to the portable
    setting everywhere else. Inside a container nvidia-smi can succeed without a usable
    OpenGL stack, so containers keep the portable setting; the NVIDIA service of the Docker
    setup passes gpu:=nvidia explicitly.
    """
    if os.path.exists('/.dockerenv'):
        return False
    try:
        return subprocess.run(
            ['nvidia-smi', '-L'], capture_output=True, timeout=3).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def _gz_args(context):
    """
    Build the Gazebo command line and GPU environment from the launch arguments.

    Chooses server-only headless rendering or the GUI, and which GPU renders, so the
    camera frames the docking pipeline depends on can be produced on any machine.
    """
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    gpu = LaunchConfiguration('gpu').perform(context)
    if gpu == 'auto':
        gpu = 'nvidia' if _nvidia_usable() else 'intel'

    world_file = os.path.join(
        get_package_share_directory('dockbot_gazebo'), 'worlds', world + '.sdf')
    args = '-r -v2 ' + world_file
    if headless:
        args = '-s --headless-rendering ' + args

    actions = []
    if gpu == 'nvidia':
        actions += [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        ]
    elif gpu == 'software':
        actions.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())
    return actions + [gz_sim]


def generate_launch_description():
    """Declare the arguments and assemble Gazebo, robot_state_publisher, spawner and bridges."""
    gazebo_share = get_package_share_directory('dockbot_gazebo')
    description_share = get_package_share_directory('dockbot_description')
    bridge_config = os.path.join(gazebo_share, 'config', 'bridge.yaml')
    xacro_file = os.path.join(description_share, 'urdf', 'dockbot.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' camera_rate:=', LaunchConfiguration('camera_rate'),
            ' camera_noise:=', LaunchConfiguration('camera_noise'),
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'dockbot', '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
            '-z', '0.01', '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen')

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera/image_raw'],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty',
                              description='World name in dockbot_gazebo/worlds (no extension)'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the server only, without the Gazebo GUI'),
        DeclareLaunchArgument('gpu', default_value='auto',
                              description='Renderer: auto (NVIDIA if usable, else intel), '
                                          'intel, nvidia, or software'),
        DeclareLaunchArgument('camera_rate', default_value='30',
                              description='Camera frame rate [Hz]'),
        DeclareLaunchArgument('camera_noise', default_value='0.0',
                              description='Camera Gaussian noise standard deviation; 0 is off'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        OpaqueFunction(function=_gz_args),
        robot_state_publisher,
        spawn,
        bridge,
        image_bridge,
    ])
