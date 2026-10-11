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

"""Static checks on the dockbot Xacro: it expands, parses, and has the documented geometry."""

import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest

XACRO = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'dockbot.urdf.xacro')


def _expand(*args):
    """Expand the Xacro to URDF text, so each test inspects what Gazebo would receive."""
    return subprocess.run(['xacro', XACRO, *args], check=True,
                          capture_output=True, text=True).stdout


def test_check_urdf(tmp_path):
    """The expanded model must parse with check_urdf, or robot_state_publisher would fail."""
    urdf = tmp_path / 'dockbot.urdf'
    urdf.write_text(_expand())
    result = subprocess.run(['check_urdf', str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_frames_and_geometry():
    """Frames and offsets that the detector and the front-face distance rely on must be right."""
    root = ET.fromstring(_expand())
    links = {link.get('name') for link in root.findall('link')}
    assert {'base_footprint', 'base_link', 'camera_link', 'camera_optical_frame'} <= links
    joints = {j.get('name'): j for j in root.findall('joint')}
    xyz = [float(v) for v in joints['camera_joint'].find('origin').get('xyz').split()]
    assert xyz == pytest.approx([-0.05, 0.0, 0.16])
    chassis = root.find("link[@name='base_link']/collision/geometry/box").get('size')
    assert [float(v) for v in chassis.split()][0] == pytest.approx(0.30)


def test_camera_arguments():
    """Camera rate and size are Xacro arguments so slow renderers can lower them."""
    root = ET.fromstring(_expand('camera_rate:=10', 'camera_width:=640', 'camera_height:=360'))
    sensor = root.find('.//sensor[@name="camera"]')
    assert sensor.find('update_rate').text == '10'
    assert sensor.find('camera/image/width').text == '640'
    assert sensor.find('gz_frame_id').text == 'camera_optical_frame'


def test_tools_available():
    """Guard so a missing xacro shows up as a clear failure instead of a confusing one."""
    assert shutil.which('xacro') and shutil.which('check_urdf')
