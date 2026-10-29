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

"""Hand-computed cases for scoring.py."""

import math

import os

import pytest

from scoring import detection_error, dock_score, marker_in_base, yaml_markers

MARKER = {'x': 3.0, 'y': 0.0, 'yaw': math.pi}


def test_marker_ahead():
    """A robot at the origin facing +x sees a marker 3 m ahead facing back at it."""
    x, y, yaw = marker_in_base((0.0, 0.0, 0.0), MARKER)
    assert abs(x - 3.0) < 1e-9 and abs(y) < 1e-9 and abs(abs(yaw) - math.pi) < 1e-9


def test_marker_robot_yawed():
    """Turning the robot 90 degrees left puts the marker on its right."""
    x, y, _ = marker_in_base((0.0, 0.0, math.pi / 2), MARKER)
    assert abs(x) < 1e-9 and abs(y + 3.0) < 1e-9


def test_detection_error():
    """Range error is the difference in distance; yaw error wraps across pi."""
    r, a = detection_error((2.0, 0.0, math.pi), (2.03, 0.0, -math.pi + 0.02))
    assert abs(r - 0.03) < 1e-9 and abs(a - 0.02) < 1e-9


def test_dock_score_ideal():
    """Base 0.25 m from the marker, facing it: front face is 0.10 m away."""
    d, lat, head = dock_score((2.75, 0.0, 0.0), MARKER)
    assert abs(d - 0.10) < 1e-9 and lat < 1e-9 and head < 1e-9


def test_dock_score_offset():
    """A 0.02 m sideways offset and a 3 degree heading error are reported as such."""
    d, lat, head = dock_score((2.75, 0.02, math.radians(3)), MARKER)
    assert abs(lat - 0.02 - 0.15 * math.sin(math.radians(3))) < 1e-9
    assert abs(head - math.radians(3)) < 1e-9


def test_yaml_markers_sources(tmp_path):
    """A scenario's own list is used for single_dock; docking_world reads markers.yaml only."""
    share = tmp_path
    os.makedirs(share / 'config')
    (share / 'config' / 'markers.yaml').write_text(
        'markers:\n  4: {x: 1.5, y: -2.5, yaw: 1.5708}\n')
    own = {'world': 'single_dock', 'markers': {0: {'x': 3.0, 'y': 0.0, 'yaw': 3.14159}}}
    assert list(yaml_markers(own, str(share))) == [0]
    assert list(yaml_markers({'world': 'docking_world'}, str(share))) == [4]
    with pytest.raises(ValueError):
        yaml_markers({'world': 'docking_world', 'markers': {0: {}}}, str(share))
    with pytest.raises(ValueError):  # a single_dock scenario must not fall back to another world
        yaml_markers({'world': 'single_dock'}, str(share))
