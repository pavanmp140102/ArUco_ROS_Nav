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

"""Scoring helpers for the end-to-end tests: ground-truth geometry, not a ROS node."""

import math


def yaw_of(q):
    """Return the yaw of quaternion (x, y, z, w), used to compare robot and marker headings."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    """Wrap an angle to [-pi, pi] so heading errors compare correctly when scoring a dock."""
    return math.atan2(math.sin(a), math.cos(a))


def marker_in_base(robot_xy_yaw, marker):
    """
    Return (x, y, yaw) of a marker face in the robot's base_link frame from ground truth.

    The detector's output is checked against this value, so the perception step that
    docking depends on is scored with no help from the nodes under test.
    """
    rx, ry, ryaw = robot_xy_yaw
    dx, dy = marker['x'] - rx, marker['y'] - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return (c * dx + s * dy, -s * dx + c * dy, wrap(marker['yaw'] - ryaw))


def detection_error(expected, measured):
    """
    Compare an expected and a measured (x, y, yaw) marker pose in base_link.

    Returns (range error [m], yaw error [rad]) so tests can apply the P1 bounds that keep
    the docking controller's inputs trustworthy.
    """
    ex, ey, eyaw = expected
    mx, my, myaw = measured
    return (abs(math.hypot(mx, my) - math.hypot(ex, ey)),
            abs(wrap(myaw - eyaw)))


def dock_score(robot_xy_yaw, marker, front_offset=0.15):
    """
    Score a dock: front-face distance to the marker plane, lateral offset, heading error.

    Implements the "definition of a successful dock" in the test plan; the front face is
    front_offset ahead of base_link.
    """
    rx, ry, ryaw = robot_xy_yaw
    nx, ny = math.cos(marker['yaw']), math.sin(marker['yaw'])
    fx, fy = rx + front_offset * math.cos(ryaw), ry + front_offset * math.sin(ryaw)
    dx, dy = fx - marker['x'], fy - marker['y']
    distance = dx * nx + dy * ny
    lateral = -dx * ny + dy * nx
    heading = abs(wrap(ryaw - (marker['yaw'] + math.pi)))
    return distance, abs(lateral), heading


def yaml_markers(scenario, gazebo_share):
    """
    Return {id: {x, y, yaw}} for scoring: the scenario's own markers, else config/markers.yaml.

    markers.yaml is the single source of truth for docking_world; scenarios that use another
    world (single_dock) list their marker poses in the scenario file instead.
    """
    import os
    import yaml
    if 'markers' in scenario:
        return {int(k): v for k, v in scenario['markers'].items()}
    with open(os.path.join(gazebo_share, 'config', 'markers.yaml')) as f:
        return {int(k): v for k, v in yaml.safe_load(f)['markers'].items()}
