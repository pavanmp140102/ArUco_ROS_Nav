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

"""Check the world files against config/markers.yaml (test S4) and the marker textures."""

import math
import os
import xml.etree.ElementTree as ET

import cv2
import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def _markers():
    """Load the expected marker poses, the reference that every world must agree with."""
    with open(os.path.join(ROOT, 'config', 'markers.yaml')) as f:
        return yaml.safe_load(f)['markers']


def _wrap(angle):
    """Wrap an angle to [-pi, pi] so yaw values that differ by 2*pi compare as equal."""
    return math.atan2(math.sin(angle), math.cos(angle))


def test_world_matches_markers_yaml():
    """Each ID in markers.yaml must have the same pose in docking_world.sdf (S4)."""
    world = ET.parse(os.path.join(ROOT, 'worlds', 'docking_world.sdf')).getroot()
    found = {}
    for inc in world.iter('include'):
        uri = inc.find('uri').text
        if uri.startswith('model://aruco_marker_'):
            marker_id = int(uri.rsplit('_', 1)[1])
            x, y, z, _, _, yaw = [float(v) for v in inc.find('pose').text.split()]
            found[marker_id] = (x, y, z, yaw)
    expected = _markers()
    assert set(found) == set(expected)
    for marker_id, pose in expected.items():
        x, y, z, yaw = found[marker_id]
        assert abs(x - pose['x']) < 1e-3
        assert abs(y - pose['y']) < 1e-3
        assert abs(z - pose['z']) < 1e-3
        assert abs(_wrap(yaw - pose['yaw'])) < 0.01


def test_textures_decode_to_their_ids():
    """Each committed PNG must be read back as its own DICT_4X4_50 ID, which catches mirroring."""
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    else:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    for marker_id in _markers():
        path = os.path.join(ROOT, 'models', 'aruco_marker_%d' % marker_id,
                            'materials', 'textures', 'marker_%d.png' % marker_id)
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        assert image is not None and image.shape == (800, 800)
        _, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
        assert ids is not None and ids.flatten().tolist() == [marker_id]
