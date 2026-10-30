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

"""Draw docs/images/world_layout.png from the world file, markers.yaml and the E7 scenario.

Run from the repository root: python3 docs/images/world_layout.py. The figure is drawn from the
same files the simulator and the tests read, so it cannot drift from the world as built.
"""

import math
import os
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as patches  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
GAZEBO = os.path.join(ROOT, 'src', 'dockbot_gazebo')
CORRIDOR_WIDTH = 1.0   # m, kept clear in front of each marker
CORRIDOR_LENGTH = 2.5  # m
CHASSIS = (0.30, 0.24)  # m, length and width; base_link sits 0.15 m behind the front face


def load_obstacles():
    """Return the boxes and pillars of docking_world as dicts, read from the SDF."""
    world = ET.parse(os.path.join(GAZEBO, 'worlds', 'docking_world.sdf')).getroot().find('world')
    found = []
    for model in world.findall('model'):
        name = model.get('name')
        if not name.startswith(('box', 'pillar')):
            continue
        pose = [float(v) for v in model.find('pose').text.split()]
        shape = model.find('.//collision/geometry')[0]
        item = {'name': name, 'x': pose[0], 'y': pose[1], 'kind': shape.tag}
        if shape.tag == 'box':
            item['size'] = [float(v) for v in shape.find('size').text.split()[:2]]
        else:
            item['r'] = float(shape.find('radius').text)
        found.append(item)
    return found


def draw_robot(ax, x, y, yaw, color):
    """Draw the chassis footprint at a pose, with its front face marked in yellow."""
    length, width = CHASSIS
    c, s = math.cos(yaw), math.sin(yaw)

    def to_world(px, py):
        return (x + c * px - s * py, y + s * px + c * py)

    corners = [to_world(px, py) for px, py in
               [(-0.15, -width / 2), (0.15, -width / 2), (0.15, width / 2), (-0.15, width / 2)]]
    ax.add_patch(patches.Polygon(corners, closed=True, fc=color, ec='black', zorder=5))
    front = [to_world(0.15, -width / 2), to_world(0.15, width / 2)]
    ax.plot([front[0][0], front[1][0]], [front[0][1], front[1][1]], color='yellow', lw=3, zorder=6)


def main():
    """Build the figure and save it next to this script."""
    with open(os.path.join(GAZEBO, 'config', 'markers.yaml')) as f:
        markers = yaml.safe_load(f)['markers']
    with open(os.path.join(ROOT, 'src', 'dockbot_bringup', 'config', 'scenarios',
                           'e07_occluded.yaml')) as f:
        e7 = yaml.safe_load(f)['spawn']
    fig, ax = plt.subplots(figsize=(8, 8), dpi=133)
    ax.set_aspect('equal')
    ax.set_xlim(-3.9, 3.9)
    ax.set_ylim(-3.9, 3.9)
    ax.grid(alpha=0.25)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('docking_world: 7 m x 7 m room, five docking markers')
    ax.add_patch(patches.Rectangle((-3.6, -3.6), 7.2, 7.2, fill=False, ec='#8c8c8c', lw=10))
    for mid, m in sorted(markers.items()):
        yaw = m['yaw']
        nx, ny = math.cos(yaw), math.sin(yaw)
        # Corridor: CORRIDOR_WIDTH wide, CORRIDOR_LENGTH long, along the face's normal.
        corners = [(m['x'] + sx * -ny * CORRIDOR_WIDTH / 2 + ln * nx * CORRIDOR_LENGTH,
                    m['y'] + sx * nx * CORRIDOR_WIDTH / 2 + ln * ny * CORRIDOR_LENGTH)
                   for sx, ln in [(-1, 0), (1, 0), (1, 1), (-1, 1)]]
        ax.add_patch(patches.Polygon(corners, closed=True, fc='#2ca02c', alpha=0.12, ec='none'))
        ax.plot([m['x'] - ny * 0.1, m['x'] + ny * 0.1], [m['y'] + nx * 0.1, m['y'] - nx * 0.1],
                color='#d62728', lw=6, solid_capstyle='butt', zorder=4)
        ax.annotate('', xy=(m['x'] + nx * 0.7, m['y'] + ny * 0.7), xytext=(m['x'], m['y']),
                    arrowprops={'arrowstyle': '->', 'color': '#d62728', 'lw': 2}, zorder=4)
        ax.text(m['x'] + nx * 0.95, m['y'] + ny * 0.95 + (0.18 if abs(ny) < 0.5 else 0.0),
                'ID %d' % mid, color='#d62728', fontsize=12, fontweight='bold',
                ha='center', va='center')
    for o in load_obstacles():
        if o['kind'] == 'box':
            w, h = o['size']
            ax.add_patch(patches.Rectangle((o['x'] - w / 2, o['y'] - h / 2), w, h,
                                           fc='#c8b595', ec='black', zorder=3))
            ax.text(o['x'], o['y'], o['name'].replace('_', ' '), ha='center', va='center',
                    fontsize=8, zorder=4)
        else:
            ax.add_patch(patches.Circle((o['x'], o['y']), o['r'], fc='#b39ddb', ec='black',
                                        zorder=3))
            ax.text(o['x'] + o['r'] + 0.1, o['y'], o['name'].replace('_', ' '),
                    fontsize=8, va='center')
    draw_robot(ax, 0.0, 0.0, 0.0, '#1f77b4')
    ax.annotate('default spawn', xy=(0, -0.12), xytext=(-0.6, -0.75), fontsize=9,
                arrowprops={'arrowstyle': '-', 'lw': 0.8})
    draw_robot(ax, e7['x'], e7['y'], e7['yaw'], '#ff7f0e')
    ax.text(e7['x'] + 0.2, e7['y'] - 0.3, 'E7 spawn\n(pillar blocks marker 2)', fontsize=8,
            color='#b35900')
    fig.text(0.5, 0.012, 'red bar and arrow: marker and the direction its printed face points;  '
             'green: docking corridor (kept clear)', ha='center', fontsize=8)
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'world_layout.png')
    fig.savefig(out, bbox_inches='tight')
    print('wrote', os.path.normpath(out))


if __name__ == '__main__':
    main()
