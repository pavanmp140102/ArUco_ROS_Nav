#!/usr/bin/env python3
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

"""
Draw the documentation figures: the world plan view and the robot and docking geometry.

The world figure is read from worlds/docking_world.sdf and config/markers.yaml, so it
cannot drift from the simulation. Run from anywhere: python3 docs/images/generate_diagrams.py
"""

import math
import os
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle  # noqa: E402
import yaml  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO = os.path.join(HERE, '..', '..', 'src', 'dockbot_gazebo')
COLORS = {'marker': '#d62728', 'robot': '#1f77b4', 'corridor': '#2ca02c', 'wall': '#8c8c8c'}


def read_world():
    """Parse the walls and obstacles out of docking_world.sdf so the plan matches the sim."""
    root = ET.parse(os.path.join(GAZEBO, 'worlds', 'docking_world.sdf')).getroot()
    shapes = []
    for model in root.iter('model'):
        name = model.get('name')
        if name == 'ground_plane' or model.find('pose') is None:
            continue
        x, y = [float(v) for v in model.find('pose').text.split()[:2]]
        box = model.find('.//collision//box/size')
        cyl = model.find('.//collision//cylinder')
        if box is not None:
            sx, sy, _ = [float(v) for v in box.text.split()]
            shapes.append((name, 'box', x, y, sx, sy))
        elif cyl is not None:
            shapes.append((name, 'cylinder', x, y, float(cyl.find('radius').text), 0))
    return shapes


def read_markers():
    """Load marker poses from markers.yaml, the single source of truth for marker positions."""
    with open(os.path.join(GAZEBO, 'config', 'markers.yaml')) as f:
        return yaml.safe_load(f)['markers']


def draw_robot_top(ax, x, y, yaw, scale=1.0, color=None):
    """Draw dockbot from above (0.30 x 0.24 m body, front face marked) at a world pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(-0.15, -0.12), (0.15, -0.12), (0.15, 0.12), (-0.15, 0.12)]
    pts = [(x + c * px * scale - s * py * scale, y + s * px * scale + c * py * scale)
           for px, py in corners]
    ax.add_patch(Polygon(pts, closed=True, fc=color or COLORS['robot'], ec='k', lw=0.8, zorder=5))
    ax.plot([pts[1][0], pts[2][0]], [pts[1][1], pts[2][1]], color='yellow', lw=2.5, zorder=6)


def world_layout():
    """Plan view of docking_world: markers and their facing, docking corridors, obstacles."""
    fig, ax = plt.subplots(figsize=(7.6, 7.6))
    for name, kind, x, y, a, b in read_world():
        if name.startswith('wall'):
            ax.add_patch(Rectangle((x - a / 2, y - b / 2), a, b, fc=COLORS['wall'], ec='none'))
        elif kind == 'box':
            ax.add_patch(Rectangle((x - a / 2, y - b / 2), a, b, fc='#c9b79c', ec='k', lw=0.8))
            ax.text(x, y, name.replace('box_', 'box ').replace('_', ' '), ha='center',
                    va='center', fontsize=7)
        else:
            ax.add_patch(Circle((x, y), a, fc='#b39ddb', ec='k', lw=0.8))
            label = 'occluder' if 'occluder' in name else 'pillar'
            ax.text(x + a + 0.08, y, label, fontsize=7, va='center')

    markers = read_markers()
    for marker_id, m in markers.items():
        x, y, yaw = m['x'], m['y'], m['yaw']
        nx, ny = math.cos(yaw), math.sin(yaw)
        # docking corridor: 1 m wide, 2.5 m long, in front of the marker face
        corner = (x + ny * 0.5, y - nx * 0.5)
        ax.add_patch(Rectangle(corner, 2.5, 1.0, angle=math.degrees(yaw), fc=COLORS['corridor'],
                               alpha=0.13, ec=COLORS['corridor'], lw=0.6, ls='--'))
        ax.add_patch(FancyArrowPatch((x, y), (x + 0.7 * nx, y + 0.7 * ny), color=COLORS['marker'],
                                     arrowstyle='-|>', mutation_scale=12, lw=1.6, zorder=6))
        ax.plot([x - ny * 0.1, x + ny * 0.1], [y + nx * 0.1, y - nx * 0.1],
                color=COLORS['marker'], lw=5, solid_capstyle='butt', zorder=7)
        ox, oy = (0.0, 0.0)
        ax.text(x + nx * 0.95 + ox, y + ny * 0.95 + oy, 'ID %d' % marker_id, color=COLORS['marker'],
                fontsize=10, fontweight='bold', ha='center', va='center', zorder=8)

    draw_robot_top(ax, 0, 0, 0)
    ax.annotate('default spawn', (0, 0), (-0.2, -0.75), fontsize=8, ha='center',
                arrowprops=dict(arrowstyle='-', lw=0.6))

    # candidate pose for test E7: marker 2 hidden behind the occluder pillar
    ex, ey, eyaw = 2.0, 0.5, 2.44
    draw_robot_top(ax, ex, ey, eyaw, color='#ff7f0e')
    ax.plot([ex, markers[2]['x']], [ey, markers[2]['y']], color='#ff7f0e', ls=':', lw=1.4)
    ax.text(ex + 0.2, ey - 0.35, 'E7 candidate spawn\n(marker 2 hidden)', fontsize=7.5,
            color='#b35a00')

    ax.set_xlim(-3.9, 3.9)
    ax.set_ylim(-3.9, 3.9)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(alpha=0.25)
    ax.set_title('docking_world: 7 m x 7 m room, five docking markers', fontsize=11)
    ax.text(0, -3.82, 'red bar and arrow: marker and the direction its printed face points;  '
            'green: docking corridor (kept clear)', ha='center', fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'world_layout.png'), dpi=140)
    plt.close(fig)


def dim_arrow(ax, x0, x1, y, text, dy=0.012):
    """Draw a horizontal dimension arrow with a centred label above it."""
    ax.annotate('', (x1, y), (x0, y), arrowprops=dict(arrowstyle='<->', lw=1.1, color='k'))
    ax.text((x0 + x1) / 2, y + dy, text, ha='center', va='bottom', fontsize=8.5)


def robot_side_view():
    """Side view of dockbot docked 10 cm from a marker: camera setback and what the camera sees."""
    fig, ax = plt.subplots(figsize=(8.6, 4.2))
    ax.add_patch(Rectangle((-0.3, -0.01), 0.75, 0.01, fc='#bbbbbb', ec='none'))
    ax.add_patch(Rectangle((-0.15, 0.02), 0.30, 0.08, fc=COLORS['robot'], ec='k', lw=0.8))
    ax.add_patch(Circle((0, 0.04), 0.04, fc='#333333', ec='k'))
    ax.add_patch(Circle((0.12, 0.02), 0.02, fc='#777777', ec='k'))
    ax.add_patch(Circle((-0.12, 0.02), 0.02, fc='#777777', ec='k'))
    ax.add_patch(Rectangle((-0.06, 0.10), 0.02, 0.10, fc='#333333', ec='k', lw=0.8))
    ax.add_patch(Rectangle((-0.07, 0.19), 0.04, 0.03, fc='black'))
    # marker plate at the dock: face 0.10 m ahead of the front face
    face = 0.15 + 0.10
    ax.add_patch(Rectangle((face, 0.0), 0.05, 0.30, fc='#555555', ec='k', lw=0.8))
    ax.add_patch(Rectangle((face - 0.004, 0.10), 0.004, 0.20, fc='white', ec='k', lw=0.8))
    ax.add_patch(Rectangle((face - 0.004, 0.125), 0.004, 0.15, fc='black', ec='none'))
    # camera view cone (vertical field of view 37.5 degrees)
    cx, cz = -0.05, 0.20
    half = math.radians(37.5 / 2)
    reach = face - cx
    ax.add_patch(Polygon([(cx, cz), (face, cz + reach * math.tan(half)),
                          (face, cz - reach * math.tan(half))], closed=True,
                         fc='#ffcc00', alpha=0.25, ec='#d9a800', lw=0.8))
    ax.plot([cx, face], [cz, cz], color='#d9a800', ls='--', lw=0.9)
    ax.plot([cx], [cz], 'o', color='#d9a800', ms=4)
    ax.text(cx - 0.012, cz + 0.035, 'camera', ha='right', fontsize=8.5)
    ax.text(0.0, -0.02, 'base_link (axle)', ha='center', va='top', fontsize=8)
    ax.text(face + 0.025, 0.305, 'marker plate', ha='center', va='bottom', fontsize=8.5)
    ax.plot([0.15, 0.15], [0.0, 0.36], color='k', lw=0.6, ls=':')
    ax.text(0.15, -0.02, 'front face', ha='center', va='top', fontsize=8)
    ax.plot([cx, cx], [0.22, 0.36], color='k', lw=0.6, ls=':')
    dim_arrow(ax, cx, 0.15, 0.355, '0.20 m setback')
    dim_arrow(ax, 0.15, face, 0.355, '0.10 m dock')
    ax.annotate('', (-0.2, 0.20), (-0.2, 0.0), arrowprops=dict(arrowstyle='<->', lw=1.1))
    ax.text(-0.215, 0.10, '0.20 m', rotation=90, ha='right', va='center', fontsize=8.5)
    ax.text(0.34, 0.16, 'camera to marker 0.30 m;\nthe 0.15 m code fills\n74% of the image height',
            fontsize=8, va='center')
    ax.set_xlim(-0.30, 0.62)
    ax.set_ylim(-0.08, 0.42)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('dockbot at the docking position (side view)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'robot_side_view.png'), dpi=150)
    plt.close(fig)


def pipeline():
    """Block diagram of the data flow from camera image to wheel commands."""
    fig, ax = plt.subplots(figsize=(10, 3.4))
    boxes = [
        (0.2, 'Gazebo\ncamera\n1280x720 @ 30 Hz', '#e3f2fd'),
        (2.6, 'aruco_detector\nimage -> marker pose\nin base_link', '#fff3e0'),
        (5.4, 'docking_controller\nstate machine +\ncontrol law', '#e8f5e9'),
        (8.0, 'Gazebo\ndiff drive', '#e3f2fd'),
    ]
    for x, text, color in boxes:
        ax.add_patch(Rectangle((x, 1.0), 1.9, 1.3, fc=color, ec='k', lw=1))
        ax.text(x + 0.95, 1.65, text, ha='center', va='center', fontsize=8.5)
    links = [(2.1, 2.6, '/camera/image_raw\n/camera/camera_info'),
             (4.5, 5.4, '/aruco/markers'),
             (7.3, 8.0, '/cmd_vel\n(TwistStamped)')]
    for x0, x1, label in links:
        ax.add_patch(FancyArrowPatch((x0, 1.65), (x1, 1.65), arrowstyle='-|>', mutation_scale=14,
                                     lw=1.5))
        ax.text((x0 + x1) / 2, 2.45, label, ha='center', fontsize=7.5)
    ax.text(6.35, 0.45, '/dock action goal: marker_id', ha='center', fontsize=8)
    ax.add_patch(FancyArrowPatch((6.35, 0.65), (6.35, 1.0), arrowstyle='-|>', mutation_scale=12))
    ax.text(5.0, 3.15, 'Control uses only the camera. Odometry, maps and ground truth are not '
            'inputs to the controller.', ha='center', fontsize=8.5, style='italic')
    ax.set_xlim(0, 10.2)
    ax.set_ylim(0.2, 3.5)
    ax.axis('off')
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'pipeline.png'), dpi=140)
    plt.close(fig)


if __name__ == '__main__':
    world_layout()
    robot_side_view()
    pipeline()
    print('wrote world_layout.png, robot_side_view.png, pipeline.png')
