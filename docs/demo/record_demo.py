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

"""Record a demo video of the robot docking to two markers, without touching the desktop.

Start the simulation first (headless is fine), then run this script from another terminal:

    ros2 launch dockbot_bringup dock.launch.py world:=docking_world headless:=true
    python3 docs/demo/record_demo.py --goals 4 0 --output docs/demo/dockbot_demo.mp4 --speed 2

The script adds a third-person camera to the running world, sends the /dock goals one after
the other, and writes each frame from simulation time, so the video plays at real speed even
when the simulator runs slower than real time. Each frame shows the third-person view, what
the robot's own camera sees with the detector's overlay, and the docking state as published
by the controller. With --raw-dir it also saves every frame's separate parts (the third-person
view, the robot's view and the state, as a log), which make_linkedin_video.py arranges into a
short video for a social post. It needs ffmpeg and a sourced ROS 2 workspace, and GZ_IP set
where the machine needs it for Gazebo discovery.
"""

import argparse
import json
import os
import signal
import subprocess
import tempfile
import threading
import time

import cv2
from dockbot_interfaces.action import Dock
from dockbot_interfaces.msg import DockingState
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WIDTH, HEIGHT = 1280, 720
CAMERA_W, CAMERA_H = 1080, 884  # the third-person camera's resolution
OVERVIEW_W = 880          # the third-person view fills the left part of each frame
PANEL_W = WIDTH - OVERVIEW_W
STATE_NAMES = {0: 'IDLE', 1: 'SEARCH', 2: 'APPROACH', 3: 'FINAL', 4: 'RECOVER', 5: 'DOCKED',
               6: 'FAILED'}
STATE_COLORS = {0: (140, 140, 140), 1: (0, 165, 255), 2: (255, 180, 60), 3: (80, 200, 120),
                4: (60, 60, 230), 5: (60, 200, 60), 6: (60, 60, 230)}  # BGR
OUTCOMES = {0: 'DOCKED', 1: 'TARGET NOT FOUND', 2: 'TARGET LOST', 3: 'RETRIES EXCEEDED',
            4: 'TIMEOUT', 5: 'CANCELED'}

# A static camera above the north-west corner, aimed at the area the robot docks in. It is added
# to the running world, so the world files stay as the tests use them.
OVERVIEW_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="overview_camera">
    <static>true</static>
    <pose>-3.3 3.3 3.6 0 0.47 -0.62</pose>
    <link name="link">
      <sensor name="overview" type="camera">
        <camera>
          <horizontal_fov>0.8</horizontal_fov>
          <image><width>%d</width><height>%d</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>30</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>15</update_rate>
        <topic>overview/image</topic>
      </sensor>
    </link>
  </model>
</sdf>
""" % (CAMERA_W, CAMERA_H)


def gz_service(world, name, req_type, request):
    """Call a Gazebo world service, so the demo can add its camera to the running simulation."""
    return subprocess.run(
        ['gz', 'service', '-s', '/world/%s/%s' % (world, name), '--reqtype', req_type,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '10000', '--req', request],
        capture_output=True, text=True)


def overview_topic_listed():
    """Return True when Gazebo lists the overview camera's image topic."""
    listed = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True).stdout
    return '/overview/image' in listed.split()


def wait_for(condition, seconds):
    """Poll until the condition holds or the time runs out, and return whether it held."""
    end = time.time() + seconds
    while time.time() < end:
        if condition():
            return True
        time.sleep(0.5)
    return condition()


def spawn_overview_camera(world):
    """Add the third-person camera to the world, replacing one left from an earlier run."""
    if overview_topic_listed():
        gz_service(world, 'remove', 'gz.msgs.Entity', 'name: "overview_camera" type: MODEL')
        # Gazebo applies the removal on a later step; creating before that would be undone by it.
        wait_for(lambda: not overview_topic_listed(), 15)
    with tempfile.NamedTemporaryFile('w', suffix='.sdf', delete=False) as f:
        f.write(OVERVIEW_SDF)
    done = gz_service(world, 'create', 'gz.msgs.EntityFactory', 'sdf_filename: "%s"' % f.name)
    # The server reads the file on a later step, so it is deleted only once the camera is there.
    appeared = wait_for(overview_topic_listed, 15)
    os.unlink(f.name)
    if 'true' not in done.stdout.lower() or not appeared:
        raise RuntimeError('could not add the overview camera: %r %r' % (done.stdout, done.stderr))


def to_bgr(msg):
    """Convert a rgb8 or bgr8 ROS image to an OpenCV BGR array for compositing the frame."""
    img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()


def put(img, text, org, scale=0.6, color=(255, 255, 255), thick=1):
    """Draw anti-aliased text, used for the status panel next to the docking views."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


class DemoRecorder(Node):
    """Collect the camera views and the docking state, and write one video frame per tick."""

    def __init__(self, writer, fps, speed, raw_dir=None):
        """Subscribe to the views and the controller state, and start the frame timer."""
        super().__init__('demo_recorder', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.writer = writer
        self.speed = speed
        self.raw_dir = raw_dir
        self.goal = None      # marker ID of the goal being docked to, None between goals
        self.result = None    # outcome and gap of the goal that just ended
        self.log = open(os.path.join(raw_dir, 'log.jsonl'), 'w') if raw_dir else None
        self.overview = self.debug = self.raw = None
        self.debug_time = -10.0
        self.state = None
        self.caption = ['']
        self.frames = 0
        self.create_subscription(Image, '/overview/image', self.on_overview, 1)
        self.create_subscription(Image, '/aruco/debug_image', self.on_debug, 1)
        self.create_subscription(Image, '/camera/image_raw', self.on_raw,
                                 qos_profile_sensor_data)
        self.create_subscription(DockingState, '/docking/state', self.on_state, 10)
        self.create_timer(1.0 / fps, self.on_frame)

    def now(self):
        """Return simulation time in seconds, the clock every frame is stamped with."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_overview(self, msg):
        """Keep the newest third-person frame."""
        self.overview = to_bgr(msg)

    def on_debug(self, msg):
        """Keep the newest detector overlay frame and when it arrived."""
        self.debug, self.debug_time = to_bgr(msg), self.now()

    def on_raw(self, msg):
        """Keep the newest raw camera frame, shown while the detector has nothing to draw."""
        self.raw = to_bgr(msg)

    def on_state(self, msg):
        """Keep the latest controller state for the status panel."""
        self.state = msg

    def robot_view(self):
        """Return the robot camera: the detector overlay if current, else the raw image."""
        recent = self.debug is not None and self.now() - self.debug_time < 0.5
        source = self.debug if recent else self.raw
        if source is None:
            return np.zeros((PANEL_W * 9 // 16, PANEL_W, 3), np.uint8)
        return cv2.resize(source, (PANEL_W, PANEL_W * 9 // 16), interpolation=cv2.INTER_AREA)

    def draw_panel(self, panel):
        """Draw the title, the robot's view and the docking state into the right-hand panel."""
        panel[:] = (32, 32, 32)
        put(panel, 'dockbot: ArUco docking', (14, 34), 0.8, thick=2)
        view = self.robot_view()
        y0 = 54
        panel[y0:y0 + view.shape[0]] = view
        put(panel, 'robot camera, detector overlay', (14, y0 + view.shape[0] + 22), 0.5,
            (180, 180, 180))
        y = y0 + view.shape[0] + 62
        for line in self.caption:
            put(panel, line, (14, y), 0.62, (255, 255, 255), 2)
            y += 28
        s = self.state
        y += 14
        if s is not None and s.state != 0:
            color = STATE_COLORS.get(s.state, (140, 140, 140))
            cv2.rectangle(panel, (14, y - 24), (PANEL_W - 14, y + 12), color, -1)
            put(panel, STATE_NAMES.get(s.state, '?'), (24, y + 2), 0.8, (20, 20, 20), 2)
            # The errors are NaN until the target has been seen, so they are shown as a dash.
            def show(value, fmt):
                return fmt % value if np.isfinite(value) else '-'

            rows = [('target marker', '%d' % s.target_id),
                    ('marker in view', 'yes' if s.target_visible else 'no'),
                    ('range to stop point', show(s.distance_to_dock * 100, '%.0f cm')),
                    ('sideways offset', show(s.lateral_error * 100, '%+.1f cm')),
                    ('heading error', show(np.degrees(s.heading_error), '%+.1f deg'))]
            for label, value in rows:
                y += 36
                put(panel, label, (14, y), 0.55, (180, 180, 180))
                put(panel, value, (PANEL_W - 130, y), 0.6, (255, 255, 255), 2)
        label = 'sim time %.1f s' % self.now()
        if self.speed != 1:
            label += '   (video at %gx speed)' % self.speed
        put(panel, label, (14, HEIGHT - 16), 0.5, (140, 140, 140))

    def save_raw(self):
        """Save this frame's parts and a log line, for arranging into another layout later."""
        n = self.frames
        cv2.imwrite(os.path.join(self.raw_dir, 'overview_%05d.jpg' % n), self.overview,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        view = cv2.resize(self.robot_view(), (640, 360), interpolation=cv2.INTER_AREA)
        cv2.imwrite(os.path.join(self.raw_dir, 'robot_%05d.jpg' % n), view,
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
        s = self.state

        def number(value):
            return float(value) if s is not None and np.isfinite(value) else None

        self.log.write(json.dumps({
            'frame': n, 't': round(self.now(), 3), 'goal': self.goal, 'result': self.result,
            'state': None if s is None else int(s.state),
            'visible': None if s is None else bool(s.target_visible),
            'range_cm': number(s.distance_to_dock * 100) if s is not None else None,
            'lateral_cm': number(s.lateral_error * 100) if s is not None else None,
            'heading_deg': number(np.degrees(s.heading_error)) if s is not None else None,
            }) + '\n')

    def on_frame(self):
        """Compose the third-person view and the panel into one frame and write it."""
        if self.overview is None:
            return
        if self.raw_dir:
            self.save_raw()
        frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        frame[:, :OVERVIEW_W] = cv2.resize(self.overview, (OVERVIEW_W, HEIGHT))
        self.draw_panel(frame[:, OVERVIEW_W:])
        self.writer.stdin.write(frame.tobytes())
        self.frames += 1


def wait_sim(node, seconds):
    """Wait for simulation time to advance, so the pauses in the video follow the clock."""
    end = node.now() + seconds
    while node.now() < end:
        time.sleep(0.05)


def dock(node, client, marker_id, index, total):
    """Send one /dock goal, narrate it on screen, and return its result."""
    node.caption = ['Goal %d of %d: dock to marker %d' % (index, total, marker_id)]
    node.goal, node.result = marker_id, None
    future = client.send_goal_async(Dock.Goal(marker_id=marker_id))
    while not future.done():
        time.sleep(0.05)
    handle = future.result()
    if not handle.accepted:
        node.caption = ['Goal rejected']
        return None
    result_future = handle.get_result_async()
    while not result_future.done():
        time.sleep(0.05)
    result = result_future.result().result
    outcome = OUTCOMES.get(result.outcome, str(result.outcome))
    node.caption = ['Marker %d: %s' % (marker_id, outcome),
                    'front face %.1f cm from the marker' % (result.final_distance * 100)]
    node.result = {'outcome': outcome, 'gap_cm': round(result.final_distance * 100, 1)}
    return result


def main():
    """Record the demo: add the camera, dock to each requested marker, and finish the video."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--goals', type=int, nargs='+', default=[4, 0], help='marker IDs in order')
    parser.add_argument('--world', default='docking_world')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--speed', type=float, default=2.0,
                        help='playback speed relative to simulation time, shown on screen')
    parser.add_argument('--output', default='docs/demo/dockbot_demo.mp4')
    parser.add_argument('--raw-dir', help='also save every frame\'s parts and a state log here')
    args = parser.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    if args.raw_dir:
        os.makedirs(args.raw_dir, exist_ok=True)

    spawn_overview_camera(args.world)
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_image', 'image_bridge', '/overview/image'],
                              start_new_session=True, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL)
    writer = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
         '-s', '%dx%d' % (WIDTH, HEIGHT), '-r', str(args.fps * args.speed), '-i', '-', '-c:v', 'libx264',
         '-pix_fmt', 'yuv420p', '-crf', '23', '-movflags', '+faststart', args.output],
        stdin=subprocess.PIPE)
    rclpy.init()
    node = DemoRecorder(writer, args.fps, args.speed, args.raw_dir)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        client = ActionClient(node, Dock, 'dock')
        if not client.wait_for_server(timeout_sec=30):
            raise RuntimeError('no /dock action server: is the simulation running?')
        deadline = time.time() + 60
        while node.overview is None and time.time() < deadline:
            time.sleep(0.2)
        if node.overview is None:
            raise RuntimeError('no frames from the overview camera')
        node.caption = ['Robot at its start pose']
        wait_sim(node, 3.0)
        for i, marker_id in enumerate(args.goals, 1):
            dock(node, client, marker_id, i, len(args.goals))
            wait_sim(node, 3.0)
    finally:
        executor.shutdown()
        if node.log:
            node.log.close()
        node.destroy_node()
        rclpy.shutdown()
        writer.stdin.close()
        writer.wait()
        os.killpg(bridge.pid, signal.SIGINT)
    print('wrote %s (%d frames)' % (args.output, node.frames))


if __name__ == '__main__':
    main()
