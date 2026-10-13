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

"""E10: detector pose against Gazebo ground truth at four distances from the marker."""

import math
import os
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from dockbot_interfaces.msg import ArucoMarkerArray
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data

from scoring import marker_in_base, wrap, yaw_of


def stamp(header):
    """Return a message header time in seconds, to compare against the simulation clock."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


MARKER = {'x': 3.0, 'y': 0.0, 'yaw': math.pi}  # marker_0 in single_dock.sdf
DISTANCES = [0.3, 1.0, 2.0, 3.8]  # base_link to marker plane [m]


@pytest.mark.launch_test
def generate_test_description():
    """Start headless single_dock and the detector; the nodes under test see no ground truth."""
    def share(pkg):
        return get_package_share_directory(pkg)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('dockbot_gazebo'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'world': 'single_dock', 'headless': 'true',
            'gpu': os.environ.get('DOCKBOT_GPU', 'intel'), 'x': '0.0'}.items())
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share('dockbot_perception'), 'launch', 'detector.launch.py')))
    return LaunchDescription([
        # Discovery over the loopback interface keeps Gazebo reachable on machines whose
        # network interfaces do not carry multicast.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        sim, detector, launch_testing.actions.ReadyToTest()])


class TestDetectionAgainstGroundTruth(unittest.TestCase):
    """Place the robot at known distances and compare detector output to ground truth."""

    @classmethod
    def setUpClass(cls):
        """Subscribe to the detector output and the (test-only) ground truth."""
        rclpy.init()
        cls.node = rclpy.create_node('e10_checker', parameter_overrides=[
            rclpy.Parameter('use_sim_time', value=True)])
        cls.markers = None
        cls.truth = None
        cls.node.create_subscription(
            ArucoMarkerArray, '/aruco/markers', cls._on_markers, 10)
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls._on_truth, qos_profile_sensor_data)
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()

    @classmethod
    def tearDownClass(cls):
        """Stop spinning and shut rclpy down."""
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_markers(cls, msg):
        """Keep the latest detector output."""
        cls.markers = msg

    @classmethod
    def _on_truth(cls, msg):
        """Keep the latest true robot pose."""
        cls.truth = msg

    def _wait(self, predicate, timeout, what):
        """Poll until predicate() is true or fail with a message naming what was awaited."""
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return
            time.sleep(0.1)
        self.fail('timed out waiting for ' + what)

    def _place(self, x):
        """Teleport the stationary robot to base_link x facing +x, via the Gazebo service."""
        req = 'name: "dockbot", position: {x: %f, y: 0.0, z: 0.01}, orientation: {w: 1}' % x
        subprocess.run(
            ['gz', 'service', '-s', '/world/single_dock/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req], check=True, capture_output=True)

    def test_detector_matches_ground_truth(self):
        """For each distance the detected marker pose is within the P1 bounds."""
        self._wait(lambda: self.truth is not None, 120, 'ground truth')
        # More than one /clock publisher means a stray simulator is on the same ROS domain,
        # which would feed this test timestamps from another world.
        self.assertEqual(
            len(self.node.get_publishers_info_by_topic('/clock')), 1,
            'expected exactly one /clock publisher; kill stray Gazebo or bridge processes')
        for d in DISTANCES:
            with self.subTest(distance=d):
                self._place(MARKER['x'] - d)
                # Only accept messages stamped well after the move, in simulation time, so
                # a frame or pose from the previous placement cannot be scored.
                t_ready = self.node.get_clock().now().nanoseconds * 1e-9 + 1.5

                def fresh():
                    m, t = self.markers, self.truth
                    return (m is not None and m.markers and t is not None and
                            stamp(m.header) > t_ready and stamp(t.header) > t_ready)
                self._wait(fresh, 30, 'a fresh detection at %.1f m' % d)
                msg, truth = self.markers, self.truth
                self.assertEqual([m.id for m in msg.markers], [0])
                p = truth.pose.pose
                robot = (p.position.x, p.position.y,
                         yaw_of((p.orientation.x, p.orientation.y,
                                 p.orientation.z, p.orientation.w)))
                ex, ey, eyaw = marker_in_base(robot, MARKER)
                m = msg.markers[0].pose
                q = m.orientation
                # z axis of the marker frame (the face normal) expressed in base_link.
                nx = 2.0 * (q.x * q.z + q.w * q.y)
                ny = 2.0 * (q.y * q.z - q.w * q.x)
                myaw = math.atan2(ny, nx)
                rng = math.hypot(ex, ey)
                range_err = abs(math.hypot(m.position.x, m.position.y) - rng)
                yaw_err = abs(wrap(myaw - eyaw))
                # Rendered markers are about one pixel off at long range (a 41 px wide marker
                # at 3.8 m is 2.4% per pixel), so beyond 2 m the bound is 3.5% instead of the
                # synthetic-image 2% of P1.
                bound = max((0.02 if rng <= 2.0 else 0.035) * rng, 0.01)
                self.assertLessEqual(range_err, bound,
                                     'range error %.4f m at %.1f m' % (range_err, d))
                if rng < 2.0:
                    self.assertLessEqual(math.degrees(yaw_err), 3.0,
                                         'yaw error %.2f deg at %.1f m' % (
                                             math.degrees(yaw_err), d))
