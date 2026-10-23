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

"""End-to-end docking tests E1 to E9, E11 and E12: one scenario file per launch test."""

import json
import math
import os
import statistics
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from dockbot_interfaces.action import Dock
from dockbot_interfaces.msg import DockingState
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from scoring import dock_score, yaml_markers, yaw_of
import yaml

STATE_NAMES = {0: 'IDLE', 1: 'SEARCH', 2: 'APPROACH', 3: 'FINAL', 4: 'RECOVER',
               5: 'DOCKED', 6: 'FAILED'}
OUTCOME_NAMES = {0: 'DOCKED', 1: 'TARGET_NOT_FOUND', 2: 'TARGET_LOST', 3: 'RETRIES_EXCEEDED',
                 4: 'TIMEOUT', 5: 'CANCELED'}

# Definition of a successful dock, from docs/test_plan.md.
GAP_NOMINAL, GAP_TOLERANCE = 0.10, 0.02
MAX_LATERAL = 0.03
MAX_HEADING = math.radians(5.0)
# Contact check: the front face must never get closer than this to the marker plane.
MIN_FRONT_GAP = 0.05
# Radius of the robot footprint about base_link (half the chassis diagonal), for keep-out zones.
ROBOT_RADIUS = 0.19


def load_scenario():
    """Read the scenario file named by the SCENARIO environment variable."""
    # APPEND_ENV in CMake joins values to the variable with a colon, so a leading one is dropped.
    name = os.environ['SCENARIO'].strip(':')
    path = os.path.join(
        get_package_share_directory('dockbot_bringup'), 'config', 'scenarios', name + '.yaml')
    with open(path) as f:
        return name, yaml.safe_load(f)


@pytest.mark.launch_test
def generate_test_description():
    """Start headless Gazebo, the detector and the controller for the chosen scenario."""
    _, scenario = load_scenario()
    spawn = scenario['spawn']
    args = {
        'world': scenario['world'], 'headless': 'true',
        'x': str(spawn['x']), 'y': str(spawn['y']), 'yaw': str(spawn['yaw']),
        'camera_noise': str(scenario.get('camera_noise', 0.0)),
        'marker_id': '-1',
    }
    if 'DOCKBOT_GPU' in os.environ:  # otherwise keep the launch file's own default
        args['gpu'] = os.environ['DOCKBOT_GPU']
    dock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('dockbot_bringup'), 'launch', 'dock.launch.py')),
        launch_arguments=args.items())
    return LaunchDescription([
        # Discovery over the loopback interface keeps Gazebo reachable on machines whose
        # network interfaces do not carry multicast.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        dock, launch_testing.actions.ReadyToTest()])


class TestScenario(unittest.TestCase):
    """Run the trials of one scenario and score each against Gazebo ground truth."""

    @classmethod
    def setUpClass(cls):
        """Create the checker node: ground truth, controller state and the /dock client."""
        cls.name, cls.scenario = load_scenario()
        rclpy.init()
        cls.node = rclpy.create_node('e2e_checker', parameter_overrides=[
            rclpy.Parameter('use_sim_time', value=True)])
        cls.truth = None
        cls.state = None
        cls.state_log = []
        cls.trajectory = []
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls._on_truth, qos_profile_sensor_data)
        cls.node.create_subscription(DockingState, '/docking/state', cls._on_state, 10)
        cls.client = ActionClient(cls.node, Dock, 'dock')
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spinner = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spinner.start()
        cls.markers = yaml_markers(cls.scenario, get_package_share_directory('dockbot_gazebo'))
        cls.rows = []

    @classmethod
    def tearDownClass(cls):
        """Print the result rows, optionally save them, and shut rclpy down."""
        print('\nE2E RESULTS ' + cls.name + ' ' + json.dumps(cls.rows))
        out = os.environ.get('E2E_RESULTS_DIR')
        if out:
            os.makedirs(out, exist_ok=True)
            with open(os.path.join(out, cls.name + '.json'), 'w') as f:
                json.dump(cls.rows, f, indent=1)
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_truth(cls, msg):
        """Keep the latest true robot pose and append it to the trajectory log."""
        p = msg.pose.pose
        q = p.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cls.truth = (t, p.position.x, p.position.y, yaw_of((q.x, q.y, q.z, q.w)))
        cls.trajectory.append(cls.truth)

    @classmethod
    def _on_state(cls, msg):
        """Keep the latest controller state and log each change, in order."""
        if not cls.state_log or cls.state_log[-1] != msg.state:
            cls.state_log.append(msg.state)
        cls.state = msg

    def sim_now(self):
        """Return the current simulation time in seconds."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def wait(self, predicate, sim_seconds, what, wall_limit=900.0):
        """Poll until predicate() holds; fail after sim_seconds (sim) or wall_limit (real)."""
        t0, w0 = self.sim_now(), time.time()
        while time.time() - w0 < wall_limit:
            if predicate():
                return
            if self.sim_now() - t0 > sim_seconds:
                break
            time.sleep(0.05)
        self.fail('timed out waiting for ' + what)

    def pose(self):
        """Latest true pose as (x, y, yaw)."""
        return self.truth[1:]

    def stationary_for(self, seconds):
        """Return True if the robot moved under 5 mm and turned under 0.01 rad over seconds."""
        now = self.sim_now()
        recent = [s for s in self.trajectory[-400:] if s[0] >= now - seconds]
        if not recent or recent[0][0] > now - seconds + 0.3:
            return False
        xs = [s[1] for s in recent]
        ys = [s[2] for s in recent]
        yaws = [s[3] for s in recent]
        return (max(xs) - min(xs) < 0.005 and max(ys) - min(ys) < 0.005 and
                max(yaws) - min(yaws) < 0.01)

    def reset_pose(self, pose):
        """Teleport the robot to pose (x, y, yaw) with the Gazebo set_pose service."""
        req = ('name: "dockbot", position: {x: %f, y: %f, z: 0.01}, '
               'orientation: {z: %f, w: %f}' % (
                   pose['x'], pose['y'], math.sin(pose['yaw'] / 2), math.cos(pose['yaw'] / 2)))
        subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % self.scenario['world'],
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req], check=True, capture_output=True)
        self.wait(lambda: self.stationary_for(1.0), 30, 'the robot to settle after the reset')

    def send_goal(self, goal):
        """Send a Dock goal and return its handle; the goal must be accepted."""
        msg = Dock.Goal(
            marker_id=int(goal['marker_id']), dock_distance=float(goal.get('dock_distance', 0.0)),
            timeout=float(goal.get('timeout', 0.0)))
        future = self.client.send_goal_async(msg)
        self.wait(future.done, 10, 'the goal to be answered')
        handle = future.result()
        self.assertTrue(handle.accepted, 'the controller rejected the goal')
        return handle

    def wait_result(self, handle, sim_seconds):
        """Wait for the action result and return it."""
        future = handle.get_result_async()
        self.wait(future.done, sim_seconds, 'the action result')
        return future.result().result

    def test_scenario(self):
        """Run every trial of the scenario in order."""
        self.wait(lambda: self.truth is not None, 180, 'ground truth', wall_limit=180)
        # More than one /clock publisher means a stray simulator is on the same ROS domain.
        self.assertEqual(
            len(self.node.get_publishers_info_by_topic('/clock')), 1,
            'expected exactly one /clock publisher; kill stray Gazebo or bridge processes')
        self.wait(lambda: self.client.server_is_ready(), 120, 'the /dock action server',
                  wall_limit=120)
        self.wait(lambda: self.stationary_for(1.0), 60, 'the spawned robot to settle')
        for index, trial in enumerate(self.scenario['trials']):
            with self.subTest(trial=index, goal=trial['goal']['marker_id']):
                self.run_trial(index, trial)
        limit = self.scenario.get('stats', {}).get('max_gap_std')
        if limit is not None:
            gaps = [r['gap'] for r in self.rows if 'gap' in r]
            self.assertLessEqual(statistics.pstdev(gaps), limit,
                                 'standard deviation of the final gap over %d runs' % len(gaps))

    def run_trial(self, index, trial):
        """Run one goal: optional reset, send, optional cancel, wait, then score."""
        if 'reset_to' in trial:
            self.reset_pose(trial['reset_to'])
        goal_id = int(trial['goal']['marker_id'])
        start_index = len(self.trajectory)
        del self.state_log[:]
        t_start = self.sim_now()
        handle = self.send_goal(trial['goal'])
        expect = trial['expect']
        t_cancel = None
        if 'cancel' in trial:
            wanted = trial['cancel']['in_state']
            self.wait(lambda: self.state is not None and
                      STATE_NAMES[self.state.state] == wanted, trial['time_limit'],
                      'state ' + wanted)
            t_in_state = self.sim_now()
            self.wait(lambda: self.sim_now() - t_in_state >= trial['cancel']['after'], 60,
                      'the cancel delay')
            t_cancel = self.sim_now()
            handle.cancel_goal_async()
        result = self.wait_result(handle, trial['time_limit'] + 30)
        duration = self.sim_now() - t_start
        outcome = OUTCOME_NAMES.get(result.outcome, result.outcome)
        # Score after the robot has been stationary for one second, as the test plan requires.
        self.wait(lambda: self.stationary_for(1.0), 30, 'the robot to stop')
        row = {'trial': index, 'goal': goal_id, 'outcome': outcome, 'duration': round(duration, 1),
               'states': '>'.join(STATE_NAMES[s][:3] for s in self.state_log)}
        if goal_id in self.markers:
            gap, lateral, heading = dock_score(self.pose(), self.markers[goal_id])
            row.update(gap=round(gap, 4), lateral=round(lateral, 4),
                       heading_deg=round(math.degrees(heading), 2))
        else:  # a goal for a marker that is not in the world (E6) has nothing to score against
            gap, lateral, heading = float('inf'), float('inf'), float('inf')
        self.rows.append(row)
        print('E2E TRIAL ' + json.dumps(row))
        path = self.trajectory[start_index:]

        # Contact checks apply to every outcome: markers and keep-out zones.
        for marker_id, marker in self.markers.items():
            lowest = min(dock_score(s[1:], marker)[0] for s in path)
            near = min(abs(dock_score(s[1:], marker)[1]) for s in path)
            if near < 0.15:
                self.assertGreaterEqual(
                    lowest, MIN_FRONT_GAP,
                    'front face came within %.3f m of marker %d' % (lowest, marker_id))
        for zone in self.scenario.get('keep_out', []):
            closest = min(math.hypot(s[1] - zone['x'], s[2] - zone['y']) for s in path)
            self.assertGreater(closest, zone['r'] + ROBOT_RADIUS,
                               'robot touched ' + zone['name'])

        docked_ok = (result.outcome == 0 and result.success and
                     abs(gap - GAP_NOMINAL) <= GAP_TOLERANCE and lateral <= MAX_LATERAL and
                     heading <= MAX_HEADING)
        if expect == 'docked':
            self.assertTrue(docked_ok, 'not docked within tolerance: %s' % row)
            self.assertLessEqual(duration, trial['time_limit'], 'docking took too long')
            self.assert_not_at_other_marker(goal_id)
        elif expect == 'docked_or_stopped':
            if not docked_ok:
                self.assertNotEqual(result.outcome, 0, 'reported docked but not in tolerance')
                self.assertTrue(self.stationary_for(1.0), 'robot still moving after the failure')
        elif expect == 'target_not_found':
            self.assertEqual(outcome, 'TARGET_NOT_FOUND')
            self.assertTrue(self.stationary_for(1.0), 'robot still moving after the failure')
        elif expect == 'canceled':
            self.assertEqual(outcome, 'CANCELED')
            self.assertLessEqual(self.stopped_after(t_cancel), 0.5,
                                 'robot took too long to stop after the cancel')
        else:
            self.fail('unknown expectation ' + str(expect))

    def assert_not_at_other_marker(self, goal_id):
        """Fail if the final pose sits in front of any marker other than the goal."""
        for marker_id, marker in self.markers.items():
            if marker_id == goal_id:
                continue
            gap, lateral, _ = dock_score(self.pose(), marker)
            self.assertFalse(abs(gap) < 0.5 and lateral < 0.3,
                             'ended in front of marker %d' % marker_id)

    def stopped_after(self, t_cancel):
        """Seconds after t_cancel until the robot's speed first stayed below 1 cm/s."""
        samples = [s for s in self.trajectory if s[0] >= t_cancel]
        for i in range(1, len(samples)):
            window = [s for s in samples[i:] if s[0] <= samples[i][0] + 0.3]
            if len(window) < 3:
                break
            speed = math.hypot(window[-1][1] - window[0][1], window[-1][2] - window[0][2]) / max(
                window[-1][0] - window[0][0], 1e-3)
            if speed < 0.01:
                return samples[i][0] - t_cancel
        return float('inf')
