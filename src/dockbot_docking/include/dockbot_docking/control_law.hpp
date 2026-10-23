//
// Copyright 2026 Pavan Patil
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// @Author: Pavan Patil
// @Date: 2026-10-23
// @Description: ROS-free docking control law: marker pose in base_link to velocity commands.

#ifndef DOCKBOT_DOCKING__CONTROL_LAW_HPP_
#define DOCKBOT_DOCKING__CONTROL_LAW_HPP_

#include <Eigen/Geometry>

namespace dockbot_docking
{

/// All tunable numbers of the docking controller; defaults match docs/interfaces.md.
struct ControlParams
{
  double dock_distance = 0.10;
  double front_offset = 0.15;
  double final_range = 0.50;
  double aim_gain = 0.8;
  double max_bearing_offset = 0.44;
  double yaw_trust_range = 2.0;
  double v_max = 0.30;
  double v_final = 0.05;
  double w_max = 1.0;
  double a_max = 0.5;
  double alpha_max = 2.0;
  double search_angular_speed = 0.4;
  double gate_lateral = 0.03;
  double gate_heading = 0.087;
  double recover_range = 2.0;
  /// Proportional gain from desired heading (rad) to turn rate (rad/s) during approach.
  double turn_gain = 1.5;
  /// Speed added per metre of range beyond final_range during approach, 1/s.
  double speed_gain = 0.5;
  /// Final-approach turn-rate gain on heading error psi.
  double final_heading_gain = 1.5;
  /// Final-approach turn-rate gain on lateral error e_y, rad/s per metre.
  double final_lateral_gain = 3.0;
};

/// Marker pose reduced to the four numbers the controller works with.
struct DockErrors
{
  /// Distance from the marker plane to base_link, along the marker normal, metres.
  double d = 0.0;
  /// Sideways offset of base_link from the normal line, metres. Positive when the robot is on
  /// the +y side of the marker normal frame, i.e. right of the line as seen from the robot.
  double e_y = 0.0;
  /// Angle from the robot's forward axis to the inward normal, radians, positive to the left.
  double psi = 0.0;
  /// Angle from the robot's forward axis to the marker centre, radians, positive to the left.
  double bearing = 0.0;
  /// Ground-plane distance to the marker centre, metres.
  double range = 0.0;
};

/// Body-frame velocity command for the differential-drive base.
struct VelocityCommand
{
  /// Forward speed, m/s (negative reverses).
  double v = 0.0;
  /// Yaw rate, rad/s, positive turns left.
  double w = 0.0;
};

/**
 * Reduce a marker pose in base_link to the docking error terms d, e_y, psi, bearing and range.
 *
 * Every later decision (approach, gates, stop) is made from these numbers, so this is the one
 * place where the marker's z axis (the docking normal) is turned into robot-relative errors.
 */
DockErrors compute_errors(const Eigen::Isometry3d & T_base_marker);

/**
 * Heading the robot should turn toward during approach, relative to its forward axis.
 *
 * It aims at a point on the marker normal and clamps the result so the marker bearing after
 * the turn stays within max_bearing_offset, which keeps the marker inside the camera view.
 */
double desired_heading(const DockErrors & errors, const ControlParams & params);

/**
 * Approach command: carrot pursuit onto the marker normal with a field-of-view clamp.
 *
 * Brings the robot from anywhere the marker is visible to final_range, aligned with the normal.
 */
VelocityCommand approach_command(const DockErrors & errors, const ControlParams & params);

/**
 * Final command: slow forward creep with small corrections on psi and e_y.
 *
 * Closes the last stretch to the dock at v_final while removing residual misalignment.
 */
VelocityCommand final_command(const DockErrors & errors, const ControlParams & params);

/**
 * Recover command: reverse, steering so the marker stays in view and the robot drifts back
 * toward the normal line.
 *
 * Gives a misaligned robot room to start a cleaner approach instead of pushing into the dock.
 */
VelocityCommand recover_command(const DockErrors & errors, const ControlParams & params);

/**
 * Speed used whenever the robot reverses (RECOVER and the back-off after losing the marker
 * close to the dock), metres per second, positive.
 *
 * Kept slow so that backing away from the dock stays gentle and the marker stays trackable.
 */
double reverse_speed(const ControlParams & params);

/**
 * Search command: rotate in place toward the side the marker was last seen on.
 *
 * Sweeps the camera round until the goal marker enters the view.
 */
VelocityCommand search_command(double last_bearing, const ControlParams & params);

/**
 * True once the front face is within dock_distance of the marker plane.
 *
 * This is the success condition of the whole docking task.
 */
bool is_docked(const DockErrors & errors, const ControlParams & params);

/**
 * True when lateral and heading errors are small enough to start the final approach.
 *
 * Prevents the robot from creeping into the dock at an angle.
 */
bool gates_pass(const DockErrors & errors, const ControlParams & params);

/**
 * Clamp a command to the velocity limits and to the acceleration limits relative to the
 * previous command, over a step of dt seconds.
 *
 * Keeps the commands that reach /cmd_vel smooth and within what the base can follow.
 */
VelocityCommand limit_command(
  const VelocityCommand & cmd, const VelocityCommand & previous, double dt,
  const ControlParams & params);

/**
 * Exponential smoothing of the marker pose, with reset.
 *
 * Averages out per-frame detector noise so the control law sees a steadier pose.
 */
class PoseFilter
{
public:
  /// Set the smoothing factor; 1.0 passes poses through unchanged.
  explicit PoseFilter(double alpha = 0.5);

  /// Change the smoothing factor without losing the current estimate.
  void set_alpha(double alpha);

  /// Forget the current estimate so the next pose is taken as is.
  void reset();

  /// Blend a new pose into the estimate and return the smoothed pose.
  Eigen::Isometry3d update(const Eigen::Isometry3d & pose);

  /// True once at least one pose has been received since the last reset.
  bool initialized() const {return initialized_;}

private:
  double alpha_;
  bool initialized_ = false;
  Eigen::Isometry3d state_ = Eigen::Isometry3d::Identity();
};

}  // namespace dockbot_docking

#endif  // DOCKBOT_DOCKING__CONTROL_LAW_HPP_
