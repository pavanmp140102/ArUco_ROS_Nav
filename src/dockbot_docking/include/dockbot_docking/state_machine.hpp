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
// @Description: ROS-free docking state machine: marker detections and a goal to commands.

#ifndef DOCKBOT_DOCKING__STATE_MACHINE_HPP_
#define DOCKBOT_DOCKING__STATE_MACHINE_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

#include "dockbot_docking/control_law.hpp"

namespace dockbot_docking
{

/// Controller states; the values match dockbot_interfaces/DockingState.
enum class State : uint8_t
{
  IDLE = 0,
  SEARCH = 1,
  APPROACH = 2,
  FINAL = 3,
  RECOVER = 4,
  DOCKED = 5,
  FAILED = 6,
};

/// How a goal ended; the values match dockbot_interfaces/Dock result outcomes.
enum class Outcome : uint8_t
{
  DOCKED = 0,
  TARGET_NOT_FOUND = 1,
  TARGET_LOST = 2,
  RETRIES_EXCEEDED = 3,
  TIMEOUT = 4,
  CANCELED = 5,
};

/// One detected marker as the controller receives it: ID, pose in base_link, and image time.
struct MarkerObservation
{
  int id = -1;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double stamp = 0.0;
};

/// Everything that arrives between two control steps.
struct StepInput
{
  /// True when a new detector message arrived since the last step (it may hold no markers).
  bool new_frame = false;
  std::vector<MarkerObservation> markers;
};

/// State machine settings: the control law parameters plus the timing parameters.
struct MachineParams
{
  ControlParams control;
  double search_timeout = 40.0;
  int acquire_frames = 3;
  double lost_hold_time = 0.3;
  double lost_timeout = 1.0;
  double pose_stale_time = 0.5;
  double pose_filter_alpha = 0.5;
  int max_retries = 3;
  double dock_timeout = 120.0;
};

/// A docking request; zero for dock_distance or timeout means "use the parameter".
struct GoalRequest
{
  int marker_id = -1;
  double dock_distance = 0.0;
  double timeout = 0.0;
};

/// What one control step produced.
struct StepOutput
{
  State state = State::IDLE;
  /// Limited command to send to the base.
  VelocityCommand cmd;
  int target_id = -1;
  bool target_visible = false;
  /// Errors from the latest filtered goal pose; meaningful only when has_errors is true.
  DockErrors errors;
  bool has_errors = false;
  int retries = 0;
  /// True on the single step in which the goal reaches DOCKED or FAILED.
  bool finished = false;
};

/// Final report of a goal.
struct GoalResult
{
  Outcome outcome = Outcome::CANCELED;
  std::string message;
  /// Front face to marker plane, last measurement, metres.
  double final_distance = 0.0;
  double final_lateral_error = 0.0;
  double final_heading_error = 0.0;
};

/**
 * The docking state machine: turns marker detections and one goal into a state and a
 * velocity command.
 *
 * It is the decision layer between the detector output and /cmd_vel. Time is passed in, so tests
 * can drive it with a fake clock, and no ROS types are used.
 */
class DockingStateMachine
{
public:
  /// Create an idle machine with the given settings.
  explicit DockingStateMachine(const MachineParams & params = MachineParams());

  /// Replace the settings (live parameter tuning); keeps the current goal and state.
  void set_params(const MachineParams & params);

  /// Current settings.
  const MachineParams & params() const {return params_;}

  /// Start a goal and enter SEARCH; returns false (and changes nothing) if a goal is active.
  bool start_goal(const GoalRequest & request, double now);

  /// Record a cancel request; the next step ends the goal with outcome CANCELED.
  void cancel();

  /// True from start_goal until the step that reports finished; a new goal may start after that.
  bool active() const {return active_ && !finished_pending_;}

  /// Current state.
  State state() const {return state_;}

  /// Dock distance in use for the current goal (the request's value, or the parameter).
  double effective_dock_distance() const {return effective_.dock_distance;}

  /// Result of the most recent finished goal.
  const GoalResult & result() const {return result_;}

  /**
   * Run one control cycle: ingest detections, apply transitions, produce the next command.
   *
   * Goal-ID filtering happens here, so no other ID can affect state or command.
   */
  StepOutput step(const StepInput & input, double now);

private:
  /// Take the goal marker out of the new detections, filter its pose and update the error terms.
  void ingest(const StepInput & input, double now);

  /// End the goal: record the outcome and the last measured errors so the node can report them.
  void fail(Outcome outcome, const std::string & message);

  /// Enter SEARCH (optionally backing off first) so a lost marker can be found again.
  void enter_search(double now, bool back_off);

  /// Enter APPROACH with a pose filter restarted from the newest raw pose.
  void enter_approach();

  /// Enter RECOVER, or fail the goal when the retries are used up; returns false on failure.
  bool enter_recover();

  /// Command for the current state from the latest filtered pose, applying state transitions.
  VelocityCommand fresh_command();

  /// Command while the target is missing: decay the last command, stop, then search again.
  VelocityCommand lost_command(double now);

  MachineParams params_;
  ControlParams effective_;
  PoseFilter filter_;
  State state_ = State::IDLE;
  bool active_ = false;
  bool pending_cancel_ = false;
  bool finished_pending_ = false;
  int goal_id_ = -1;
  double goal_start_ = 0.0;
  double goal_timeout_ = 0.0;
  double search_start_ = 0.0;
  double back_off_until_ = -1.0;
  double last_step_ = 0.0;
  double last_seen_ = -1e9;
  double last_bearing_ = 0.0;
  bool ever_acquired_ = false;
  bool found_this_step_ = false;
  int consecutive_ = 0;
  int retries_ = 0;
  bool has_errors_ = false;
  DockErrors errors_;
  Eigen::Isometry3d last_raw_pose_ = Eigen::Isometry3d::Identity();
  bool has_raw_pose_ = false;
  VelocityCommand last_raw_cmd_;
  VelocityCommand last_cmd_;
  GoalResult result_;
};

}  // namespace dockbot_docking

#endif  // DOCKBOT_DOCKING__STATE_MACHINE_HPP_
