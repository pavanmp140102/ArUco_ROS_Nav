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
// @Date: 2026-10-11
// @Description: ROS-free docking state machine: marker detections and a goal to commands.

#include "dockbot_docking/state_machine.hpp"

#include <algorithm>
#include <cmath>

namespace dockbot_docking
{

namespace
{

/// A pose this recent counts as a live measurement, so commands follow it directly, seconds.
constexpr double kFreshWindow = 0.15;

/// Window in which the target counts as visible in the published state, seconds.
constexpr double kVisibleWindow = 0.2;

/// Largest time step the command limiter integrates over, seconds.
constexpr double kMaxDt = 0.5;

/// Gates are widened by this factor in FINAL before the robot gives up and backs off.
constexpr double kFinalGateSlack = 2.0;

/// True when every element of the pose is finite; NaN poses are treated as no detection.
bool finite(const Eigen::Isometry3d & pose)
{
  return pose.matrix().allFinite();
}

}  // namespace

DockingStateMachine::DockingStateMachine(const MachineParams & params)
: params_(params), effective_(params.control), filter_(params.pose_filter_alpha) {}

void DockingStateMachine::set_params(const MachineParams & params)
{
  const double goal_distance = effective_.dock_distance;
  const bool custom = active() && goal_distance != params_.control.dock_distance;
  params_ = params;
  effective_ = params.control;
  if (custom) {
    effective_.dock_distance = goal_distance;
  }
  filter_.set_alpha(params.pose_filter_alpha);
}

bool DockingStateMachine::start_goal(const GoalRequest & request, double now)
{
  if (active()) {
    return false;
  }
  effective_ = params_.control;
  if (request.dock_distance > 0.0) {
    effective_.dock_distance = request.dock_distance;
  }
  goal_timeout_ = request.timeout > 0.0 ? request.timeout : params_.dock_timeout;
  goal_id_ = request.marker_id;
  goal_start_ = now;
  last_step_ = now;
  last_seen_ = -1e9;
  last_bearing_ = 0.0;
  ever_acquired_ = false;
  retries_ = 0;
  has_errors_ = false;
  has_raw_pose_ = false;
  pending_cancel_ = false;
  finished_pending_ = false;
  last_raw_cmd_ = VelocityCommand();
  last_cmd_ = VelocityCommand();
  filter_.reset();
  active_ = true;
  enter_search(now);
  return true;
}

void DockingStateMachine::cancel()
{
  if (active()) {
    pending_cancel_ = true;
  }
}

void DockingStateMachine::ingest(const StepInput & input, double now)
{
  found_this_step_ = false;
  if (!input.new_frame) {
    return;
  }
  for (const auto & m : input.markers) {
    // Single entry point for detections: only the goal ID gets past here.
    if (m.id != goal_id_ || !finite(m.pose) || now - m.stamp > params_.pose_stale_time) {
      continue;
    }
    found_this_step_ = true;
    last_raw_pose_ = m.pose;
    has_raw_pose_ = true;
    break;
  }
  if (!found_this_step_) {
    consecutive_ = 0;
    return;
  }
  ++consecutive_;
  last_seen_ = now;
  ever_acquired_ = true;
  errors_ = compute_errors(filter_.update(last_raw_pose_));
  has_errors_ = true;
  last_bearing_ = errors_.bearing;
}

void DockingStateMachine::fail(Outcome outcome, const std::string & message)
{
  state_ = outcome == Outcome::DOCKED ? State::DOCKED : State::FAILED;
  result_.outcome = outcome;
  result_.message = message;
  result_.final_distance = has_errors_ ? errors_.d - effective_.front_offset : 0.0;
  result_.final_lateral_error = has_errors_ ? errors_.e_y : 0.0;
  result_.final_heading_error = has_errors_ ? errors_.psi : 0.0;
  finished_pending_ = true;
}

void DockingStateMachine::enter_search(double now)
{
  state_ = State::SEARCH;
  search_start_ = now;
  consecutive_ = 0;
}

void DockingStateMachine::enter_approach()
{
  state_ = State::APPROACH;
  // The filter restarts from the newest raw pose so old samples from before the search
  // cannot pull the estimate.
  filter_.reset();
  errors_ = compute_errors(filter_.update(last_raw_pose_));
}

bool DockingStateMachine::enter_recover()
{
  if (retries_ >= params_.max_retries) {
    fail(Outcome::RETRIES_EXCEEDED, "alignment gates failed after the maximum number of retries");
    return false;
  }
  ++retries_;
  state_ = State::RECOVER;
  return true;
}

VelocityCommand DockingStateMachine::fresh_command(double now)
{
  (void)now;
  switch (state_) {
    case State::APPROACH:
      if (errors_.d < effective_.final_range) {
        if (gates_pass(errors_, effective_)) {
          state_ = State::FINAL;
          return final_command(errors_, effective_);
        }
        if (!enter_recover()) {
          return VelocityCommand();
        }
        return recover_command(errors_, effective_);
      }
      return approach_command(errors_, effective_);
    case State::FINAL:
      if (is_docked(errors_, effective_)) {
        fail(Outcome::DOCKED, "docked");
        return VelocityCommand();
      }
      if (std::abs(errors_.e_y) > kFinalGateSlack * effective_.gate_lateral ||
        std::abs(errors_.psi) > kFinalGateSlack * effective_.gate_heading)
      {
        if (!enter_recover()) {
          return VelocityCommand();
        }
        return recover_command(errors_, effective_);
      }
      return final_command(errors_, effective_);
    case State::RECOVER:
      if (errors_.d >= effective_.recover_range) {
        state_ = State::APPROACH;
        return approach_command(errors_, effective_);
      }
      return recover_command(errors_, effective_);
    default:
      return VelocityCommand();
  }
}

VelocityCommand DockingStateMachine::lost_command(double now)
{
  const double lost = now - last_seen_;
  if (lost > params_.lost_timeout) {
    enter_search(now);
    return search_command(last_bearing_, effective_);
  }
  if (lost > params_.lost_hold_time) {
    return VelocityCommand();
  }
  const double span = std::max(params_.lost_hold_time - kFreshWindow, 1e-6);
  const double scale = std::clamp(1.0 - (lost - kFreshWindow) / span, 0.0, 1.0);
  return VelocityCommand{last_raw_cmd_.v * scale, last_raw_cmd_.w * scale};
}

StepOutput DockingStateMachine::step(const StepInput & input, double now)
{
  StepOutput out;
  out.target_id = goal_id_;
  if (finished_pending_) {
    // The previous step reported the end of the goal; settle back to idle.
    finished_pending_ = false;
    active_ = false;
    state_ = State::IDLE;
    out.target_id = -1;
  }
  if (!active_) {
    out.state = State::IDLE;
    out.target_id = -1;
    return out;
  }

  const double dt = std::clamp(now - last_step_, 0.0, kMaxDt);
  last_step_ = now;
  ingest(input, now);

  VelocityCommand raw;
  if (pending_cancel_) {
    pending_cancel_ = false;
    fail(Outcome::CANCELED, "canceled");
  } else if (now - goal_start_ > goal_timeout_) {
    fail(Outcome::TIMEOUT, "goal timed out");
  } else if (state_ == State::SEARCH) {
    if (consecutive_ >= params_.acquire_frames) {
      enter_approach();
      raw = fresh_command(now);
    } else if (now - search_start_ > params_.search_timeout) {
      fail(
        ever_acquired_ ? Outcome::TARGET_LOST : Outcome::TARGET_NOT_FOUND,
        ever_acquired_ ? "target lost and not found again" : "target not found");
    } else {
      raw = search_command(last_bearing_, effective_);
    }
  } else if (now - last_seen_ <= kFreshWindow) {
    raw = fresh_command(now);
    last_raw_cmd_ = raw;
  } else {
    raw = lost_command(now);
  }

  const bool terminal = state_ == State::DOCKED || state_ == State::FAILED;
  if (terminal) {
    last_cmd_ = VelocityCommand();
  } else {
    last_cmd_ = limit_command(raw, last_cmd_, dt, effective_);
  }

  out.state = state_;
  out.cmd = last_cmd_;
  out.target_visible = now - last_seen_ <= kVisibleWindow;
  out.errors = errors_;
  out.has_errors = has_errors_;
  out.retries = retries_;
  out.finished = terminal;
  return out;
}

}  // namespace dockbot_docking
