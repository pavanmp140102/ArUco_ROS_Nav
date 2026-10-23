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
// @Description: ROS-free docking control law: marker pose in base_link to velocity commands.

#include "dockbot_docking/control_law.hpp"

#include <algorithm>
#include <cmath>

namespace dockbot_docking
{

namespace
{

/// Width of the range band over which the marker orientation fades out below yaw_trust_range.
constexpr double kTrustRamp = 0.5;

/// Bearing band below the field-of-view limit over which approach speed ramps down, radians.
constexpr double kEdgeBand = 0.25;

/// Lowest speed factor from the edge-of-view slowdown.
constexpr double kMinEdgeScale = 0.15;

/// Tangent of the angle off the normal at which RECOVER uses its full bearing offset.
constexpr double kRecoverFullOffset = 0.3;

/// Reverse speed in RECOVER, as a multiple of v_final.
constexpr double kRecoverSpeedFactor = 2.0;

/// Wrap an angle to (-pi, pi]; keeps heading differences meaningful near the +-pi seam.
double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

DockErrors compute_errors(const Eigen::Isometry3d & T_base_marker)
{
  DockErrors e;
  const Eigen::Vector2d p = T_base_marker.translation().head<2>();
  const Eigen::Vector3d z = T_base_marker.linear().col(2);
  Eigen::Vector2d n = z.head<2>();
  if (n.norm() < 1e-6) {
    // Marker face points straight up or down: fall back to the direction from marker to robot.
    n = p.norm() > 1e-9 ? Eigen::Vector2d(-p / p.norm()) : Eigen::Vector2d(-1.0, 0.0);
  } else {
    n.normalize();
  }
  const Eigen::Vector2d t(-n.y(), n.x());
  const Eigen::Vector2d r = -p;  // robot position relative to the marker, in base_link axes
  e.d = r.dot(n);
  e.e_y = r.dot(t);
  e.psi = std::atan2(-n.y(), -n.x());
  e.bearing = std::atan2(p.y(), p.x());
  e.range = p.norm();
  return e;
}

double desired_heading(const DockErrors & errors, const ControlParams & params)
{
  const Eigen::Vector2d p(errors.range * std::cos(errors.bearing),
    errors.range * std::sin(errors.bearing));
  const Eigen::Vector2d n(-std::cos(errors.psi), -std::sin(errors.psi));
  const double trust =
    std::clamp((params.yaw_trust_range - errors.range) / kTrustRamp, 0.0, 1.0);
  const Eigen::Vector2d aim = p + trust * params.aim_gain * std::max(errors.d, 0.0) * n;
  const double h = std::atan2(aim.y(), aim.x());
  const double lo = errors.bearing - params.max_bearing_offset;
  const double hi = errors.bearing + params.max_bearing_offset;
  return std::clamp(h, lo, hi);
}

VelocityCommand approach_command(const DockErrors & errors, const ControlParams & params)
{
  const double h = desired_heading(errors, params);
  VelocityCommand cmd;
  cmd.w = std::clamp(params.turn_gain * h, -params.w_max, params.w_max);
  const double speed = std::min(
    params.v_max,
    params.v_final + params.speed_gain * std::max(0.0, errors.range - params.final_range));
  const double c = std::cos(h);
  // Slow down as the marker nears the edge of the view, so the turn catches up with the bearing.
  const double edge = std::clamp(
    (params.max_bearing_offset + 0.06 - std::abs(errors.bearing)) / kEdgeBand, kMinEdgeScale, 1.0);
  cmd.v = c > 0.0 ? speed * c * c * edge : 0.0;
  return cmd;
}

VelocityCommand final_command(const DockErrors & errors, const ControlParams & params)
{
  VelocityCommand cmd;
  cmd.v = params.v_final;
  cmd.w = std::clamp(
    params.final_heading_gain * errors.psi + params.final_lateral_gain * errors.e_y,
    -params.w_max, params.w_max);
  return cmd;
}

VelocityCommand recover_command(const DockErrors & errors, const ControlParams & params)
{
  // Backing away with the marker held off-centre on the robot's side of the normal line swings
  // the line of sight toward the normal; the offset grows with the angle off the normal and is
  // limited to stay inside the field of view.
  const double sign = errors.e_y >= 0.0 ? 1.0 : -1.0;
  const double off_normal = std::abs(errors.e_y) / std::max(errors.d, 0.3);
  const double target_bearing =
    sign * 0.9 * params.max_bearing_offset * std::min(1.0, off_normal / kRecoverFullOffset);
  VelocityCommand cmd;
  cmd.v = -kRecoverSpeedFactor * params.v_final;
  cmd.w = std::clamp(
    params.turn_gain * wrap(errors.bearing - target_bearing), -params.w_max, params.w_max);
  return cmd;
}

VelocityCommand search_command(double last_bearing, const ControlParams & params)
{
  VelocityCommand cmd;
  cmd.w = (last_bearing >= 0.0 ? 1.0 : -1.0) * params.search_angular_speed;
  return cmd;
}

bool is_docked(const DockErrors & errors, const ControlParams & params)
{
  return errors.d - params.front_offset <= params.dock_distance;
}

bool gates_pass(const DockErrors & errors, const ControlParams & params)
{
  return std::abs(errors.e_y) <= params.gate_lateral &&
         std::abs(errors.psi) <= params.gate_heading;
}

VelocityCommand limit_command(
  const VelocityCommand & cmd, const VelocityCommand & previous, double dt,
  const ControlParams & params)
{
  const double dv = params.a_max * dt;
  const double dw = params.alpha_max * dt;
  VelocityCommand out;
  out.v = std::clamp(cmd.v, -params.v_max, params.v_max);
  out.w = std::clamp(cmd.w, -params.w_max, params.w_max);
  out.v = std::clamp(out.v, previous.v - dv, previous.v + dv);
  out.w = std::clamp(out.w, previous.w - dw, previous.w + dw);
  return out;
}

PoseFilter::PoseFilter(double alpha)
: alpha_(alpha) {}

void PoseFilter::set_alpha(double alpha)
{
  alpha_ = alpha;
}

void PoseFilter::reset()
{
  initialized_ = false;
}

Eigen::Isometry3d PoseFilter::update(const Eigen::Isometry3d & pose)
{
  if (!initialized_) {
    state_ = pose;
    initialized_ = true;
    return state_;
  }
  const Eigen::Quaterniond q0(state_.linear());
  const Eigen::Quaterniond q1(pose.linear());
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = (1.0 - alpha_) * state_.translation() + alpha_ * pose.translation();
  out.linear() = q0.slerp(alpha_, q1).normalized().toRotationMatrix();
  state_ = out;
  return state_;
}

}  // namespace dockbot_docking
