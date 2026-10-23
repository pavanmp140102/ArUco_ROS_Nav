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
// @Description: ROS node that turns marker poses and a Dock goal into velocity commands.

#include "dockbot_docking/docking_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace dockbot_docking
{

namespace
{

/// Convert a ROS pose to an Eigen transform; the detector's pose is the marker frame in base_link.
Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-9) {
    // An all-zero quaternion is invalid; make the pose non-finite so it is ignored.
    T.translation().x() = std::numeric_limits<double>::quiet_NaN();
    return T;
  }
  T.linear() = q.normalized().toRotationMatrix();
  return T;
}

}  // namespace

DockingController::DockingController(const rclcpp::NodeOptions & options)
: rclcpp::Node("docking_controller", options)
{
  params_ = declare_parameters();
  machine_.set_params(params_);

  markers_sub_ = create_subscription<dockbot_interfaces::msg::ArucoMarkerArray>(
    "aruco/markers", rclcpp::QoS(10),
    std::bind(&DockingController::on_markers, this, std::placeholders::_1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", rclcpp::QoS(10));
  state_pub_ = create_publisher<dockbot_interfaces::msg::DockingState>(
    "docking/state", rclcpp::QoS(10));

  action_server_ = rclcpp_action::create_server<Dock>(
    this, "dock",
    std::bind(&DockingController::on_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DockingController::on_cancel, this, std::placeholders::_1),
    std::bind(&DockingController::on_accepted, this, std::placeholders::_1));

  param_callback_ = add_on_set_parameters_callback(
    std::bind(&DockingController::on_set_parameters, this, std::placeholders::_1));

  create_timer();
  RCLCPP_INFO(get_logger(), "docking_controller ready (control_rate %.1f Hz)", control_rate_);
}

DockingController::~DockingController()
{
  if (machine_.active() && rclcpp::ok()) {
    publish_cmd(VelocityCommand());
  }
}

MachineParams DockingController::declare_parameters()
{
  const MachineParams d;
  double_specs_ = {
    {"dock_distance", d.control.dock_distance, 0.01, 1.0,
      "Target gap between the front face and the marker plane, metres",
      [](MachineParams & p) -> double & {return p.control.dock_distance;}},
    {"front_offset", d.control.front_offset, 0.0, 1.0,
      "Distance from base_link to the front face, metres",
      [](MachineParams & p) -> double & {return p.control.front_offset;}},
    {"final_range", d.control.final_range, 0.2, 3.0,
      "Distance d below which the final approach may start, metres",
      [](MachineParams & p) -> double & {return p.control.final_range;}},
    {"aim_gain", d.control.aim_gain, 0.0, 0.95,
      "Fraction of d at which the aim point sits in front of the marker",
      [](MachineParams & p) -> double & {return p.control.aim_gain;}},
    {"max_bearing_offset", d.control.max_bearing_offset, 0.05, 0.9,
      "Largest allowed marker bearing during approach, radians",
      [](MachineParams & p) -> double & {return p.control.max_bearing_offset;}},
    {"yaw_trust_range", d.control.yaw_trust_range, 0.3, 10.0,
      "Beyond this range the marker orientation is ignored, metres",
      [](MachineParams & p) -> double & {return p.control.yaw_trust_range;}},
    {"v_max", d.control.v_max, 0.01, 1.0, "Linear speed limit, m/s",
      [](MachineParams & p) -> double & {return p.control.v_max;}},
    {"v_final", d.control.v_final, 0.005, 0.3, "Speed during the final approach, m/s",
      [](MachineParams & p) -> double & {return p.control.v_final;}},
    {"w_max", d.control.w_max, 0.05, 3.0, "Angular speed limit, rad/s",
      [](MachineParams & p) -> double & {return p.control.w_max;}},
    {"a_max", d.control.a_max, 0.05, 3.0, "Linear acceleration limit, m/s^2",
      [](MachineParams & p) -> double & {return p.control.a_max;}},
    {"alpha_max", d.control.alpha_max, 0.1, 10.0, "Angular acceleration limit, rad/s^2",
      [](MachineParams & p) -> double & {return p.control.alpha_max;}},
    {"search_angular_speed", d.control.search_angular_speed, 0.05, 2.0,
      "Rotation speed while searching, rad/s",
      [](MachineParams & p) -> double & {return p.control.search_angular_speed;}},
    {"search_timeout", d.search_timeout, 1.0, 600.0,
      "Give up searching after this long, seconds",
      [](MachineParams & p) -> double & {return p.search_timeout;}},
    {"gate_lateral", d.control.gate_lateral, 0.005, 0.5,
      "Largest e_y allowed when entering FINAL, metres",
      [](MachineParams & p) -> double & {return p.control.gate_lateral;}},
    {"gate_heading", d.control.gate_heading, 0.01, 0.8,
      "Largest psi allowed when entering FINAL, radians",
      [](MachineParams & p) -> double & {return p.control.gate_heading;}},
    {"lost_hold_time", d.lost_hold_time, 0.0, 5.0,
      "Keep driving on the last command this long after a dropout, seconds",
      [](MachineParams & p) -> double & {return p.lost_hold_time;}},
    {"lost_timeout", d.lost_timeout, 0.1, 30.0,
      "Return to SEARCH after this long without the target, seconds",
      [](MachineParams & p) -> double & {return p.lost_timeout;}},
    {"pose_stale_time", d.pose_stale_time, 0.05, 5.0,
      "Poses older than this are ignored, seconds",
      [](MachineParams & p) -> double & {return p.pose_stale_time;}},
    {"pose_filter_alpha", d.pose_filter_alpha, 0.05, 1.0,
      "Smoothing factor for the marker pose, 1.0 disables it",
      [](MachineParams & p) -> double & {return p.pose_filter_alpha;}},
    {"recover_range", d.control.recover_range, 0.3, 6.0,
      "Distance to back off to before retrying, metres",
      [](MachineParams & p) -> double & {return p.control.recover_range;}},
    {"dock_timeout", d.dock_timeout, 1.0, 3600.0,
      "Overall time limit for one goal, seconds",
      [](MachineParams & p) -> double & {return p.dock_timeout;}},
    {"turn_gain", d.control.turn_gain, 0.1, 5.0,
      "Turn rate per radian of desired heading during approach, 1/s",
      [](MachineParams & p) -> double & {return p.control.turn_gain;}},
    {"speed_gain", d.control.speed_gain, 0.0, 2.0,
      "Approach speed added per metre beyond final_range, 1/s",
      [](MachineParams & p) -> double & {return p.control.speed_gain;}},
    {"final_heading_gain", d.control.final_heading_gain, 0.0, 5.0,
      "Final approach turn rate per radian of heading error, 1/s",
      [](MachineParams & p) -> double & {return p.control.final_heading_gain;}},
    {"final_lateral_gain", d.control.final_lateral_gain, 0.0, 10.0,
      "Final approach turn rate per metre of lateral error, rad/s/m",
      [](MachineParams & p) -> double & {return p.control.final_lateral_gain;}},
  };
  int_specs_ = {
    {"acquire_frames", d.acquire_frames, 1, 30,
      "Consecutive detections needed to leave SEARCH",
      [](MachineParams & p) -> int & {return p.acquire_frames;}},
    {"max_retries", d.max_retries, 0, 20, "Recovery attempts before failing",
      [](MachineParams & p) -> int & {return p.max_retries;}},
  };

  MachineParams params;
  for (const auto & s : double_specs_) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = s.description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = s.min_value;
    range.to_value = s.max_value;
    desc.floating_point_range.push_back(range);
    s.field(params) = declare_parameter<double>(s.name, s.default_value, desc);
  }
  for (const auto & s : int_specs_) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = s.description;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = s.min_value;
    range.to_value = s.max_value;
    range.step = 1;
    desc.integer_range.push_back(range);
    s.field(params) = declare_parameter<int>(s.name, s.default_value, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Control loop frequency, Hz";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 1.0;
    range.to_value = 200.0;
    desc.floating_point_range.push_back(range);
    control_rate_ = declare_parameter<double>("control_rate", 20.0, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "If zero or greater, start docking to this ID at launch without a goal";
    autostart_marker_id_ = declare_parameter<int>("autostart_marker_id", -1, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Frame ID stamped on /cmd_vel";
    cmd_frame_ = declare_parameter<std::string>("cmd_frame", "base_link", desc);
  }
  return params;
}

rcl_interfaces::msg::SetParametersResult DockingController::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  MachineParams updated = params_;
  bool rate_changed = false;
  for (const auto & p : parameters) {
    bool known = false;
    for (const auto & s : double_specs_) {
      if (s.name == p.get_name() && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        s.field(updated) = p.as_double();
        known = true;
      }
    }
    for (const auto & s : int_specs_) {
      if (s.name == p.get_name() && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        s.field(updated) = static_cast<int>(p.as_int());
        known = true;
      }
    }
    if (known) {
      continue;
    }
    if (p.get_name() == "control_rate" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      rate_changed = p.as_double() != control_rate_;
      control_rate_ = p.as_double();
    }
  }
  if (updated.lost_hold_time > updated.lost_timeout) {
    result.successful = false;
    result.reason = "lost_hold_time must not exceed lost_timeout";
    return result;
  }
  params_ = updated;
  machine_.set_params(params_);
  if (rate_changed) {
    create_timer();
  }
  return result;
}

void DockingController::create_timer()
{
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&DockingController::control_cycle, this));
}

void DockingController::on_markers(
  const dockbot_interfaces::msg::ArucoMarkerArray::ConstSharedPtr & msg)
{
  latest_markers_ = msg;
  markers_consumed_ = false;
}

rclcpp_action::GoalResponse DockingController::on_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Dock::Goal> goal)
{
  if (machine_.active() || goal_handle_) {
    RCLCPP_WARN(get_logger(), "Rejecting goal for marker %d: a goal is already active",
      goal->marker_id);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->marker_id < 0) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: marker_id %d is not valid", goal->marker_id);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->dock_distance < 0.0f || goal->timeout < 0.0f) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: negative dock_distance or timeout");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingController::on_cancel(
  const std::shared_ptr<GoalHandle>)
{
  machine_.cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingController::on_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  GoalRequest request;
  request.marker_id = goal->marker_id;
  request.dock_distance = goal->dock_distance;
  request.timeout = goal->timeout;
  goal_handle_ = goal_handle;
  machine_.start_goal(request, now().seconds());
  RCLCPP_INFO(get_logger(), "Docking to marker %d", request.marker_id);
}

void DockingController::publish_cmd(const VelocityCommand & cmd)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = cmd_frame_;
  msg.twist.linear.x = cmd.v;
  msg.twist.angular.z = cmd.w;
  cmd_pub_->publish(msg);
}

dockbot_interfaces::msg::DockingState DockingController::make_state_msg(
  const StepOutput & out) const
{
  dockbot_interfaces::msg::DockingState msg;
  msg.header.stamp = now();
  msg.header.frame_id = cmd_frame_;
  msg.state = static_cast<uint8_t>(out.state);
  msg.target_id = out.target_id;
  msg.target_visible = out.target_visible;
  if (out.has_errors) {
    const double gap = out.errors.d - machine_.params().control.front_offset -
      machine_.effective_dock_distance();
    msg.distance_to_dock = out.state == State::DOCKED ? 0.0f : static_cast<float>(gap);
    msg.lateral_error = static_cast<float>(out.errors.e_y);
    msg.heading_error = static_cast<float>(out.errors.psi);
  }
  msg.retries = static_cast<uint8_t>(out.retries);
  return msg;
}

void DockingController::finish_goal(const StepOutput & out)
{
  const GoalResult & r = machine_.result();
  RCLCPP_INFO(get_logger(), "Goal finished: %s", r.message.c_str());
  if (!goal_handle_) {
    return;
  }
  auto result = std::make_shared<Dock::Result>();
  result->success = r.outcome == Outcome::DOCKED;
  result->outcome = static_cast<uint8_t>(r.outcome);
  result->message = r.message;
  result->final_distance = static_cast<float>(r.final_distance);
  result->final_lateral_error = static_cast<float>(r.final_lateral_error);
  result->final_heading_error = static_cast<float>(r.final_heading_error);
  (void)out;
  if (r.outcome == Outcome::DOCKED) {
    goal_handle_->succeed(result);
  } else if (r.outcome == Outcome::CANCELED) {
    goal_handle_->canceled(result);
  } else {
    goal_handle_->abort(result);
  }
  goal_handle_.reset();
}

void DockingController::control_cycle()
{
  const double now_s = now().seconds();

  if (autostart_marker_id_ >= 0 && !autostarted_ && !machine_.active()) {
    autostarted_ = true;
    GoalRequest request;
    request.marker_id = autostart_marker_id_;
    machine_.start_goal(request, now_s);
    RCLCPP_INFO(get_logger(), "Autostart: docking to marker %d", autostart_marker_id_);
  }

  StepInput input;
  if (!markers_consumed_ && latest_markers_) {
    markers_consumed_ = true;
    input.new_frame = true;
    for (const auto & m : latest_markers_->markers) {
      MarkerObservation obs;
      obs.id = m.id;
      obs.pose = to_isometry(m.pose);
      obs.stamp = rclcpp::Time(latest_markers_->header.stamp).seconds();
      input.markers.push_back(obs);
    }
  }

  const bool was_active = machine_.active();
  const StepOutput out = machine_.step(input, now_s);
  if (!was_active && !out.finished && out.state == State::IDLE) {
    return;
  }

  publish_cmd(out.cmd);
  state_pub_->publish(make_state_msg(out));

  if (goal_handle_ && goal_handle_->is_active() && !out.finished) {
    auto feedback = std::make_shared<Dock::Feedback>();
    feedback->state = make_state_msg(out);
    goal_handle_->publish_feedback(feedback);
  }
  if (out.finished) {
    finish_goal(out);
  }
}

}  // namespace dockbot_docking

RCLCPP_COMPONENTS_REGISTER_NODE(dockbot_docking::DockingController)
