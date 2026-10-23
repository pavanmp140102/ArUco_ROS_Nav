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

#ifndef DOCKBOT_DOCKING__DOCKING_CONTROLLER_HPP_
#define DOCKBOT_DOCKING__DOCKING_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "dockbot_docking/state_machine.hpp"
#include "dockbot_interfaces/action/dock.hpp"
#include "dockbot_interfaces/msg/aruco_marker_array.hpp"
#include "dockbot_interfaces/msg/docking_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace dockbot_docking
{

/**
 * Docks the robot to the ArUco marker named in a /dock goal, using only /aruco/markers.
 *
 * This node is the last link of the chain camera, detector, controller, base: it owns the
 * action server, runs the ROS-free state machine at a fixed rate, and publishes /cmd_vel.
 * It must run on a single-threaded executor; the callbacks only record requests and the
 * control timer acts on them, so no locks are needed.
 */
class DockingController : public rclcpp::Node
{
public:
  using Dock = dockbot_interfaces::action::Dock;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Dock>;

  /// Declare the parameters, create the subscriptions, publishers, action server and timer.
  explicit DockingController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Publish a zero velocity if a goal is still active, so the robot does not keep moving.
  ~DockingController() override;

private:
  /// Describes one floating-point parameter and where its value lives in MachineParams.
  struct DoubleSpec
  {
    std::string name;
    double default_value;
    double min_value;
    double max_value;
    std::string description;
    std::function<double &(MachineParams &)> field;
  };

  /// Describes one integer parameter and where its value lives in MachineParams.
  struct IntSpec
  {
    std::string name;
    int default_value;
    int min_value;
    int max_value;
    std::string description;
    std::function<int &(MachineParams &)> field;
  };

  /// Declare every parameter with its description and range and return the initial settings.
  MachineParams declare_parameters();

  /// Check and apply live parameter changes so gains can be tuned while the robot runs.
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  /// (Re)create the control timer at the current control_rate.
  void create_timer();

  /// Keep the newest detector message for the next control cycle.
  void on_markers(const dockbot_interfaces::msg::ArucoMarkerArray::ConstSharedPtr & msg);

  /// Accept a Dock goal unless one is already running or its marker ID is invalid.
  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Dock::Goal> goal);

  /// Record a cancel request; the next control cycle stops the robot and ends the goal.
  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle> goal_handle);

  /// Start the accepted goal in the state machine.
  void on_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  /// One control cycle: step the state machine, publish /cmd_vel and the state, finish goals.
  void control_cycle();

  /// Fill a DockingState message from a state machine output.
  dockbot_interfaces::msg::DockingState make_state_msg(const StepOutput & out) const;

  /// Send the action result for a finished goal and release it.
  void finish_goal(const StepOutput & out);

  /// Publish a velocity command with the current stamp.
  void publish_cmd(const VelocityCommand & cmd);

  std::vector<DoubleSpec> double_specs_;
  std::vector<IntSpec> int_specs_;
  MachineParams params_;
  double control_rate_ = 20.0;
  int autostart_marker_id_ = -1;
  bool autostarted_ = false;
  std::string cmd_frame_;

  DockingStateMachine machine_;
  dockbot_interfaces::msg::ArucoMarkerArray::ConstSharedPtr latest_markers_;
  bool markers_consumed_ = true;
  std::shared_ptr<GoalHandle> goal_handle_;

  rclcpp::Subscription<dockbot_interfaces::msg::ArucoMarkerArray>::SharedPtr markers_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<dockbot_interfaces::msg::DockingState>::SharedPtr state_pub_;
  rclcpp_action::Server<Dock>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace dockbot_docking

#endif  // DOCKBOT_DOCKING__DOCKING_CONTROLLER_HPP_
